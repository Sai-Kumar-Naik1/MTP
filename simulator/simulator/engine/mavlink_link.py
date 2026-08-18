"""Real MAVLink link to one vehicle (ArduPilot/PX4 SITL or a real autopilot).

This is the only place in the project that encodes or decodes MAVLink. It
drives the single supported operation - fly from a start point to a destination
- as an explicit state machine over genuine protocol traffic:

    HEARTBEAT -> set GUIDED -> ARM -> NAV_TAKEOFF -> SET_POSITION_TARGET -> LAND

and reports progress back as the plain `DroneTelemetry` schema, so nothing
above this module needs to know a MAVLink message from a JSON object.

Preconditions and exceptions are expressed as protocol events, not as local
guesses: an arm refusal arrives as COMMAND_ACK/MAV_RESULT_DENIED, GPS loss as
GPS_RAW_INT.fix_type dropping below a 2D fix, low battery as SYS_STATUS.

Runs on a worker thread - every call here blocks on socket I/O and must never
touch the GUI thread.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional

from pymavlink import mavutil

from contracts.gui_orchestration import (
    DroneConfig,
    DroneStatus,
    DroneTelemetry,
    FaultType,
    LatLon,
)

# ArduPilot Copter custom flight modes (MAV_CMD_DO_SET_MODE custom_mode).
COPTER_MODE_GUIDED = 4
COPTER_MODE_LAND = 9
COPTER_MODE_RTL = 6
COPTER_MODE_BRAKE = 17

ARRIVAL_RADIUS_M = 3.0        # close enough to call the waypoint reached
TAKEOFF_TOLERANCE = 0.95      # fraction of target altitude that counts as up
HEARTBEAT_TIMEOUT_S = 5.0     # no heartbeat for this long = link lost
COMMAND_ACK_TIMEOUT_S = 3.0
GPS_MIN_FIX = 2               # MAV_GPS_FIX_TYPE_2D_FIX
LOW_BATTERY_PCT = 20.0

# Only position is commanded in SET_POSITION_TARGET_GLOBAL_INT; every other
# axis (velocity, accel, yaw) is masked off and left to the autopilot.
POSITION_TYPE_MASK = 0b0000_1111_1111_1000


class LinkState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ARMING = "arming"
    TAKING_OFF = "taking_off"
    CRUISING = "cruising"
    LANDING = "landing"
    DONE = "done"
    FAILED = "failed"


@dataclass
class VehicleSnapshot:
    """Latest decoded state of one vehicle, rebuilt from inbound messages."""

    lat: float = 0.0
    lon: float = 0.0
    relative_alt_m: float = 0.0
    heading_deg: float = 0.0
    ground_speed_mps: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    battery_pct: float = 100.0
    gps_fix_type: int = 0
    satellites: int = 0
    armed: bool = False
    custom_mode: int = 0
    last_heartbeat_s: float = 0.0
    last_message: str = ""

    @property
    def position(self) -> LatLon:
        return LatLon(lat=self.lat, lon=self.lon)

    @property
    def has_gps(self) -> bool:
        return self.gps_fix_type >= GPS_MIN_FIX


def haversine_m(a: LatLon, b: LatLon) -> float:
    """Great-circle distance in metres."""
    lat1, lat2 = math.radians(a.lat), math.radians(b.lat)
    dlat = lat2 - lat1
    dlon = math.radians(b.lon - a.lon)
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * 6_371_000.0 * math.asin(min(1.0, math.sqrt(h)))


class MavlinkCommandError(RuntimeError):
    """An autopilot refused a command, or never acknowledged it."""


class MavlinkVehicle:
    """One MAVLink connection, driving one vehicle through one mission."""

    def __init__(self, config: DroneConfig, endpoint: str, source_system: int = 255):
        self.config = config
        self.endpoint = endpoint
        self.source_system = source_system
        self.master: Optional[mavutil.mavfile] = None
        self.snapshot = VehicleSnapshot()
        self.state = LinkState.DISCONNECTED
        self.home: Optional[LatLon] = None
        self.target: Optional[LatLon] = None
        self.last_error: str = ""
        self._raw_log: list[str] = []

    # ---- Connection ----

    def connect(self, timeout_s: float = 30.0) -> bool:
        """Open the link and wait for the vehicle's first HEARTBEAT."""
        self.state = LinkState.CONNECTING
        self.master = mavutil.mavlink_connection(
            self.endpoint, source_system=self.source_system, autoreconnect=True
        )
        # An outbound UDP socket has no peer until it transmits, so receiving
        # first fails. Announce ourselves as a GCS, which we should be doing
        # regardless, and that establishes the path both ways.
        self.send_gcs_heartbeat()
        heartbeat = self.master.wait_heartbeat(timeout=timeout_s)
        if heartbeat is None:
            self.state = LinkState.FAILED
            self.last_error = f"no HEARTBEAT from {self.endpoint} within {timeout_s:.0f}s"
            return False
        self.snapshot.last_heartbeat_s = time.time()
        self.state = LinkState.CONNECTED
        self._request_streams()
        return True

    def send_gcs_heartbeat(self) -> None:
        """Identify ourselves as a ground station. Autopilots expect this, and
        it is what opens the return path on an outbound UDP socket."""
        if self.master is None:
            return
        self.master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def close(self) -> None:
        if self.master is not None:
            self.master.close()
            self.master = None
        self.state = LinkState.DISCONNECTED

    def _request_streams(self, rate_hz: int = 4) -> None:
        """Ask the autopilot for the telemetry the dashboard needs."""
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            rate_hz,
            1,  # start
        )

    # ---- Inbound ----

    def pump(self, budget_s: float = 0.05) -> None:
        """Drain waiting messages into `snapshot`. Non-blocking-ish."""
        if self.master is None:
            return
        deadline = time.time() + budget_s
        while time.time() < deadline:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                return
            self._apply(msg)

    def _apply(self, msg) -> None:
        kind = msg.get_type()
        snap = self.snapshot

        if kind == "HEARTBEAT":
            snap.last_heartbeat_s = time.time()
            snap.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            snap.custom_mode = msg.custom_mode
        elif kind == "GLOBAL_POSITION_INT":
            snap.lat = msg.lat / 1e7
            snap.lon = msg.lon / 1e7
            snap.relative_alt_m = msg.relative_alt / 1000.0
            snap.heading_deg = (msg.hdg / 100.0) % 360.0
            snap.ground_speed_mps = math.hypot(msg.vx, msg.vy) / 100.0
        elif kind == "ATTITUDE":
            snap.roll_deg = math.degrees(msg.roll)
            snap.pitch_deg = math.degrees(msg.pitch)
        elif kind == "SYS_STATUS":
            if msg.battery_remaining >= 0:
                snap.battery_pct = float(msg.battery_remaining)
        elif kind == "GPS_RAW_INT":
            snap.gps_fix_type = msg.fix_type
            snap.satellites = msg.satellites_visible
        else:
            return

        snap.last_message = f"{kind} {msg.to_dict()}"
        self._raw_log.append(snap.last_message)
        if len(self._raw_log) > 200:
            del self._raw_log[:100]

    def take_raw_log(self) -> list[str]:
        """Decoded messages seen since the last call (for the MAVLink log view)."""
        entries, self._raw_log = self._raw_log, []
        return entries

    @property
    def link_alive(self) -> bool:
        if not self.snapshot.last_heartbeat_s:
            return False
        return (time.time() - self.snapshot.last_heartbeat_s) < HEARTBEAT_TIMEOUT_S

    # ---- Outbound commands ----

    def _send_command_long(self, command: int, *params: float) -> None:
        padded = list(params) + [0.0] * (7 - len(params))
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            command,
            0,  # confirmation
            *padded[:7],
        )

    def _await_ack(self, command: int, timeout_s: float = COMMAND_ACK_TIMEOUT_S) -> int:
        """Block for this command's COMMAND_ACK and return its MAV_RESULT."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self.master.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.command != command:
                self._apply(msg)
                continue
            self._raw_log.append(f"COMMAND_ACK {msg.to_dict()}")
            return msg.result
        raise MavlinkCommandError(f"no COMMAND_ACK for command {command} within {timeout_s:.0f}s")

    def set_mode(self, custom_mode: int) -> None:
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            custom_mode,
        )

    def arm(self) -> None:
        """Arm the vehicle. Raises if the autopilot refuses (pre-arm failure)."""
        self.state = LinkState.ARMING
        self._send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        result = self._await_ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.state = LinkState.FAILED
            self.last_error = f"arm refused by autopilot (MAV_RESULT={result})"
            raise MavlinkCommandError(self.last_error)

    def disarm(self) -> None:
        self._send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)

    def takeoff(self, altitude_m: float) -> None:
        self.state = LinkState.TAKING_OFF
        self._send_command_long(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude_m
        )
        result = self._await_ack(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self.state = LinkState.FAILED
            self.last_error = f"takeoff refused (MAV_RESULT={result})"
            raise MavlinkCommandError(self.last_error)

    def goto(self, destination: LatLon, altitude_m: float) -> None:
        """Command a position target. Re-sending this is how re-routing works."""
        self.target = destination
        self.state = LinkState.CRUISING
        self.master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms, autopilot fills in
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            POSITION_TYPE_MASK,
            int(destination.lat * 1e7),
            int(destination.lon * 1e7),
            altitude_m,
            0, 0, 0,      # velocity
            0, 0, 0,      # acceleration
            0, 0,         # yaw, yaw_rate
        )

    def land(self) -> None:
        self.state = LinkState.LANDING
        self.set_mode(COPTER_MODE_LAND)

    def return_to_launch(self) -> None:
        """Fall back to the launch point - the autopilot's own RTL."""
        self.state = LinkState.LANDING
        self._send_command_long(mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH)

    def hold_position(self) -> None:
        """Hold station where we are (used when GPS is lost and the controller
        declines to fall back)."""
        self.set_mode(COPTER_MODE_BRAKE)

    # ---- Derived state ----

    def distance_to_target_m(self) -> float:
        if self.target is None:
            return 0.0
        return haversine_m(self.snapshot.position, self.target)

    def drone_status(self) -> DroneStatus:
        """Map link state and autopilot flags onto the shared status enum."""
        snap = self.snapshot
        if self.state is LinkState.FAILED:
            return DroneStatus.FAILSAFE
        if self.state is LinkState.DONE:
            return DroneStatus.LANDED
        if not snap.armed:
            return DroneStatus.LANDED if self.state is LinkState.LANDING else DroneStatus.STANDBY
        if self.state is LinkState.TAKING_OFF:
            return DroneStatus.TAKING_OFF
        if self.state is LinkState.LANDING:
            return DroneStatus.LANDING
        if self.state is LinkState.CRUISING:
            return DroneStatus.IN_FLIGHT
        return DroneStatus.ARMED

    def telemetry(self) -> DroneTelemetry:
        """The snapshot, in the schema everything above this layer speaks."""
        snap = self.snapshot
        faults: list[FaultType] = []
        if not snap.has_gps:
            faults.append(FaultType.GPS_LOSS)
        if not self.link_alive:
            faults.append(FaultType.COMMS_LOSS)

        return DroneTelemetry(
            sysid=self.config.sysid,
            status=self.drone_status(),
            lat=snap.lat,
            lon=snap.lon,
            altitude_m=snap.relative_alt_m,
            heading_deg=snap.heading_deg,
            ground_speed_mps=snap.ground_speed_mps,
            roll_deg=snap.roll_deg,
            pitch_deg=snap.pitch_deg,
            battery_pct=max(0.0, min(100.0, snap.battery_pct)),
            link_quality_pct=100.0 if self.link_alive else 0.0,
            active_faults=faults,
            raw_mavlink=snap.last_message or None,
        )
