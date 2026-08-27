"""Flight controller interface and firmware models.

    MAVLink Message -> Flight Controller -> ArduPilot / PX4

Two implementations behind one interface:

`SimulatedFlightController` is an ArduPilot-shaped firmware model. It decodes
real MAVLink frames, enforces arming and mode rules, executes an uploaded
waypoint mission, integrates the airframe, and reports back with real
HEARTBEAT / GLOBAL_POSITION_INT / SYS_STATUS / GPS_RAW_INT messages. It exists
so the whole stack is runnable and testable without SITL installed.

`MavlinkFlightController` is the same interface over a UDP socket to a genuine
ArduPilot or PX4. Swapping one for the other changes nothing above this layer -
the drone thread only ever hands down frames and reads back state.
"""
from __future__ import annotations
import math
import socket
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional

from pymavlink.dialects.v20 import ardupilotmega as mav2

from engine.mavlink_interpreter import (
    MODE_AUTO,
    MODE_BRAKE,
    MODE_GUIDED,
    MODE_LAND,
    MODE_LOITER,
    MODE_NAMES,
    MODE_RTL,
    MODE_STABILIZE,
)

EARTH_RADIUS_M = 6_371_000.0
CLIMB_RATE_MPS = 3.0
DESCENT_RATE_MPS = 2.0
WAYPOINT_RADIUS_M = 5.0
NOMINAL_DRAW_A = 20.0
AUTOPILOT_SYSID = 1
AUTOPILOT_COMPID = 1


@dataclass
class FirmwareState:
    """What the flight controller reports about the airframe."""

    lat: float = 0.0
    lon: float = 0.0
    relative_alt_m: float = 0.0
    heading_deg: float = 0.0
    ground_speed_mps: float = 0.0
    vertical_speed_mps: float = 0.0
    battery_pct: float = 100.0
    armed: bool = False
    mode: int = MODE_STABILIZE
    gps_fix_type: int = 3
    mission_seq: int = 0
    mission_complete: bool = False
    landed: bool = False

    @property
    def mode_name(self) -> str:
        return MODE_NAMES.get(self.mode, f"MODE_{self.mode}")


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlat = p2 - p1
    dlon = math.radians(lon2 - lon1)
    h = math.sin(dlat / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    y = math.sin(dlon) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


class FlightController(ABC):
    """Everything above this line speaks MAVLink frames and nothing else."""

    @abstractmethod
    def send(self, frame: bytes) -> None:
        """Hand a MAVLink frame down to the firmware."""

    @abstractmethod
    def receive(self) -> list:
        """Decoded messages the firmware has sent up since the last call."""

    @abstractmethod
    def state(self) -> FirmwareState:
        """Latest airframe state."""

    def update(self, dt: float) -> None:
        """Advance the firmware. A no-op against real hardware, which has its
        own clock."""


class SimulatedFlightController(FlightController):
    """ArduPilot-shaped firmware: decodes MAVLink, flies the plan, reports back."""

    def __init__(
        self,
        sysid: int = AUTOPILOT_SYSID,
        home_lat: float = 0.0,
        home_lon: float = 0.0,
        cruise_speed_mps: float = 15.0,
        battery_capacity_mah: float = 5200.0,
        verbose: bool = False,
    ):
        self.sysid = sysid
        self.home_lat = home_lat
        self.home_lon = home_lon
        self.cruise_speed_mps = max(0.1, cruise_speed_mps)
        self.battery_capacity_mah = max(1.0, battery_capacity_mah)
        self.verbose = verbose

        self._rx = mav2.MAVLink(None, srcSystem=sysid, srcComponent=AUTOPILOT_COMPID)
        self._rx.robust_parsing = True
        self._tx = mav2.MAVLink(None, srcSystem=sysid, srcComponent=AUTOPILOT_COMPID)
        self._tx.robust_parsing = True

        self.st = FirmwareState(lat=home_lat, lon=home_lon)
        self._consumed_mah = 0.0
        self._outbox: list = []

        # Mission storage and the upload conversation.
        self._mission: list = []
        self._expected_count = 0
        self._receiving = False
        self._target_alt_m = 0.0

        self.commands_seen: list[str] = []
        self.frames_decoded = 0

    # ---- Downlink: frames arriving from the interpreter ----

    def send(self, frame: bytes) -> None:
        messages = self._rx.parse_buffer(frame) or []
        for message in messages:
            self.frames_decoded += 1
            self._handle(message)

    def _handle(self, message) -> None:
        kind = message.get_type()

        if kind == "COMMAND_LONG":
            self._handle_command_long(message)
        elif kind == "MISSION_COUNT":
            self._mission = []
            self._expected_count = message.count
            self._receiving = True
            self.commands_seen.append(f"MISSION_COUNT({message.count})")
            self._emit(self._tx.mission_request_int_encode(
                message.get_srcSystem(), message.get_srcComponent(), 0,
                mav2.MAV_MISSION_TYPE_MISSION))
        elif kind in ("MISSION_ITEM_INT", "MISSION_ITEM"):
            self._handle_mission_item(message)
        elif kind == "SET_POSITION_TARGET_GLOBAL_INT":
            # Guided-mode target, accepted as an immediate single waypoint.
            self._target_alt_m = message.alt
            self._mission = [(message.lat_int / 1e7, message.lon_int / 1e7, message.alt)]
            self.st.mission_seq = 0
            self.st.mission_complete = False
            self.commands_seen.append("SET_POSITION_TARGET_GLOBAL_INT")

    def _handle_command_long(self, message) -> None:
        command = message.command
        result = mav2.MAV_RESULT_ACCEPTED

        if command == mav2.MAV_CMD_DO_SET_MODE:
            self.st.mode = int(message.param2)
            self.commands_seen.append(f"DO_SET_MODE({self.st.mode_name})")
        elif command == mav2.MAV_CMD_COMPONENT_ARM_DISARM:
            want_armed = bool(message.param1)
            if want_armed and self.st.battery_pct <= 0:
                result = mav2.MAV_RESULT_DENIED  # pre-arm: no battery
            elif want_armed and self.st.gps_fix_type < 2:
                result = mav2.MAV_RESULT_DENIED  # pre-arm: no GPS lock
            else:
                self.st.armed = want_armed
                if not want_armed:
                    self.st.ground_speed_mps = 0.0
            self.commands_seen.append(
                f"ARM_DISARM({int(want_armed)})"
                + ("" if result == mav2.MAV_RESULT_ACCEPTED else " DENIED")
            )
        elif command == mav2.MAV_CMD_NAV_TAKEOFF:
            if not self.st.armed:
                result = mav2.MAV_RESULT_TEMPORARILY_REJECTED
            else:
                self._target_alt_m = message.param7
                self.st.landed = False
            self.commands_seen.append(f"NAV_TAKEOFF({message.param7:.0f}m)")
        elif command == mav2.MAV_CMD_NAV_RETURN_TO_LAUNCH:
            self.st.mode = MODE_RTL
            self._mission = [(self.home_lat, self.home_lon, self._target_alt_m)]
            self.st.mission_seq = 0
            self.st.mission_complete = False
            self.commands_seen.append("NAV_RETURN_TO_LAUNCH")
        else:
            result = mav2.MAV_RESULT_UNSUPPORTED

        self._emit(self._tx.command_ack_encode(command, result))

    def _handle_mission_item(self, message) -> None:
        if not self._receiving:
            return
        lat = message.x / 1e7 if abs(message.x) > 1000 else float(message.x)
        lon = message.y / 1e7 if abs(message.y) > 1000 else float(message.y)
        self._mission.append((lat, lon, message.z))
        if message.command == mav2.MAV_CMD_NAV_TAKEOFF:
            self._target_alt_m = message.z
        self.commands_seen.append(f"MISSION_ITEM_INT(seq={message.seq})")

        next_seq = len(self._mission)
        if next_seq < self._expected_count:
            self._emit(self._tx.mission_request_int_encode(
                message.get_srcSystem(), message.get_srcComponent(), next_seq,
                mav2.MAV_MISSION_TYPE_MISSION))
        else:
            self._receiving = False
            self.st.mission_seq = 0
            self.st.mission_complete = False
            self._emit(self._tx.mission_ack_encode(
                message.get_srcSystem(), message.get_srcComponent(),
                mav2.MAV_MISSION_ACCEPTED, mav2.MAV_MISSION_TYPE_MISSION))

    # ---- Uplink ----

    def _emit(self, message) -> None:
        self._outbox.append(message)

    def receive(self) -> list:
        out, self._outbox = self._outbox, []
        return out

    def state(self) -> FirmwareState:
        return self.st

    # ---- Airframe integration ----

    def update(self, dt: float) -> None:
        """One firmware step: burn power, then fly according to the mode."""
        st = self.st
        if st.armed:
            self._consumed_mah += NOMINAL_DRAW_A * (dt / 3600.0) * 1000.0
            st.battery_pct = max(
                0.0, 100.0 * (1.0 - self._consumed_mah / self.battery_capacity_mah)
            )
            if st.battery_pct <= 0.0:
                st.armed = False
                st.ground_speed_mps = 0.0

        if not st.armed:
            self._telemetry_burst()
            return

        if st.mode in (MODE_BRAKE, MODE_LOITER):
            st.ground_speed_mps = 0.0
            st.vertical_speed_mps = 0.0
        elif st.mode == MODE_LAND:
            self._descend(dt)
        elif st.mode in (MODE_AUTO, MODE_GUIDED, MODE_RTL):
            self._fly_mission(dt)

        self._telemetry_burst()

    def _fly_mission(self, dt: float) -> None:
        st = self.st
        if st.relative_alt_m < self._target_alt_m - 0.1:
            st.relative_alt_m = min(
                self._target_alt_m, st.relative_alt_m + CLIMB_RATE_MPS * dt
            )
            st.vertical_speed_mps = CLIMB_RATE_MPS
            st.ground_speed_mps = 0.0
            return

        st.vertical_speed_mps = 0.0
        if st.mission_seq >= len(self._mission):
            st.mission_complete = True
            if st.mode == MODE_RTL:
                self._descend(dt)
            else:
                st.ground_speed_mps = 0.0
            return

        lat, lon, alt = self._mission[st.mission_seq]
        if lat == 0.0 and lon == 0.0:
            st.mission_seq += 1  # takeoff item, already satisfied by the climb
            return

        remaining = _haversine_m(st.lat, st.lon, lat, lon)
        if remaining <= WAYPOINT_RADIUS_M:
            st.lat, st.lon = lat, lon
            st.mission_seq += 1
            if st.mission_seq >= len(self._mission):
                st.mission_complete = True
                st.ground_speed_mps = 0.0
                st.mode = MODE_LAND  # ArduPilot's land-at-end behaviour
            return

        step = self.cruise_speed_mps * dt
        fraction = min(1.0, step / remaining)
        st.lat += (lat - st.lat) * fraction
        st.lon += (lon - st.lon) * fraction
        st.heading_deg = _bearing_deg(st.lat, st.lon, lat, lon)
        st.ground_speed_mps = self.cruise_speed_mps

    def _descend(self, dt: float) -> None:
        st = self.st
        st.ground_speed_mps = 0.0
        st.relative_alt_m = max(0.0, st.relative_alt_m - DESCENT_RATE_MPS * dt)
        st.vertical_speed_mps = -DESCENT_RATE_MPS
        if st.relative_alt_m <= 0.0:
            st.vertical_speed_mps = 0.0
            st.armed = False
            st.landed = True

    def _telemetry_burst(self) -> None:
        """What a real autopilot streams back."""
        st = self.st
        base_mode = mav2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        if st.armed:
            base_mode |= mav2.MAV_MODE_FLAG_SAFETY_ARMED
        self._emit(self._tx.heartbeat_encode(
            mav2.MAV_TYPE_QUADROTOR, mav2.MAV_AUTOPILOT_ARDUPILOTMEGA,
            base_mode, st.mode, mav2.MAV_STATE_ACTIVE))
        self._emit(self._tx.global_position_int_encode(
            0, int(st.lat * 1e7), int(st.lon * 1e7),
            int(st.relative_alt_m * 1000), int(st.relative_alt_m * 1000),
            0, 0, int(st.vertical_speed_mps * -100), int(st.heading_deg * 100)))
        self._emit(self._tx.sys_status_encode(
            0, 0, 0, 0, 12000, -1, int(st.battery_pct), 0, 0, 0, 0, 0, 0))
        self._emit(self._tx.gps_raw_int_encode(
            0, st.gps_fix_type, int(st.lat * 1e7), int(st.lon * 1e7),
            int(st.relative_alt_m * 1000), 100, 100,
            int(st.ground_speed_mps * 100), int(st.heading_deg * 100), 12))

    # ---- Fault injection against the firmware, not the model above it ----

    def set_gps(self, available: bool) -> None:
        """SITL's SIM_GPS_DISABLE, in miniature."""
        self.st.gps_fix_type = 3 if available else 0


class MavlinkFlightController(FlightController):
    """The same interface over UDP to a genuine ArduPilot or PX4 (SITL).

    Binds a local port and learns the autopilot's address from its first
    packet, which is how ArduPilot's `--out=udp:host:port` stream behaves. A
    GCS heartbeat goes out periodically: autopilots expect one, and on an
    outbound UDP socket it is what opens the return path.
    """

    def __init__(
        self,
        endpoint: Optional[tuple[str, int]] = None,
        bind_host: str = "0.0.0.0",
        bind_port: int = 14550,
        heartbeat_interval_s: float = 1.0,
    ):
        self.endpoint = endpoint
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((bind_host, bind_port))
        self._sock.setblocking(False)
        self._rx = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
        self._rx.robust_parsing = True
        self._tx = mav2.MAVLink(None, srcSystem=255, srcComponent=190)
        self._tx.robust_parsing = True

        self.st = FirmwareState()
        self.bind_port = bind_port
        self.heartbeat_interval_s = heartbeat_interval_s
        self._last_heartbeat = 0.0
        self._streams_requested = False
        self.connected = False
        self.frames_sent = 0
        self.messages_seen = 0

        if endpoint is not None:
            self._announce()

    # ---- Downlink ----

    def send(self, frame: bytes) -> None:
        if self.endpoint is None:
            return  # nowhere to send yet; the autopilot has not been heard from
        try:
            self._sock.sendto(frame, self.endpoint)
            self.frames_sent += 1
        except OSError:
            pass

    def _announce(self) -> None:
        """Identify as a ground station, opening the return path."""
        heartbeat = self._tx.heartbeat_encode(
            mav2.MAV_TYPE_GCS, mav2.MAV_AUTOPILOT_INVALID, 0, 0, mav2.MAV_STATE_ACTIVE
        )
        self.send(heartbeat.pack(self._tx))

    def _request_streams(self, rate_hz: int = 4) -> None:
        request = self._tx.request_data_stream_encode(
            1, 1, mav2.MAV_DATA_STREAM_ALL, rate_hz, 1
        )
        self.send(request.pack(self._tx))
        self._streams_requested = True

    # ---- Uplink ----

    def receive(self) -> list:
        messages = []
        while True:
            try:
                raw, sender = self._sock.recvfrom(65535)
            except (BlockingIOError, OSError):
                break
            if self.endpoint is None:
                # First contact: this is where the autopilot lives.
                self.endpoint = sender
                self.connected = True
                self._announce()
            for message in self._rx.parse_buffer(raw) or []:
                self.messages_seen += 1
                self._absorb(message)
                messages.append(message)
        return messages

    def _absorb(self, message) -> None:
        kind = message.get_type()
        st = self.st
        if kind == "HEARTBEAT":
            self.connected = True
            was_armed = st.armed
            st.armed = bool(message.base_mode & mav2.MAV_MODE_FLAG_SAFETY_ARMED)
            st.mode = message.custom_mode
            # Disarming on the ground after a descent is a landing.
            if was_armed and not st.armed and st.relative_alt_m < 1.0:
                st.landed = True
        elif kind == "GLOBAL_POSITION_INT":
            st.lat = message.lat / 1e7
            st.lon = message.lon / 1e7
            st.relative_alt_m = message.relative_alt / 1000.0
            st.heading_deg = (message.hdg / 100.0) % 360.0
            st.ground_speed_mps = math.hypot(message.vx, message.vy) / 100.0
            st.vertical_speed_mps = -message.vz / 100.0
        elif kind == "SYS_STATUS" and message.battery_remaining >= 0:
            st.battery_pct = float(message.battery_remaining)
        elif kind == "GPS_RAW_INT":
            st.gps_fix_type = message.fix_type
        elif kind == "MISSION_CURRENT":
            st.mission_seq = message.seq
        elif kind == "MISSION_ITEM_REACHED":
            st.mission_seq = message.seq + 1

    def update(self, dt: float) -> None:
        """Real firmware keeps its own clock; only the link needs servicing."""
        now = time.time()
        if now - self._last_heartbeat >= self.heartbeat_interval_s:
            self._last_heartbeat = now
            self._announce()
            if self.endpoint is not None and not self._streams_requested:
                self._request_streams()

    def state(self) -> FirmwareState:
        return self.st

    def close(self) -> None:
        self._sock.close()
