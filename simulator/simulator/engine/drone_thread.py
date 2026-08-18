"""Independent drone thread - one UAV, one thread, one private state.

Each drone owns its state and runs its own loop:

    while running:
        receive_command()      # drain the inbox
        update_state()         # integrate motion and battery
        monitor_health()       # battery, comms, GPS
        execute_operation()    # advance the mission state machine
        publish_telemetry()    # emit an immutable snapshot

Nothing outside the thread ever mutates that state. The only ways in and out
are a command inbox (`queue.Queue`) and a telemetry callback, so a drone can
later be moved to another process or another machine without the callers
noticing.

Deliberately free of Qt and of MAVLink: this is the node itself. Transport
adapters (`engine.mavlink_link`) and GUI plumbing (`services.*`) sit outside.
"""
from __future__ import annotations

import math
import queue
import threading
import time
from dataclasses import dataclass, field, replace
from enum import Enum
from typing import Callable, Optional

EARTH_RADIUS_M = 6_371_000.0
DEFAULT_TICK_S = 0.1          # 10 Hz loop
CLIMB_RATE_MPS = 3.0
DESCENT_RATE_MPS = 2.0
NOMINAL_DRAW_A = 20.0         # cruise current, for battery accounting
ARRIVAL_RADIUS_M = 5.0
LOW_BATTERY_PCT = 20.0
CRITICAL_BATTERY_PCT = 5.0
COMMS_TIMEOUT_S = 5.0


# --------------------------------------------------------------------------
# State vocabulary
# --------------------------------------------------------------------------


class MissionState(str, Enum):
    IDLE = "IDLE"
    ARMED = "ARMED"
    TAKEOFF = "TAKEOFF"
    EN_ROUTE = "EN_ROUTE"
    HOLDING = "HOLDING"
    LANDING = "LANDING"
    LANDED = "LANDED"
    ABORTED = "ABORTED"
    FAILED = "FAILED"


class HealthStatus(str, Enum):
    NOMINAL = "NOMINAL"
    DEGRADED = "DEGRADED"      # low battery, or GPS lost
    CRITICAL = "CRITICAL"      # nearly flat
    FAILED = "FAILED"          # battery exhausted in flight


class CommunicationStatus(str, Enum):
    ONLINE = "ONLINE"
    DEGRADED = "DEGRADED"
    LOST = "LOST"


class Operation(str, Enum):
    NONE = "NONE"
    GOTO = "GOTO"              # the one supported operation: source -> destination
    RETURN_TO_LAUNCH = "RETURN_TO_LAUNCH"
    LAND = "LAND"


class CommandType(str, Enum):
    ARM = "ARM"
    DISARM = "DISARM"
    TAKEOFF = "TAKEOFF"
    GOTO = "GOTO"
    SET_DESTINATION = "SET_DESTINATION"   # re-route in flight
    HOLD = "HOLD"
    RESUME = "RESUME"
    RETURN_TO_LAUNCH = "RETURN_TO_LAUNCH"
    LAND = "LAND"
    ABORT = "ABORT"
    SET_GPS = "SET_GPS"                   # fault injection
    HEARTBEAT = "HEARTBEAT"
    SHUTDOWN = "SHUTDOWN"


@dataclass(frozen=True)
class Command:
    type: CommandType
    payload: dict = field(default_factory=dict)


@dataclass(frozen=True)
class Position:
    lat: float = 0.0
    lon: float = 0.0
    alt_m: float = 0.0


@dataclass(frozen=True)
class Velocity:
    ground_speed_mps: float = 0.0
    heading_deg: float = 0.0
    vertical_speed_mps: float = 0.0


@dataclass(frozen=True)
class DroneSnapshot:
    """Immutable copy of a drone's state, safe to hand to any other thread."""

    drone_id: int
    name: str
    position: Position
    velocity: Velocity
    battery: float
    health_status: HealthStatus
    communication_status: CommunicationStatus
    armed_status: bool
    current_operation: Operation
    source: Optional[Position]
    destination: Optional[Position]
    mission_state: MissionState
    gps_available: bool
    elapsed_s: float
    note: str = ""


def haversine_m(a: Position, b: Position) -> float:
    lat1, lat2 = math.radians(a.lat), math.radians(b.lat)
    dlat = lat2 - lat1
    dlon = math.radians(b.lon - a.lon)
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def bearing_deg(a: Position, b: Position) -> float:
    lat1, lat2 = math.radians(a.lat), math.radians(b.lat)
    dlon = math.radians(b.lon - a.lon)
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


# --------------------------------------------------------------------------
# The drone thread
# --------------------------------------------------------------------------


class DroneThread(threading.Thread):
    """One UAV as an independent node with its own state and execution loop."""

    def __init__(
        self,
        drone_id: int,
        name: str = "",
        *,
        cruise_speed_mps: float = 15.0,
        cruise_altitude_m: float = 50.0,
        battery_capacity_mah: float = 5200.0,
        start: Optional[Position] = None,
        telemetry_sink: Optional[Callable[[DroneSnapshot], None]] = None,
        tick_s: float = DEFAULT_TICK_S,
        time_scale: float = 1.0,
    ):
        super().__init__(name=f"Drone-{drone_id}", daemon=True)

        # ---- identity and configuration ----
        self.drone_id = drone_id
        self.drone_name = name or f"drone-{drone_id}"
        self.cruise_speed_mps = max(0.1, cruise_speed_mps)
        self.cruise_altitude_m = max(1.0, cruise_altitude_m)
        self.battery_capacity_mah = max(1.0, battery_capacity_mah)
        self.tick_s = tick_s
        self.time_scale = max(1.0, time_scale)

        # ---- private state: only this thread mutates it ----
        self.position = start or Position()
        self.velocity = Velocity()
        self.battery = 100.0
        self.health_status = HealthStatus.NOMINAL
        self.communication_status = CommunicationStatus.ONLINE
        self.armed_status = False
        self.current_operation = Operation.NONE
        self.source: Optional[Position] = start
        self.destination: Optional[Position] = None
        self.mission_state = MissionState.IDLE

        # ---- internals ----
        self.gps_available = True
        self.elapsed_s = 0.0
        self._consumed_mah = 0.0
        self._note = ""
        self._running = threading.Event()
        self._inbox: "queue.Queue[Command]" = queue.Queue()
        self._telemetry_sink = telemetry_sink
        self._lock = threading.Lock()
        self._last_contact_s = time.monotonic()
        self._hold_requested = False

    # ---- Public API (called from other threads) ----

    def send(self, command: Command) -> None:
        """Queue a command. Never blocks the caller on drone work."""
        self._inbox.put(command)

    def shutdown(self) -> None:
        self.send(Command(CommandType.SHUTDOWN))

    def snapshot(self) -> DroneSnapshot:
        """Consistent copy of this drone's state, safe from any thread."""
        with self._lock:
            return DroneSnapshot(
                drone_id=self.drone_id,
                name=self.drone_name,
                position=self.position,
                velocity=self.velocity,
                battery=self.battery,
                health_status=self.health_status,
                communication_status=self.communication_status,
                armed_status=self.armed_status,
                current_operation=self.current_operation,
                source=self.source,
                destination=self.destination,
                mission_state=self.mission_state,
                gps_available=self.gps_available,
                elapsed_s=self.elapsed_s,
                note=self._note,
            )

    @property
    def is_running(self) -> bool:
        return self._running.is_set()

    @property
    def mission_finished(self) -> bool:
        return self.mission_state in (
            MissionState.LANDED, MissionState.ABORTED, MissionState.FAILED
        )

    # ---- Execution loop ----

    def run(self) -> None:
        self._running.set()
        print(f"Drone {self.drone_id} Thread started")
        try:
            while self._running.is_set():
                self.receive_command()
                self.update_state()
                self.monitor_health()
                self.execute_operation()
                self.publish_telemetry()
                time.sleep(self.tick_s)
        finally:
            self._running.clear()
            print(f"Drone {self.drone_id} Thread stopped")

    # ---- 1. receive_command ----

    def receive_command(self) -> None:
        """Drain the inbox. Commands mutate intent, never position directly."""
        while True:
            try:
                command = self._inbox.get_nowait()
            except queue.Empty:
                return
            self._last_contact_s = time.monotonic()
            self._apply_command(command)

    def _apply_command(self, command: Command) -> None:
        kind, payload = command.type, command.payload

        if kind is CommandType.SHUTDOWN:
            self._running.clear()
        elif kind is CommandType.HEARTBEAT:
            pass  # timestamp already refreshed
        elif kind is CommandType.ARM:
            if self.mission_state is MissionState.IDLE:
                with self._lock:
                    self.armed_status = True
                    self.mission_state = MissionState.ARMED
                    self._note = "armed"
        elif kind is CommandType.DISARM:
            if not self._airborne():
                with self._lock:
                    self.armed_status = False
                    self.mission_state = MissionState.IDLE
        elif kind is CommandType.TAKEOFF:
            if self.armed_status:
                with self._lock:
                    self.cruise_altitude_m = payload.get("altitude_m", self.cruise_altitude_m)
                    self.mission_state = MissionState.TAKEOFF
                    self.current_operation = Operation.GOTO
                    self.source = self.position
        elif kind in (CommandType.GOTO, CommandType.SET_DESTINATION):
            destination = payload.get("destination")
            if destination is not None:
                with self._lock:
                    self.destination = destination
                    self.current_operation = Operation.GOTO
                    self._hold_requested = False
                    if self.mission_state in (MissionState.HOLDING, MissionState.LANDING):
                        self.mission_state = MissionState.EN_ROUTE
                    self._note = "destination set"
        elif kind is CommandType.HOLD:
            self._hold_requested = True
        elif kind is CommandType.RESUME:
            self._hold_requested = False
        elif kind is CommandType.RETURN_TO_LAUNCH:
            with self._lock:
                self.destination = self.source
                self.current_operation = Operation.RETURN_TO_LAUNCH
                self._hold_requested = False
                self.mission_state = MissionState.EN_ROUTE
                self._note = "returning to launch"
        elif kind is CommandType.LAND:
            with self._lock:
                self.current_operation = Operation.LAND
                self._hold_requested = False
                self.mission_state = MissionState.LANDING
        elif kind is CommandType.ABORT:
            with self._lock:
                self.mission_state = MissionState.ABORTED
                self.current_operation = Operation.NONE
                self.velocity = Velocity()
        elif kind is CommandType.SET_GPS:
            with self._lock:
                self.gps_available = bool(payload.get("available", True))
                self._note = "GPS lost" if not self.gps_available else "GPS reacquired"

    # ---- 2. update_state ----

    def update_state(self) -> None:
        """Integrate motion and battery for one tick."""
        if self.mission_finished or self.mission_state is MissionState.IDLE:
            return

        dt = self.tick_s * self.time_scale
        with self._lock:
            self.elapsed_s += dt
            self._consumed_mah += NOMINAL_DRAW_A * (dt / 3600.0) * 1000.0
            self.battery = max(
                0.0, 100.0 * (1.0 - self._consumed_mah / self.battery_capacity_mah)
            )

        if self.mission_state is MissionState.TAKEOFF:
            self._climb(dt)
        elif self.mission_state is MissionState.EN_ROUTE:
            self._cruise(dt)
        elif self.mission_state is MissionState.LANDING:
            self._descend(dt)
        elif self.mission_state is MissionState.HOLDING:
            with self._lock:
                self.velocity = Velocity(0.0, self.velocity.heading_deg, 0.0)

    def _climb(self, dt: float) -> None:
        with self._lock:
            alt = min(self.cruise_altitude_m, self.position.alt_m + CLIMB_RATE_MPS * dt)
            self.position = replace(self.position, alt_m=alt)
            self.velocity = Velocity(0.0, self.velocity.heading_deg, CLIMB_RATE_MPS)

    def _cruise(self, dt: float) -> None:
        if self.destination is None:
            return
        remaining = haversine_m(self.position, self.destination)
        step = self.cruise_speed_mps * dt
        with self._lock:
            if remaining <= step or remaining <= 0.0:
                self.position = replace(
                    self.position, lat=self.destination.lat, lon=self.destination.lon
                )
            else:
                fraction = step / remaining
                self.position = replace(
                    self.position,
                    lat=self.position.lat + (self.destination.lat - self.position.lat) * fraction,
                    lon=self.position.lon + (self.destination.lon - self.position.lon) * fraction,
                )
            self.velocity = Velocity(
                self.cruise_speed_mps, bearing_deg(self.position, self.destination), 0.0
            )

    def _descend(self, dt: float) -> None:
        with self._lock:
            alt = max(0.0, self.position.alt_m - DESCENT_RATE_MPS * dt)
            self.position = replace(self.position, alt_m=alt)
            self.velocity = Velocity(0.0, self.velocity.heading_deg, -DESCENT_RATE_MPS)

    # ---- 3. monitor_health ----

    def monitor_health(self) -> None:
        """Derive health and comms from the drone's own observations."""
        with self._lock:
            if self.battery <= 0.0 and self._airborne():
                self.health_status = HealthStatus.FAILED
                self.mission_state = MissionState.FAILED
                self.current_operation = Operation.NONE
                self.velocity = Velocity()
                self._note = "battery exhausted - drone lost"
            elif self.battery <= CRITICAL_BATTERY_PCT:
                self.health_status = HealthStatus.CRITICAL
            elif self.battery <= LOW_BATTERY_PCT or not self.gps_available:
                self.health_status = HealthStatus.DEGRADED
            else:
                self.health_status = HealthStatus.NOMINAL

            silence = time.monotonic() - self._last_contact_s
            if silence > COMMS_TIMEOUT_S * 2:
                self.communication_status = CommunicationStatus.LOST
            elif silence > COMMS_TIMEOUT_S:
                self.communication_status = CommunicationStatus.DEGRADED
            else:
                self.communication_status = CommunicationStatus.ONLINE

    # ---- 4. execute_operation ----

    def execute_operation(self) -> None:
        """Advance the mission state machine off the state just updated."""
        if self.mission_finished:
            return

        # Losing GPS, or being told to hold, parks the drone where it is.
        if self._airborne() and (self._hold_requested or not self.gps_available):
            if self.mission_state is not MissionState.HOLDING:
                with self._lock:
                    self.mission_state = MissionState.HOLDING
                    self._note = "holding" if self._hold_requested else "holding - no GPS"
            return

        if self.mission_state is MissionState.HOLDING:
            with self._lock:
                self.mission_state = MissionState.EN_ROUTE
                self._note = "resuming"
            return

        if self.mission_state is MissionState.TAKEOFF:
            if self.position.alt_m >= self.cruise_altitude_m:
                with self._lock:
                    self.mission_state = MissionState.EN_ROUTE
                    self._note = "en route"
            return

        if self.mission_state is MissionState.EN_ROUTE:
            if self.destination is None:
                return
            if haversine_m(self.position, self.destination) <= ARRIVAL_RADIUS_M:
                with self._lock:
                    self.mission_state = MissionState.LANDING
                    self.current_operation = Operation.LAND
                    self._note = "arrived - landing"
            return

        if self.mission_state is MissionState.LANDING:
            if self.position.alt_m <= 0.0:
                with self._lock:
                    self.mission_state = MissionState.LANDED
                    self.armed_status = False
                    self.current_operation = Operation.NONE
                    self.velocity = Velocity()
                    self._note = "landed"

    # ---- 5. publish_telemetry ----

    def publish_telemetry(self) -> None:
        """Hand an immutable snapshot to whoever is listening."""
        if self._telemetry_sink is None:
            return
        self._telemetry_sink(self.snapshot())

    # ---- helpers ----

    def _airborne(self) -> bool:
        return self.mission_state in (
            MissionState.TAKEOFF, MissionState.EN_ROUTE,
            MissionState.HOLDING, MissionState.LANDING,
        )
