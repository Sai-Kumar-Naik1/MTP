"""Local flight preview for Module 1.

Module 2 owns the real flight dynamics, but when it is not connected there is
no telemetry stream at all, so "Emulate" has nothing to draw. This module flies
a simple, self-consistent profile - climb to cruise altitude, run the course to
the destination, descend - and emits the very same `SwarmTelemetryBatch`
objects the orchestrator would, so the map, Global State Matrix and Drone
Inspector all light up through the ordinary telemetry path.

State is integrated tick by tick rather than sampled from a fixed plan, so a
drone can be re-targeted mid-air and will turn for the new destination from
wherever it happens to be.

It is a visualisation aid, not a flight model: no wind, no acceleration
envelope, no turn radius. Positions interpolate linearly in lat/lon, which is
indistinguishable from a great circle at the ranges a single battery allows.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from PySide6.QtCore import QObject, QTimer, Signal

from contracts.gui_orchestration import (
    DroneConfig,
    DroneStatus,
    DroneTelemetry,
    FaultType,
    InjectFault,
    LatLon,
    SwarmTelemetryBatch,
)

EARTH_RADIUS_M = 6_371_000.0
TICK_MS = 100                    # 10 Hz, a typical telemetry stream rate
TICK_S = TICK_MS / 1000.0
CLIMB_RATE_MPS = 3.0
DESCENT_RATE_MPS = 2.0
CRASH_RATE_MPS = 12.0            # an unpowered drone comes down rather faster
NOMINAL_DRAW_A = 20.0            # rough cruise current, for endurance maths
SWARM_LATERAL_SPACING_M = 150.0  # so a swarm does not stack on one pixel
METRES_PER_DEG_LAT = 111_320.0
LOW_BATTERY_PCT = 20.0           # threshold at which the controller is asked

# Wall-clock budget per phase. Compressing the mission uniformly would step
# straight over the climb and descent (a cruise leg is orders of magnitude
# longer), so each phase gets its own slice of the animation instead.
CLIMB_WALL_S = 5.0
CRUISE_WALL_S = 30.0
DESCENT_WALL_S = 5.0
TARGET_WALL_CLOCK_S = CLIMB_WALL_S + CRUISE_WALL_S + DESCENT_WALL_S

AIRBORNE = (DroneStatus.TAKING_OFF, DroneStatus.IN_FLIGHT, DroneStatus.LANDING)


def haversine_m(a: LatLon, b: LatLon) -> float:
    """Great-circle distance in metres."""
    lat1, lat2 = math.radians(a.lat), math.radians(b.lat)
    dlat = lat2 - lat1
    dlon = math.radians(b.lon - a.lon)
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(h)))


def initial_bearing_deg(a: LatLon, b: LatLon) -> float:
    """Compass bearing from `a` to `b`, degrees clockwise from north."""
    lat1, lat2 = math.radians(a.lat), math.radians(b.lat)
    dlon = math.radians(b.lon - a.lon)
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def format_duration(seconds: float) -> str:
    """`92.5` -> `01:32`, `4210` -> `1:10:10`."""
    seconds = max(0, int(round(seconds)))
    hours, rem = divmod(seconds, 3600)
    minutes, secs = divmod(rem, 60)
    return f"{hours}:{minutes:02d}:{secs:02d}" if hours else f"{minutes:02d}:{secs:02d}"


def nearest_point(origin: LatLon, candidates: list[LatLon]) -> Optional[LatLon]:
    """Whichever candidate is closest to `origin`."""
    if not candidates:
        return None
    return min(candidates, key=lambda p: haversine_m(origin, p))


@dataclass
class _Flight:
    """One drone: its planned leg, and its live state as the mission runs."""

    config: DroneConfig
    start: LatLon
    dest: LatLon
    distance_m: float
    bearing_deg: float
    lateral_offset_m: float = 0.0

    # Derived from the config, fixed for the mission.
    cruise_speed_mps: float = field(init=False)
    cruise_alt_m: float = field(init=False)
    climb_s: float = field(init=False)
    cruise_s: float = field(init=False)
    descent_s: float = field(init=False)
    total_s: float = field(init=False)
    endurance_s: float = field(init=False)

    # Live state, integrated each tick.
    lat: float = field(init=False)
    lon: float = field(init=False)
    altitude_m: float = field(init=False, default=0.0)
    consumed_mah: float = field(init=False, default=0.0)
    elapsed_s: float = field(init=False, default=0.0)
    status: DroneStatus = field(init=False, default=DroneStatus.TAKING_OFF)
    low_battery_flagged: bool = field(init=False, default=False)
    emergency: bool = field(init=False, default=False)

    # GPS loss: `gps_hold` means holding position awaiting either a decision
    # from the controller or the return of GPS.
    gps_lost: bool = field(init=False, default=False)
    gps_hold: bool = field(init=False, default=False)
    gps_prompted: bool = field(init=False, default=False)
    gps_clear_at_s: Optional[float] = field(init=False, default=None)

    def __post_init__(self) -> None:
        self.cruise_speed_mps = max(0.1, self.config.max_velocity_mps)
        self.cruise_alt_m = max(1.0, self.config.cruise_altitude_m)
        self.climb_s = self.cruise_alt_m / CLIMB_RATE_MPS
        self.cruise_s = self.distance_m / self.cruise_speed_mps
        self.descent_s = self.cruise_alt_m / DESCENT_RATE_MPS
        self.total_s = self.climb_s + self.cruise_s + self.descent_s
        self.endurance_s = (self.config.battery_capacity_mah / 1000.0) / NOMINAL_DRAW_A * 3600.0
        self.lat = self.start.lat
        self.lon = self.start.lon

    # ---- Pre-flight assessment (the straight leg, before anything moves) ----

    @property
    def range_m(self) -> float:
        """How far this drone can actually fly on one battery."""
        return max(0.0, self.endurance_s - self.climb_s - self.descent_s) * self.cruise_speed_mps

    @property
    def required_mah(self) -> float:
        """Charge the full profile - climb, cruise, descent - would draw."""
        return NOMINAL_DRAW_A * (self.total_s / 3600.0) * 1000.0

    @property
    def feasible(self) -> bool:
        """Whether this drone can complete the leg on the battery it carries."""
        return self.required_mah <= self.config.battery_capacity_mah

    # ---- Live state ----

    @property
    def position(self) -> LatLon:
        return LatLon(lat=self.lat, lon=self.lon)

    @property
    def battery_pct(self) -> float:
        if self.config.battery_capacity_mah <= 0:
            return 0.0
        remaining = 1.0 - self.consumed_mah / self.config.battery_capacity_mah
        return max(0.0, min(100.0, remaining * 100.0))

    @property
    def is_airborne(self) -> bool:
        return self.status in AIRBORNE

    @property
    def is_lost(self) -> bool:
        return self.status is DroneStatus.FAILSAFE

    @property
    def is_done(self) -> bool:
        return self.status in (DroneStatus.LANDED, DroneStatus.FAILSAFE)

    @property
    def remaining_distance_m(self) -> float:
        return haversine_m(self.position, self.dest)

    @property
    def remaining_s(self) -> float:
        """Rough time still to fly, for the ETA readout."""
        if self.is_done:
            return 0.0
        climb_left = max(0.0, self.cruise_alt_m - self.altitude_m) / CLIMB_RATE_MPS
        cruise_left = self.remaining_distance_m / self.cruise_speed_mps
        return climb_left + cruise_left + self.descent_s

    def retarget(self, destination: LatLon) -> None:
        """Divert to a new destination from wherever the drone is now."""
        if self.is_lost:
            return
        self.dest = destination
        self.bearing_deg = initial_bearing_deg(self.position, destination)
        if self.status in (DroneStatus.LANDING, DroneStatus.LANDED):
            # Already on the way down (or down): climb back out and go again.
            self.status = DroneStatus.TAKING_OFF if self.altitude_m < self.cruise_alt_m else DroneStatus.IN_FLIGHT

    @property
    def remaining_endurance_s(self) -> float:
        """Flying time left in the battery at the nominal draw."""
        remaining_mah = max(0.0, self.config.battery_capacity_mah - self.consumed_mah)
        return (remaining_mah / 1000.0) / NOMINAL_DRAW_A * 3600.0

    def can_reach(self, point: LatLon) -> bool:
        """Whether the remaining charge covers flying there *and* landing."""
        travel_s = haversine_m(self.position, point) / self.cruise_speed_mps
        climb_s = 0.0
        if self.status is DroneStatus.TAKING_OFF:
            climb_s = max(0.0, self.cruise_alt_m - self.altitude_m) / CLIMB_RATE_MPS
        descent_s = self.altitude_m / DESCENT_RATE_MPS
        return climb_s + travel_s + descent_s <= self.remaining_endurance_s

    def divert_to(self, destination: LatLon) -> None:
        """Emergency landing: make for `destination` and put down there."""
        self.gps_hold = False  # committing to land means it stops holding
        self.retarget(destination)
        self.emergency = True

    def land_in_place(self) -> None:
        """Nothing is within reach: put down directly below, right now."""
        if self.is_lost or self.status is DroneStatus.LANDED:
            return
        self.gps_hold = False
        self.dest = self.position
        self.status = DroneStatus.LANDING
        self.emergency = True

    # ---- GPS ----

    def begin_gps_hold(self, clear_at_s: Optional[float]) -> None:
        """GPS is gone: stop navigating and hold station where we are."""
        if self.is_done:
            return
        self.gps_lost = True
        self.gps_clear_at_s = clear_at_s
        if not self.emergency:
            self.gps_hold = True
            self.gps_prompted = False
            # A hold is a new situation: let the 20% warning speak up again.
            self.low_battery_flagged = False
            if self.status is DroneStatus.TAKING_OFF:
                self.status = DroneStatus.IN_FLIGHT

    def restore_gps(self) -> None:
        """GPS is back: pick the mission up again from where we hovered."""
        self.gps_lost = False
        self.gps_clear_at_s = None
        self.gps_prompted = False
        if self.gps_hold:
            self.gps_hold = False
            if not self.is_done and not self.emergency:
                self.status = DroneStatus.IN_FLIGHT
                self.bearing_deg = initial_bearing_deg(self.position, self.dest)

    def advance(self, dt: float) -> None:
        """Integrate `dt` simulated seconds of flight."""
        if self.status is DroneStatus.LANDED:
            return

        if self.is_lost:
            # Unpowered: falls until it hits the ground, then stays put.
            self.altitude_m = max(0.0, self.altitude_m - CRASH_RATE_MPS * dt)
            return

        self.elapsed_s += dt
        self.consumed_mah += NOMINAL_DRAW_A * (dt / 3600.0) * 1000.0

        if self.battery_pct <= 0.0:
            self.status = DroneStatus.FAILSAFE
            return

        if self.gps_hold:
            # Holding station: no navigation without GPS, but the rotors are
            # still turning, so the battery keeps draining.
            return

        if self.status is DroneStatus.TAKING_OFF:
            self.altitude_m += CLIMB_RATE_MPS * dt
            if self.altitude_m >= self.cruise_alt_m:
                self.altitude_m = self.cruise_alt_m
                self.status = DroneStatus.IN_FLIGHT
            return

        if self.status is DroneStatus.IN_FLIGHT:
            remaining = self.remaining_distance_m
            step = self.cruise_speed_mps * dt
            if remaining <= step or remaining <= 0.0:
                self.lat, self.lon = self.dest.lat, self.dest.lon
                self.status = DroneStatus.LANDING
            else:
                fraction = step / remaining
                self.lat += (self.dest.lat - self.lat) * fraction
                self.lon += (self.dest.lon - self.lon) * fraction
                self.bearing_deg = initial_bearing_deg(self.position, self.dest)
            return

        if self.status is DroneStatus.LANDING:
            self.altitude_m -= DESCENT_RATE_MPS * dt
            if self.altitude_m <= 0.0:
                self.altitude_m = 0.0
                self.status = DroneStatus.LANDED

    def _display_position(self) -> tuple[float, float]:
        """Shift sideways off the course line so swarm members stay distinct."""
        if not self.lateral_offset_m:
            return self.lat, self.lon
        perpendicular = math.radians(self.bearing_deg + 90.0)
        north_m = self.lateral_offset_m * math.cos(perpendicular)
        east_m = self.lateral_offset_m * math.sin(perpendicular)
        lat_out = self.lat + north_m / METRES_PER_DEG_LAT
        scale = METRES_PER_DEG_LAT * max(0.01, math.cos(math.radians(self.lat)))
        return lat_out, self.lon + east_m / scale

    def telemetry(self, tick: int) -> DroneTelemetry:
        lat, lon = self._display_position()
        holding = self.gps_hold
        speed = 0.0 if holding else (self.cruise_speed_mps if self.status is DroneStatus.IN_FLIGHT else 0.0)
        pitch = 0.0 if holding else {
            DroneStatus.TAKING_OFF: 10.0,
            DroneStatus.IN_FLIGHT: -5.0,
        }.get(self.status, 0.0)
        faults = []
        if self.gps_lost:
            faults.append(FaultType.GPS_LOSS)
        if self.is_lost:
            faults.append(FaultType.BATTERY_FAIL)

        raw = None
        if tick % 5 == 0:
            raw = (
                f"GLOBAL_POSITION_INT sysid={self.config.sysid} "
                f"lat={lat:.6f} lon={lon:.6f} alt={self.altitude_m:.1f}m "
                f"hdg={self.bearing_deg:.0f} vel={speed:.1f}m/s bat={self.battery_pct:.0f}%"
            )

        return DroneTelemetry(
            sysid=self.config.sysid,
            status=self.status,
            lat=lat,
            lon=lon,
            altitude_m=self.altitude_m,
            heading_deg=self.bearing_deg,
            ground_speed_mps=speed,
            roll_deg=0.0,
            pitch_deg=pitch,
            battery_pct=self.battery_pct,
            link_quality_pct=100.0,
            active_faults=faults,
            raw_mavlink=raw,
        )


def plan_route_flights(drones: list[DroneConfig], waypoints: list[LatLon]) -> list[_Flight]:
    """Like `plan_flights`, but for a multi-leg PDDL route instead of a
    straight line - `_Flight.feasible`/`.required_mah`/`.range_m` only need
    the total distance, so the same dataclass covers both cases.
    """
    total_distance = sum(haversine_m(a, b) for a, b in zip(waypoints, waypoints[1:]))
    bearing = initial_bearing_deg(waypoints[0], waypoints[1]) if len(waypoints) > 1 else 0.0
    return [
        _Flight(
            config=drone,
            start=waypoints[0],
            dest=waypoints[-1],
            distance_m=total_distance,
            bearing_deg=bearing,
        )
        for drone in drones
    ]


@dataclass
class EmergencyLanding:
    """Where a drone was sent when the controller granted an emergency landing."""

    target: LatLon
    distance_m: float
    in_place: bool  # nothing was in reach, so it put down where it stood


def plan_flights(drones: list[DroneConfig], start: LatLon, destination: LatLon) -> list[_Flight]:
    """Work out each drone's profile for this leg, without committing to fly it.

    Lets the caller screen out drones whose battery cannot cover the distance
    before anything takes off. The same objects are handed to
    `LocalFlightSimulator.start`, so the check and the flight can never drift
    apart.
    """
    distance = haversine_m(start, destination)
    bearing = initial_bearing_deg(start, destination)
    return [
        _Flight(
            config=drone,
            start=start,
            dest=destination,
            distance_m=distance,
            bearing_deg=bearing,
        )
        for drone in drones
    ]


class LocalFlightSimulator(QObject):
    """Drives a set of `_Flight`s and emits telemetry batches on a timer."""

    batch_ready = Signal(object)        # SwarmTelemetryBatch
    progress = Signal(float, float)     # simulated elapsed_s, estimated total_s
    finished = Signal(float)            # simulated elapsed_s at completion
    low_battery = Signal(int, float)    # sysid, battery_pct - mission is paused
    drone_lost = Signal(int)            # sysid - battery flat, drone sacrificed
    retargeted = Signal(int)            # count of drones diverted
    gps_lost = Signal(int)              # sysid - holding, mission is paused
    gps_restored = Signal(int)          # sysid - GPS back, mission resumed

    def __init__(self, parent=None):
        super().__init__(parent)
        self._timer = QTimer(self)
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._tick)
        self._flights: list[_Flight] = []
        self._elapsed_s = 0.0
        self._tick_index = 0
        self._climb_scale = 1.0
        self._cruise_scale = 1.0
        self._descent_scale = 1.0

    # ---- Lifecycle ----

    def is_running(self) -> bool:
        return self._timer.isActive()

    def is_active(self) -> bool:
        """A mission is loaded - running, or paused awaiting a decision."""
        return bool(self._flights) and not all(f.is_done for f in self._flights)

    @property
    def time_scale(self) -> float:
        """Simulated seconds elapsed per real second during cruise."""
        return self._cruise_scale

    @property
    def flights(self) -> list[_Flight]:
        return list(self._flights)

    def start(self, flights: list[_Flight]) -> bool:
        """Begin the mission for the given flights. False if there is nothing
        to fly. Callers screen out infeasible drones first - see
        `plan_flights` and `_Flight.feasible`; anything passed here launches."""
        self.stop()
        if not flights:
            return False

        centre = (len(flights) - 1) / 2.0
        for index, flight in enumerate(flights):
            flight.lateral_offset_m = (index - centre) * SWARM_LATERAL_SPACING_M
        self._flights = list(flights)
        self._elapsed_s = 0.0
        self._tick_index = 0

        pace = max(self._flights, key=lambda f: f.total_s)
        self._climb_scale = max(1.0, pace.climb_s / CLIMB_WALL_S)
        self._cruise_scale = max(1.0, pace.cruise_s / CRUISE_WALL_S)
        self._descent_scale = max(1.0, pace.descent_s / DESCENT_WALL_S)

        self._timer.start()
        return True

    def stop(self) -> None:
        self._timer.stop()
        self._flights = []
        self._elapsed_s = 0.0
        self._tick_index = 0

    def pause(self) -> None:
        self._timer.stop()

    def resume(self) -> None:
        if self._flights and not self._timer.isActive():
            self._timer.start()

    # ---- Commands ----

    def retarget_all(self, destination: LatLon) -> int:
        """Divert every still-flying drone to a new destination."""
        diverted = [f for f in self._flights if not f.is_done]
        for flight in diverted:
            flight.retarget(destination)
        if diverted:
            # The leg length just changed, so re-pace the cruise phase.
            longest = max(f.remaining_distance_m / f.cruise_speed_mps for f in diverted)
            self._cruise_scale = max(1.0, longest / CRUISE_WALL_S)
            self.retargeted.emit(len(diverted))
        return len(diverted)

    def emergency_land(self, sysid: int, candidates: list[LatLon]) -> Optional[EmergencyLanding]:
        """Put one drone down at the closest point it can still reach.

        Flying to a waypoint that the remaining charge cannot cover would just
        crash the drone somewhere else, so if nothing is in reach it lands
        directly below instead.
        """
        flight = self._flight_for(sysid)
        if flight is None or flight.is_done:
            return None

        reachable = [point for point in candidates if flight.can_reach(point)]
        target = nearest_point(flight.position, reachable)
        if target is not None:
            distance = haversine_m(flight.position, target)
            flight.divert_to(target)
            return EmergencyLanding(target=target, distance_m=distance, in_place=False)

        flight.land_in_place()
        return EmergencyLanding(target=flight.position, distance_m=0.0, in_place=True)

    def fall_back(self, sysid: int) -> Optional[EmergencyLanding]:
        """Abandon the mission and return to the point the drone launched from.

        Same reachability rule as an emergency landing: flying to a start point
        the battery cannot reach would only crash it further away.
        """
        flight = self._flight_for(sysid)
        if flight is None or flight.is_done:
            return None
        return self.emergency_land(sysid, [flight.start])

    def inject_fault(self, command: InjectFault) -> list[int]:
        """Apply an injected fault locally. Only GPS loss is modelled here -
        the rest stay Module 2's business."""
        if command.fault_type is not FaultType.GPS_LOSS:
            return []
        affected = [
            f for f in self._flights
            if f.config.sysid in command.sysids and not f.is_done and not f.gps_lost
        ]
        clear_at = self._elapsed_s + command.duration_s if command.duration_s else None
        for flight in affected:
            flight.begin_gps_hold(clear_at)
        if affected:
            self._check_prompts()
        return [f.config.sysid for f in affected]

    def clear_faults(self, sysids: Optional[list[int]] = None) -> list[int]:
        """Hand GPS back, so anything holding resumes its mission."""
        restored = []
        for flight in self._flights:
            if flight.gps_lost and (sysids is None or flight.config.sysid in sysids):
                flight.restore_gps()
                restored.append(flight.config.sysid)
                self.gps_restored.emit(flight.config.sysid)
        if restored:
            self.resume()
        return restored

    def _flight_for(self, sysid: int) -> Optional[_Flight]:
        return next((f for f in self._flights if f.config.sysid == sysid), None)

    # ---- Tick ----

    def _dt_for_phase(self) -> float:
        """Cruise legs dwarf climb and descent, so each phase runs on its own
        compression factor - otherwise a single tick would skip takeoff whole."""
        active = [f for f in self._flights if f.is_airborne]
        if any(f.status is DroneStatus.IN_FLIGHT for f in active):
            return TICK_S * self._cruise_scale
        if any(f.status is DroneStatus.TAKING_OFF for f in active):
            return TICK_S * self._climb_scale
        if any(f.status is DroneStatus.LANDING for f in active):
            return TICK_S * self._descent_scale
        return TICK_S * self._cruise_scale

    def _check_prompts(self) -> bool:
        """Hold the mission on the first drone needing a decision. One at a
        time: the next is picked up on the tick after the controller answers."""
        for flight in self._flights:
            if flight.gps_lost and flight.gps_hold and not flight.gps_prompted and not flight.is_done:
                flight.gps_prompted = True
                self.pause()
                self.gps_lost.emit(flight.config.sysid)
                return True
        for flight in self._flights:
            if (
                not flight.low_battery_flagged
                and not flight.emergency
                and flight.is_airborne
                and flight.battery_pct <= LOW_BATTERY_PCT
            ):
                flight.low_battery_flagged = True
                self.pause()
                self.low_battery.emit(flight.config.sysid, flight.battery_pct)
                return True
        return False

    def _tick(self) -> None:
        dt = self._dt_for_phase()
        self._elapsed_s += dt
        self._tick_index += 1

        for flight in self._flights:
            if (
                flight.gps_lost
                and flight.gps_clear_at_s is not None
                and self._elapsed_s >= flight.gps_clear_at_s
            ):
                flight.restore_gps()
                self.gps_restored.emit(flight.config.sysid)

        newly_lost: list[int] = []
        for flight in self._flights:
            was_lost = flight.is_lost
            flight.advance(dt)
            if flight.is_lost and not was_lost:
                newly_lost.append(flight.config.sysid)

        self.batch_ready.emit(
            SwarmTelemetryBatch(
                tick=self._tick_index,
                timestamp=self._elapsed_s,
                drones=[f.telemetry(self._tick_index) for f in self._flights],
            )
        )

        remaining = max((f.remaining_s for f in self._flights if not f.is_done), default=0.0)
        self.progress.emit(self._elapsed_s, self._elapsed_s + remaining)

        for sysid in newly_lost:
            self.drone_lost.emit(sysid)

        if all(f.is_done for f in self._flights):
            self._timer.stop()
            self.finished.emit(self._elapsed_s)
            return

        self._check_prompts()
