"""Swarm-level MAVLink mission runner.

Presents the same signals and methods as `LocalFlightSimulator`, so the main
window can swap one for the other without knowing which is flying. All the
protocol work happens in `engine.mavlink_link`; all the socket I/O happens on a
worker thread, because every MAVLink call blocks.

The mission is the single supported operation - start point to destination -
with the same preconditions and exceptions as the local preview, except that
each is now driven by real protocol traffic:

    battery precondition  -> checked before ARM; a refusal also arrives as
                             COMMAND_ACK / MAV_RESULT_DENIED
    re-route              -> a fresh SET_POSITION_TARGET_GLOBAL_INT
    low battery (20%)     -> SYS_STATUS.battery_remaining
    GPS loss              -> GPS_RAW_INT.fix_type below a 2D fix
    fall back             -> MAV_CMD_NAV_RETURN_TO_LAUNCH
    emergency landing     -> MAV_CMD_NAV_LAND / mode LAND
"""
from __future__ import annotations

import time
from typing import Optional

from PySide6.QtCore import QObject, QThread, QTimer, Signal, Slot

from contracts.gui_orchestration import (
    DroneConfig,
    FaultType,
    InjectFault,
    LatLon,
    SwarmTelemetryBatch,
)
from engine.mavlink_link import (
    ARRIVAL_RADIUS_M,
    LOW_BATTERY_PCT,
    TAKEOFF_TOLERANCE,
    COPTER_MODE_GUIDED,
    LinkState,
    MavlinkCommandError,
    MavlinkVehicle,
    haversine_m,
)
from services.local_flight import EmergencyLanding, nearest_point, plan_flights

TICK_MS = 200  # 5 Hz mission stepping; telemetry arrives faster and is buffered

# One SITL instance per drone. ArduPilot's first vehicle listens on 14550 and
# each additional instance is offset by 10 (SITL --instance N).
DEFAULT_BASE_PORT = 14550
DEFAULT_PORT_STRIDE = 10


def default_endpoint(index: int, host: str = "127.0.0.1") -> str:
    """`udpin:` - we bind and wait for SITL to send to us, which is how
    ArduPilot's default --out stream behaves."""
    return f"udpin:{host}:{DEFAULT_BASE_PORT + index * DEFAULT_PORT_STRIDE}"


class _Mission:
    """One vehicle's progress through the source-to-destination task."""

    def __init__(self, vehicle: MavlinkVehicle, destination: LatLon, altitude_m: float):
        self.vehicle = vehicle
        self.destination = destination
        self.altitude_m = altitude_m
        self.home: Optional[LatLon] = None
        self.started_at = time.time()
        self.low_battery_flagged = False
        self.gps_flagged = False
        self.gps_hold = False
        self.emergency = False
        self.done = False

    # Attributes the main window's dialogs read, mirroring `_Flight`.

    @property
    def config(self) -> DroneConfig:
        return self.vehicle.config

    @property
    def position(self) -> LatLon:
        return self.vehicle.snapshot.position

    @property
    def start(self) -> LatLon:
        return self.home or self.vehicle.snapshot.position

    @property
    def battery_pct(self) -> float:
        return self.vehicle.snapshot.battery_pct

    @property
    def is_lost(self) -> bool:
        return self.vehicle.state is LinkState.FAILED

    @property
    def is_done(self) -> bool:
        return self.done or self.vehicle.state in (LinkState.DONE, LinkState.FAILED)

    def can_reach(self, point: LatLon) -> bool:
        """Endurance estimate for the remaining charge, same rule the local
        preview uses - the autopilot will not tell us this."""
        snap = self.vehicle.snapshot
        speed = max(0.1, self.config.max_velocity_mps)
        remaining_mah = self.config.battery_capacity_mah * max(0.0, snap.battery_pct) / 100.0
        endurance_s = (remaining_mah / 1000.0) / 20.0 * 3600.0
        travel_s = haversine_m(snap.position, point) / speed
        descent_s = snap.relative_alt_m / 2.0
        return travel_s + descent_s <= endurance_s


class _Worker(QObject):
    """Runs on the MAVLink thread. Owns every vehicle connection."""

    batch_ready = Signal(object)
    progress = Signal(float, float)
    finished = Signal(float)
    low_battery = Signal(int, float)
    drone_lost = Signal(int)
    gps_lost = Signal(int)
    gps_restored = Signal(int)
    link_failed = Signal(str)
    launched = Signal(int)

    def __init__(self):
        super().__init__()
        self._missions: list[_Mission] = []
        self._timer: Optional[QTimer] = None
        self._paused = False
        self._started_at = 0.0
        self._tick = 0

    # ---- Lifecycle (all invoked from the GUI thread via queued signals) ----

    @Slot(list, object, object, list)
    def start(self, drones: list, start: object, destination: object, endpoints: list) -> None:
        self.shutdown()
        missions: list[_Mission] = []
        for index, drone in enumerate(drones):
            endpoint = endpoints[index] if index < len(endpoints) else default_endpoint(index)
            vehicle = MavlinkVehicle(drone, endpoint)
            if not vehicle.connect(timeout_s=20.0):
                self.link_failed.emit(f"{drone.name}: {vehicle.last_error}")
                continue
            try:
                vehicle.set_mode(COPTER_MODE_GUIDED)
                time.sleep(0.3)
                vehicle.arm()
                vehicle.takeoff(drone.cruise_altitude_m)
            except MavlinkCommandError as exc:
                self.link_failed.emit(f"{drone.name}: {exc}")
                vehicle.close()
                continue
            mission = _Mission(vehicle, destination, drone.cruise_altitude_m)
            mission.home = vehicle.snapshot.position
            missions.append(mission)

        self._missions = missions
        self.launched.emit(len(missions))
        if not missions:
            return

        self._started_at = time.time()
        self._tick = 0
        self._paused = False
        self._timer = QTimer()
        self._timer.setInterval(TICK_MS)
        self._timer.timeout.connect(self._step)
        self._timer.start()

    @Slot()
    def shutdown(self) -> None:
        if self._timer is not None:
            self._timer.stop()
            self._timer = None
        for mission in self._missions:
            mission.vehicle.close()
        self._missions = []

    @Slot()
    def pause(self) -> None:
        """Hold every vehicle in place while the controller decides."""
        self._paused = True
        for mission in self._missions:
            if not mission.is_done:
                mission.vehicle.hold_position()

    @Slot()
    def resume(self) -> None:
        self._paused = False
        for mission in self._missions:
            if not mission.is_done and not mission.emergency and not mission.gps_hold:
                mission.vehicle.set_mode(COPTER_MODE_GUIDED)
                mission.vehicle.goto(mission.destination, mission.altitude_m)

    # ---- Commands ----

    @Slot(object)
    def retarget_all(self, destination: object) -> None:
        for mission in self._missions:
            if mission.is_done or mission.emergency:
                continue
            mission.destination = destination
            mission.vehicle.set_mode(COPTER_MODE_GUIDED)
            mission.vehicle.goto(destination, mission.altitude_m)

    @Slot(int)
    def fall_back(self, sysid: int) -> None:
        mission = self._mission_for(sysid)
        if mission is None or mission.is_done:
            return
        mission.emergency = True
        mission.gps_hold = False
        mission.vehicle.return_to_launch()

    @Slot(int, object)
    def emergency_land(self, sysid: int, target: object) -> None:
        mission = self._mission_for(sysid)
        if mission is None or mission.is_done:
            return
        mission.emergency = True
        mission.gps_hold = False
        if target is None:
            mission.vehicle.land()
        else:
            mission.destination = target
            mission.vehicle.set_mode(COPTER_MODE_GUIDED)
            mission.vehicle.goto(target, mission.altitude_m)

    @Slot(int)
    def hold(self, sysid: int) -> None:
        mission = self._mission_for(sysid)
        if mission is not None and not mission.is_done:
            mission.gps_hold = True
            mission.vehicle.hold_position()

    def _mission_for(self, sysid: int) -> Optional[_Mission]:
        return next((m for m in self._missions if m.config.sysid == sysid), None)

    # ---- Tick ----

    def _step(self) -> None:
        if not self._missions:
            return
        self._tick += 1

        for mission in self._missions:
            mission.vehicle.pump()

        if not self._paused:
            for mission in self._missions:
                self._advance(mission)

        elapsed = time.time() - self._started_at
        self.batch_ready.emit(
            SwarmTelemetryBatch(
                tick=self._tick,
                timestamp=elapsed,
                drones=[m.vehicle.telemetry() for m in self._missions],
            )
        )

        remaining = max(
            (m.vehicle.distance_to_target_m() / max(0.1, m.config.max_velocity_mps)
             for m in self._missions if not m.is_done),
            default=0.0,
        )
        self.progress.emit(elapsed, elapsed + remaining)

        if all(m.is_done for m in self._missions):
            if self._timer is not None:
                self._timer.stop()
            self.finished.emit(elapsed)
            return

        self._check_exceptions()

    def _advance(self, mission: _Mission) -> None:
        """Step one vehicle's mission state machine off decoded telemetry."""
        vehicle = mission.vehicle
        snap = vehicle.snapshot

        if mission.is_done or mission.gps_hold:
            return

        if not vehicle.link_alive:
            vehicle.state = LinkState.FAILED
            mission.done = True
            self.drone_lost.emit(mission.config.sysid)
            return

        if vehicle.state is LinkState.TAKING_OFF:
            if snap.relative_alt_m >= mission.altitude_m * TAKEOFF_TOLERANCE:
                vehicle.goto(mission.destination, mission.altitude_m)
            return

        if vehicle.state is LinkState.CRUISING:
            if vehicle.distance_to_target_m() <= ARRIVAL_RADIUS_M:
                vehicle.land()
            return

        if vehicle.state is LinkState.LANDING:
            if not snap.armed and snap.relative_alt_m < 1.0:
                vehicle.state = LinkState.DONE
                mission.done = True

    def _check_exceptions(self) -> None:
        """Raise the first situation needing a controller decision, and hold."""
        for mission in self._missions:
            if mission.is_done:
                continue
            snap = mission.vehicle.snapshot

            if not snap.has_gps and not mission.gps_flagged:
                mission.gps_flagged = True
                mission.gps_hold = True
                mission.low_battery_flagged = False
                mission.vehicle.hold_position()
                self.pause()
                self.gps_lost.emit(mission.config.sysid)
                return

            if snap.has_gps and mission.gps_flagged:
                mission.gps_flagged = False
                if mission.gps_hold:
                    mission.gps_hold = False
                    mission.vehicle.set_mode(COPTER_MODE_GUIDED)
                    mission.vehicle.goto(mission.destination, mission.altitude_m)
                self.gps_restored.emit(mission.config.sysid)
                return

            if (
                not mission.low_battery_flagged
                and not mission.emergency
                and snap.battery_pct <= LOW_BATTERY_PCT
            ):
                mission.low_battery_flagged = True
                self.pause()
                self.low_battery.emit(mission.config.sysid, snap.battery_pct)
                return


class MavlinkSwarmBackend(QObject):
    """GUI-facing handle. Mirrors `LocalFlightSimulator`'s surface."""

    batch_ready = Signal(object)
    progress = Signal(float, float)
    finished = Signal(float)
    low_battery = Signal(int, float)
    drone_lost = Signal(int)
    gps_lost = Signal(int)
    gps_restored = Signal(int)
    retargeted = Signal(int)
    link_failed = Signal(str)

    _start_requested = Signal(list, object, object, list)
    _shutdown_requested = Signal()
    _pause_requested = Signal()
    _resume_requested = Signal()
    _retarget_requested = Signal(object)
    _fall_back_requested = Signal(int)
    _emergency_requested = Signal(int, object)
    _hold_requested = Signal(int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._thread = QThread(self)
        self._worker = _Worker()
        self._worker.moveToThread(self._thread)
        self._active = False
        self._running = False

        for name in (
            "batch_ready", "progress", "finished", "low_battery",
            "drone_lost", "gps_lost", "gps_restored", "link_failed",
        ):
            getattr(self._worker, name).connect(getattr(self, name))

        self._worker.launched.connect(self._on_launched)
        self._start_requested.connect(self._worker.start)
        self._shutdown_requested.connect(self._worker.shutdown)
        self._pause_requested.connect(self._worker.pause)
        self._resume_requested.connect(self._worker.resume)
        self._retarget_requested.connect(self._worker.retarget_all)
        self._fall_back_requested.connect(self._worker.fall_back)
        self._emergency_requested.connect(self._worker.emergency_land)
        self._hold_requested.connect(self._worker.hold)
        self.finished.connect(lambda *_: setattr(self, "_running", False))
        self._thread.start()

    # ---- Surface shared with LocalFlightSimulator ----

    def is_running(self) -> bool:
        return self._running

    def is_active(self) -> bool:
        return self._active

    @property
    def time_scale(self) -> float:
        return 1.0  # a real vehicle flies in real time

    @property
    def flights(self) -> list:
        return list(self._worker._missions)

    def _on_launched(self, count: int) -> None:
        self._active = count > 0
        self._running = count > 0

    def start_mission(
        self,
        drones: list[DroneConfig],
        start: LatLon,
        destination: LatLon,
        endpoints: Optional[list[str]] = None,
    ) -> None:
        endpoints = endpoints or [default_endpoint(i) for i in range(len(drones))]
        self._start_requested.emit(list(drones), start, destination, endpoints)

    def stop(self) -> None:
        self._active = False
        self._running = False
        self._shutdown_requested.emit()

    def pause(self) -> None:
        self._pause_requested.emit()

    def resume(self) -> None:
        self._resume_requested.emit()

    def retarget_all(self, destination: LatLon) -> int:
        active = [m for m in self.flights if not m.is_done]
        self._retarget_requested.emit(destination)
        if active:
            self.retargeted.emit(len(active))
        return len(active)

    def fall_back(self, sysid: int) -> Optional[EmergencyLanding]:
        mission = next((m for m in self.flights if m.config.sysid == sysid), None)
        if mission is None:
            return None
        home = mission.start
        reachable = mission.can_reach(home)
        self._fall_back_requested.emit(sysid)
        return EmergencyLanding(
            target=home if reachable else mission.position,
            distance_m=haversine_m(mission.position, home) if reachable else 0.0,
            in_place=not reachable,
        )

    def emergency_land(self, sysid: int, candidates: list[LatLon]) -> Optional[EmergencyLanding]:
        mission = next((m for m in self.flights if m.config.sysid == sysid), None)
        if mission is None:
            return None
        reachable = [p for p in candidates if mission.can_reach(p)]
        target = nearest_point(mission.position, reachable)
        self._emergency_requested.emit(sysid, target)
        if target is None:
            return EmergencyLanding(target=mission.position, distance_m=0.0, in_place=True)
        return EmergencyLanding(
            target=target,
            distance_m=haversine_m(mission.position, target),
            in_place=False,
        )

    def inject_fault(self, command: InjectFault) -> list[int]:
        """A real autopilot cannot be told to lose GPS from here; fault
        injection against live vehicles belongs to SITL's own parameters
        (e.g. SIM_GPS_DISABLE). Reported so the GUI can say so."""
        return []

    def clear_faults(self, sysids: Optional[list[int]] = None) -> list[int]:
        return []

    def preflight_check(self, drones: list[DroneConfig], start: LatLon, destination: LatLon):
        """Battery feasibility, reusing the local preview's estimate so there is
        one implementation of the rule."""
        flights = plan_flights(drones, start, destination)
        return [f for f in flights if f.feasible], [f for f in flights if not f.feasible]

    def shutdown_thread(self) -> None:
        self.stop()
        self._thread.quit()
        self._thread.wait(2000)
