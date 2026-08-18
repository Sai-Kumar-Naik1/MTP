"""Drone-thread flight source.

Runs the swarm as independent threads talking to a Ground Control Station over
UDP (`engine.drone_thread`, `engine.drone_link`, `engine.gcs`), and presents the
same signals and methods as `LocalFlightSimulator`, so the main window can use
it without knowing that each drone is now its own node on a network.

Telemetry is pulled on a GUI-thread timer rather than pushed from the drone
threads: Qt widgets may only be touched from the GUI thread, and polling a
queue keeps the crossing explicit.
"""
from __future__ import annotations

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
from engine.drone_link import DroneLink
from engine.drone_thread import (
    CRITICAL_BATTERY_PCT,
    LOW_BATTERY_PCT,
    DroneSnapshot,
    HealthStatus,
    MissionState,
    Position,
    haversine_m as engine_haversine,
)
from engine.flight_controller import SimulatedFlightController
from engine.fleet import DroneFleet
from engine.gcs import GroundControlStation
from engine.protocol import CommandName
from services.local_flight import EmergencyLanding, nearest_point, plan_flights

POLL_MS = 100

# The node's mission vocabulary, in the terms the dashboard speaks.
_STATUS_MAP = {
    MissionState.IDLE: DroneStatus.STANDBY,
    MissionState.ARMED: DroneStatus.ARMED,
    MissionState.TAKEOFF: DroneStatus.TAKING_OFF,
    MissionState.EN_ROUTE: DroneStatus.IN_FLIGHT,
    MissionState.HOLDING: DroneStatus.IN_FLIGHT,
    MissionState.LANDING: DroneStatus.LANDING,
    MissionState.LANDED: DroneStatus.LANDED,
    MissionState.ABORTED: DroneStatus.FAILSAFE,
    MissionState.FAILED: DroneStatus.FAILSAFE,
}


def _to_latlon(position: Optional[Position]) -> Optional[LatLon]:
    if position is None:
        return None
    return LatLon(lat=position.lat, lon=position.lon)


class _FlightView:
    """What the main window's dialogs read, backed by a live drone thread."""

    def __init__(self, config: DroneConfig, drone, link: DroneLink, home: LatLon):
        self.config = config
        self._drone = drone
        self._link = link
        self._home = home

    @property
    def _snap(self) -> DroneSnapshot:
        return self._drone.snapshot()

    @property
    def position(self) -> LatLon:
        return _to_latlon(self._snap.position)

    @property
    def start(self) -> LatLon:
        return self._home

    @property
    def battery_pct(self) -> float:
        return self._snap.battery

    @property
    def gps_hold(self) -> bool:
        snap = self._snap
        return snap.mission_state is MissionState.HOLDING and not snap.gps_available

    @property
    def is_lost(self) -> bool:
        return self._snap.mission_state is MissionState.FAILED

    @property
    def is_done(self) -> bool:
        return self._snap.mission_state in (
            MissionState.LANDED, MissionState.ABORTED, MissionState.FAILED
        )

    @property
    def cruise_speed_mps(self) -> float:
        return self._drone.cruise_speed_mps

    @property
    def range_m(self) -> float:
        endurance_s = (self.config.battery_capacity_mah / 1000.0) / 20.0 * 3600.0
        return max(0.0, endurance_s) * self.cruise_speed_mps

    def can_reach(self, point: LatLon) -> bool:
        """Endurance estimate on the remaining charge - the same rule the local
        preview applies, evaluated against live node state."""
        snap = self._snap
        remaining_mah = self.config.battery_capacity_mah * max(0.0, snap.battery) / 100.0
        endurance_s = (remaining_mah / 1000.0) / 20.0 * 3600.0
        target = Position(lat=point.lat, lon=point.lon)
        travel_s = engine_haversine(snap.position, target) / max(0.1, self.cruise_speed_mps)
        descent_s = snap.position.alt_m / 2.0
        return travel_s + descent_s <= endurance_s


class ThreadSwarmBackend(QObject):
    """Flight source backed by one thread per drone, commanded over UDP."""

    batch_ready = Signal(object)
    progress = Signal(float, float)
    finished = Signal(float)
    low_battery = Signal(int, float)
    drone_lost = Signal(int)
    gps_lost = Signal(int)
    gps_restored = Signal(int)
    retargeted = Signal(int)
    link_failed = Signal(str)

    def __init__(self, parent=None, time_scale: float = 20.0, use_firmware: bool = True):
        super().__init__(parent)
        # With firmware on, each drone drives a MAVLink flight controller
        # rather than integrating its own motion.
        self.use_firmware = use_firmware
        self._fleet: Optional[DroneFleet] = None
        self._gcs: Optional[GroundControlStation] = None
        self._views: dict[int, _FlightView] = {}
        self._destination: Optional[LatLon] = None
        self._time_scale = time_scale
        self._elapsed_s = 0.0
        self._tick = 0
        self._paused = False
        self._flagged_low: set[int] = set()
        self._flagged_gps: set[int] = set()
        self._flagged_lost: set[int] = set()
        self._emergency: set[int] = set()

        self._timer = QTimer(self)
        self._timer.setInterval(POLL_MS)
        self._timer.timeout.connect(self._poll)

    # ---- Surface shared with the other flight sources ----

    def is_running(self) -> bool:
        return self._timer.isActive()

    def is_active(self) -> bool:
        return bool(self._views) and not all(v.is_done for v in self._views.values())

    @property
    def time_scale(self) -> float:
        return self._time_scale

    @property
    def flights(self) -> list:
        return list(self._views.values())

    def start_mission(self, drones: list[DroneConfig], start: LatLon, destination: LatLon) -> bool:
        """Spawn a node per drone, then command the mission over UDP."""
        self.stop()
        if not drones:
            return False

        self._gcs = GroundControlStation(verbose=False)
        self._gcs.start()
        self._fleet = DroneFleet()
        self._destination = destination
        home = Position(lat=start.lat, lon=start.lon)

        for drone in drones:
            try:
                link = DroneLink(sysid=drone.sysid, gcs_address=self._gcs.address, verbose=False)
            except OSError as exc:
                self.link_failed.emit(f"{drone.name}: cannot bind SYSID {drone.sysid} ({exc})")
                continue
            controller = None
            if self.use_firmware:
                controller = SimulatedFlightController(
                    sysid=drone.sysid,
                    home_lat=start.lat,
                    home_lon=start.lon,
                    cruise_speed_mps=drone.max_velocity_mps,
                    battery_capacity_mah=drone.battery_capacity_mah,
                )
            thread = self._fleet.spawn(
                drone_id=drone.sysid,
                name=drone.name,
                cruise_speed_mps=drone.max_velocity_mps,
                cruise_altitude_m=drone.cruise_altitude_m,
                battery_capacity_mah=drone.battery_capacity_mah,
                start=home,
                time_scale=self._time_scale,
                link=link,
                flight_controller=controller,
            )
            self._views[drone.sysid] = _FlightView(drone, thread, link, start)

        if not self._views:
            self.stop()
            return False

        self._gcs.register_many(self._views)
        self._gcs.wait_for_heartbeats(self._views, timeout_s=3.0)

        source = [start.lat, start.lon, 0.0]
        target = [destination.lat, destination.lon, drones[0].cruise_altitude_m]
        for sysid in self._views:
            self._gcs.navigate(sysid, source, target)

        self._elapsed_s = 0.0
        self._tick = 0
        self._paused = False
        for flags in (self._flagged_low, self._flagged_gps, self._flagged_lost, self._emergency):
            flags.clear()
        self._timer.start()
        return True

    def stop(self) -> None:
        self._timer.stop()
        if self._fleet is not None:
            self._fleet.shutdown()
            self._fleet = None
        if self._gcs is not None:
            self._gcs.stop()
            self._gcs = None
        self._views.clear()

    def pause(self) -> None:
        """Hold every node in place while the controller decides."""
        self._paused = True
        self._timer.stop()
        self._broadcast(CommandName.HOLD)

    def resume(self) -> None:
        self._paused = False
        for sysid, view in self._views.items():
            if not view.is_done and sysid not in self._emergency and not view.gps_hold:
                self._send(sysid, CommandName.RESUME)
        if self._views:
            self._timer.start()

    # ---- Commands ----

    def retarget_all(self, destination: LatLon) -> int:
        active = [s for s, v in self._views.items() if not v.is_done]
        self._destination = destination
        altitude = next(iter(self._views.values())).config.cruise_altitude_m if self._views else 50.0
        for sysid in active:
            self._send(
                sysid,
                CommandName.NAVIGATE,
                source=None,
                destination=[destination.lat, destination.lon, altitude],
            )
        if active:
            self.retargeted.emit(len(active))
        return len(active)

    def fall_back(self, sysid: int) -> Optional[EmergencyLanding]:
        view = self._views.get(sysid)
        if view is None or view.is_done:
            return None
        self._emergency.add(sysid)
        if view.can_reach(view.start):
            self._send(sysid, CommandName.RETURN_TO_LAUNCH)
            return EmergencyLanding(
                target=view.start,
                distance_m=engine_haversine(
                    view._snap.position, Position(lat=view.start.lat, lon=view.start.lon)
                ),
                in_place=False,
            )
        self._send(sysid, CommandName.LAND)
        return EmergencyLanding(target=view.position, distance_m=0.0, in_place=True)

    def emergency_land(self, sysid: int, candidates: list[LatLon]) -> Optional[EmergencyLanding]:
        view = self._views.get(sysid)
        if view is None or view.is_done:
            return None
        self._emergency.add(sysid)
        reachable = [p for p in candidates if view.can_reach(p)]
        target = nearest_point(view.position, reachable)
        if target is None:
            self._send(sysid, CommandName.LAND)
            return EmergencyLanding(target=view.position, distance_m=0.0, in_place=True)
        altitude = view.config.cruise_altitude_m
        self._send(
            sysid, CommandName.NAVIGATE, destination=[target.lat, target.lon, altitude]
        )
        return EmergencyLanding(
            target=target,
            distance_m=engine_haversine(
                view._snap.position, Position(lat=target.lat, lon=target.lon)
            ),
            in_place=False,
        )

    def inject_fault(self, command: InjectFault) -> list[int]:
        """GPS loss is injected through the command link, as the nodes are
        simulated. Other fault types are not modelled here."""
        if command.fault_type is not FaultType.GPS_LOSS:
            return []
        affected = []
        for sysid in command.sysids:
            if sysid in self._views and not self._views[sysid].is_done:
                self._send(sysid, CommandName.SET_GPS, available=False)
                affected.append(sysid)
        return affected

    def clear_faults(self, sysids: Optional[list[int]] = None) -> list[int]:
        targets = sysids if sysids is not None else list(self._views)
        restored = []
        for sysid in targets:
            if sysid in self._views:
                self._send(sysid, CommandName.SET_GPS, available=True)
                restored.append(sysid)
        return restored

    def preflight_check(self, drones: list[DroneConfig], start: LatLon, destination: LatLon):
        flights = plan_flights(drones, start, destination)
        return [f for f in flights if f.feasible], [f for f in flights if not f.feasible]

    # ---- Internals ----

    def _send(self, sysid: int, command: CommandName, **kwargs) -> None:
        if self._gcs is not None:
            self._gcs.send(sysid, command, **kwargs)

    def _broadcast(self, command: CommandName, **kwargs) -> None:
        for sysid, view in self._views.items():
            if not view.is_done:
                self._send(sysid, command, **kwargs)

    def _poll(self) -> None:
        """Pull the latest snapshot from each node and publish one batch."""
        if self._fleet is None:
            return
        self._tick += 1
        self._elapsed_s += POLL_MS / 1000.0 * self._time_scale

        snapshots = self._fleet.latest()
        if not snapshots:
            return

        drones = [self._telemetry_of(s) for s in snapshots.values()]
        self.batch_ready.emit(
            SwarmTelemetryBatch(tick=self._tick, timestamp=self._elapsed_s, drones=drones)
        )

        remaining = 0.0
        for sysid, view in self._views.items():
            if view.is_done or self._destination is None:
                continue
            snap = view._snap
            target = Position(lat=self._destination.lat, lon=self._destination.lon)
            remaining = max(
                remaining,
                engine_haversine(snap.position, target) / max(0.1, view.cruise_speed_mps),
            )
        self.progress.emit(self._elapsed_s, self._elapsed_s + remaining)

        if all(v.is_done for v in self._views.values()):
            self._timer.stop()
            self.finished.emit(self._elapsed_s)
            return

        self._check_exceptions(snapshots)

    def _check_exceptions(self, snapshots: dict[int, DroneSnapshot]) -> None:
        """Surface the first situation needing a decision, and hold the swarm."""
        for sysid, snap in snapshots.items():
            if snap.mission_state is MissionState.FAILED and sysid not in self._flagged_lost:
                self._flagged_lost.add(sysid)
                self.drone_lost.emit(sysid)
                return

            if not snap.gps_available and sysid not in self._flagged_gps:
                self._flagged_gps.add(sysid)
                self._flagged_low.discard(sysid)  # a hold is a new situation
                self.pause()
                self.gps_lost.emit(sysid)
                return

            if snap.gps_available and sysid in self._flagged_gps:
                self._flagged_gps.discard(sysid)
                self.gps_restored.emit(sysid)
                return

            if (
                snap.battery <= LOW_BATTERY_PCT
                and sysid not in self._flagged_low
                and sysid not in self._emergency
                and snap.mission_state not in (MissionState.LANDED, MissionState.FAILED)
            ):
                self._flagged_low.add(sysid)
                self.pause()
                self.low_battery.emit(sysid, snap.battery)
                return

    def _telemetry_of(self, snap: DroneSnapshot) -> DroneTelemetry:
        faults: list[FaultType] = []
        if not snap.gps_available:
            faults.append(FaultType.GPS_LOSS)
        if snap.health_status is HealthStatus.FAILED:
            faults.append(FaultType.BATTERY_FAIL)

        return DroneTelemetry(
            sysid=snap.drone_id,
            status=_STATUS_MAP.get(snap.mission_state, DroneStatus.STANDBY),
            lat=snap.position.lat,
            lon=snap.position.lon,
            altitude_m=snap.position.alt_m,
            heading_deg=snap.velocity.heading_deg,
            ground_speed_mps=snap.velocity.ground_speed_mps,
            roll_deg=0.0,
            pitch_deg=-5.0 if snap.mission_state is MissionState.EN_ROUTE else 0.0,
            battery_pct=max(0.0, min(100.0, snap.battery)),
            link_quality_pct=100.0 if snap.communication_status.value == "ONLINE" else 40.0,
            active_faults=faults,
            raw_mavlink=(
                f"[{snap.drone_id}] {snap.mission_state.value} "
                f"pos=({snap.position.lat:.6f},{snap.position.lon:.6f},{snap.position.alt_m:.1f}) "
                f"batt={snap.battery:.1f}% {snap.health_status.value}"
                + (f" | {snap.note}" if snap.note else "")
            ),
        )
