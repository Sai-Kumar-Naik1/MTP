"""Module 1 - GUI Application main window (SRS §3).

Ties together the Drone Management Panel, Map Viewer, Live Telemetry
Dashboard and Fault Injection Controls, wiring user interaction through to
Module 2's REST/WebSocket contract via `services.api_client.OrchestrationClient`.
"""
from __future__ import annotations

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QComboBox,
    QDockWidget,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QToolBar,
    QWidget,
)

from contracts.gui_orchestration import FlockCommand, LatLon
from gui.drone_management import DroneManagementPanel
from gui.fault_injection import FaultInjectionPanel
from gui.map_viewer import MapViewer
from gui.mission_planner_panel import MissionPlannerPanel
from gui.telemetry_dashboard import TelemetryDashboard
from services.api_client import DEFAULT_BASE_URL, DEFAULT_WS_URL, OrchestrationClient
from services.mavlink_backend import MavlinkSwarmBackend
from services.plan_service import PlanRunResult, PlanService
from services.thread_backend import ThreadSwarmBackend
from services.local_flight import (
    LOW_BATTERY_PCT,
    LocalFlightSimulator,
    format_duration,
    haversine_m,
    nearest_point,
    plan_flights,
    plan_route_flights,
)
from services.storage import ProfileStore

NFZ_CORNER_COUNT = 4  # a restricted area is a quadrilateral, click order = winding order


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Swarm Simulator - GUI (Module 1)")
        self.resize(1440, 900)

        self.store = ProfileStore()
        self.client = OrchestrationClient(DEFAULT_BASE_URL, DEFAULT_WS_URL, parent=self)

        # Start/destination picked on the map, and the local preview that flies
        # between them when Module 2 is not supplying telemetry.
        self._start_point: LatLon | None = None
        self._destination_point: LatLon | None = None
        # Every point the controller has placed, in order - an emergency
        # landing diverts to whichever of these is nearest.
        self._landing_candidates: list[LatLon] = []
        self._connected = False

        # PDDL mission planning: while armed, the next map clicks are a
        # plan's start, then destination, then (if the panel's checkbox asks
        # for one) the restricted area's corners - in that order, regardless
        # of the map's own "Click mode" dropdown, which this drives itself
        # so the user never has to operate it by hand mid-sequence.
        self.plan_service = PlanService(self)
        self._planning_mode = False
        self._plan_mark_nfz = False
        self._pending_plan_name: str | None = None
        self._plan_source: LatLon | None = None
        self._plan_destination: LatLon | None = None
        self._plan_no_fly_zone: list[LatLon] = []

        # Two interchangeable flight sources with the same surface: the local
        # kinematic preview, and real MAVLink to SITL/autopilots. `flight_sim`
        # always points at whichever is selected.
        self.local_sim = LocalFlightSimulator(self)
        self.mavlink_sim = MavlinkSwarmBackend(self)
        self.thread_sim = ThreadSwarmBackend(self)
        self.flight_sim = self.local_sim

        self.drone_management = DroneManagementPanel(self.store)
        self.map_viewer = MapViewer()
        self.telemetry_dashboard = TelemetryDashboard()
        self.fault_injection = FaultInjectionPanel()
        self.mission_planner = MissionPlannerPanel()

        self.setCentralWidget(self.map_viewer)

        self.left_dock = QDockWidget("Drone Management", self)
        self.left_dock.setWidget(self.drone_management)
        self.left_dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.left_dock)

        self.right_dock = QDockWidget("Fault Injection", self)
        self.right_dock.setWidget(self.fault_injection)
        self.right_dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.RightDockWidgetArea, self.right_dock)

        self.planner_dock = QDockWidget("Mission Planner", self)
        self.planner_dock.setWidget(self.mission_planner)
        self.planner_dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.RightDockWidgetArea, self.planner_dock)
        self.splitDockWidget(self.right_dock, self.planner_dock, Qt.Vertical)

        # Working Area spans the full window width beneath the drone, map and
        # fault panels, so the map keeps the whole central area.
        self.setCorner(Qt.BottomLeftCorner, Qt.BottomDockWidgetArea)
        self.setCorner(Qt.BottomRightCorner, Qt.BottomDockWidgetArea)

        self.region_dock = QDockWidget("Working Area (Bounding Box)", self)
        self.region_dock.setWidget(self.map_viewer.region_panel())
        self.region_dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.region_dock)

        self.bottom_dock = QDockWidget("Live Telemetry", self)
        self.bottom_dock.setWidget(self.telemetry_dashboard)
        self.bottom_dock.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable)
        self.addDockWidget(Qt.BottomDockWidgetArea, self.bottom_dock)
        # Stack them: Working Area strip on top, telemetry below it.
        self.splitDockWidget(self.region_dock, self.bottom_dock, Qt.Vertical)

        self._build_connection_toolbar()
        self._build_status_bar()
        self._wire_signals()

        # Give the map the majority of the window by default; docks can
        # still be dragged wider/taller by the user at any time.
        QTimer.singleShot(0, self._tune_initial_dock_sizes)

    def _tune_initial_dock_sizes(self) -> None:
        self.resizeDocks([self.left_dock, self.right_dock], [300, 300], Qt.Horizontal)
        self.resizeDocks(
            [self.region_dock, self.bottom_dock],
            [self.region_dock.sizeHint().height(), max(220, int(self.height() * 0.28))],
            Qt.Vertical,
        )

    # ---- Setup ----

    def _build_connection_toolbar(self) -> None:
        toolbar = QToolBar("Orchestrator Connection", self)
        toolbar.setMovable(False)
        self.addToolBar(toolbar)

        self.base_url_edit = QLineEdit(DEFAULT_BASE_URL)
        self.base_url_edit.setMinimumWidth(220)
        self.ws_url_edit = QLineEdit(DEFAULT_WS_URL)
        self.ws_url_edit.setMinimumWidth(220)
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self._on_connect_clicked)

        self.backend_combo = QComboBox()
        self.backend_combo.addItem("Local preview", "local")
        self.backend_combo.addItem("Drone threads (UDP)", "threads")
        self.backend_combo.addItem("MAVLink (SITL)", "mavlink")
        self.backend_combo.currentIndexChanged.connect(self._on_backend_changed)

        container = QWidget()
        row = QHBoxLayout(container)
        row.setContentsMargins(4, 0, 4, 0)
        row.addWidget(QLabel("Module 2 REST:"))
        row.addWidget(self.base_url_edit)
        row.addWidget(QLabel("WS:"))
        row.addWidget(self.ws_url_edit)
        row.addWidget(self.connect_btn)
        row.addWidget(QLabel("Flight source:"))
        row.addWidget(self.backend_combo)
        toolbar.addWidget(container)

    def _on_backend_changed(self) -> None:
        """Swap the active flight source. Both expose the same surface, so
        nothing else in the window changes."""
        self.local_sim.stop()
        self.mavlink_sim.stop()
        self.thread_sim.stop()
        choice = self.backend_combo.currentData()
        self.flight_sim = {
            "mavlink": self.mavlink_sim,
            "threads": self.thread_sim,
        }.get(choice, self.local_sim)
        self.statusBar().showMessage(
            {
                "mavlink": "Flight source: real MAVLink to SITL (start SITL before pressing Emulate).",
                "threads": "Flight source: independent drone threads, commanded over UDP.",
            }.get(choice, "Flight source: local kinematic preview.")
        )

    def _build_status_bar(self) -> None:
        self.flight_label = QLabel("No flight")
        self.statusBar().addPermanentWidget(self.flight_label)

        self.connection_label = QLabel("Disconnected")
        self.connection_label.setStyleSheet("color: #e74c3c; font-weight: bold;")
        self.statusBar().addPermanentWidget(self.connection_label)
        self.statusBar().showMessage("Ready.")

    def _wire_signals(self) -> None:
        self.drone_management.emulate_requested.connect(self._on_emulate)
        self.drone_management.stop_requested.connect(self._on_stop)
        self.drone_management.status_message.connect(self.statusBar().showMessage)

        self.map_viewer.status_message.connect(self.statusBar().showMessage)
        self.map_viewer.point_picked.connect(self._on_point_picked)

        self.fault_injection.fault_requested.connect(self._on_fault_requested)
        self.fault_injection.status_message.connect(self.statusBar().showMessage)

        self.mission_planner.plan_requested.connect(self._on_plan_requested)
        self.plan_service.finished.connect(self._on_plan_ready)
        self.plan_service.failed.connect(self._on_plan_failed)

        self.client.telemetry_received.connect(self._on_telemetry)
        self.client.connection_state_changed.connect(self._on_connection_state_changed)
        self.client.request_failed.connect(self._on_request_failed)

        # Both flight sources feed the identical telemetry path as Module 2.
        for backend in (self.local_sim, self.mavlink_sim, self.thread_sim):
            backend.batch_ready.connect(self._on_telemetry)
            backend.progress.connect(self._on_flight_progress)
            backend.finished.connect(self._on_flight_finished)
            backend.low_battery.connect(self._on_low_battery)
            backend.drone_lost.connect(self._on_drone_lost)
            backend.gps_lost.connect(self._on_gps_lost)
            backend.gps_restored.connect(self._on_gps_restored)
        self.mavlink_sim.link_failed.connect(
            lambda msg: self.statusBar().showMessage(f"MAVLink: {msg}", 15000)
        )
        self.thread_sim.link_failed.connect(
            lambda msg: self.statusBar().showMessage(f"Drone node: {msg}", 15000)
        )

    # ---- Connection ----

    def _on_connect_clicked(self) -> None:
        self.client.set_endpoints(self.base_url_edit.text().strip(), self.ws_url_edit.text().strip())
        self.client.connect_telemetry()
        self.statusBar().showMessage("Connecting to Module 2 telemetry stream...")

    def _on_connection_state_changed(self, connected: bool) -> None:
        self._connected = connected
        if connected:
            # Real telemetry supersedes the local preview.
            self.flight_sim.stop()
            self.connection_label.setText("Connected")
            self.connection_label.setStyleSheet("color: #2ecc71; font-weight: bold;")
        else:
            self.connection_label.setText("Disconnected")
            self.connection_label.setStyleSheet("color: #e74c3c; font-weight: bold;")

    def _on_request_failed(self, tag: str, error: str) -> None:
        self.statusBar().showMessage(f"[{tag}] request failed: {error}", 5000)

    # ---- Drone management wiring ----

    def _warn_grounded(self, grounded: list, total_count: int, distance_km: float, label: str = "leg") -> None:
        """Tell the controller which drones the battery gate kept on the
        ground, if any. Shared by the straight-leg Emulate flow and the PDDL
        mission-planner flow."""
        if not grounded:
            return
        detail = "\n".join(
            f"- {f.config.name} (SYSID {f.config.sysid}): needs "
            f"{f.required_mah:,.0f} mAh but carries {f.config.battery_capacity_mah:,.0f} mAh "
            f"- range about {f.range_m / 1000:.1f} km at {f.cruise_speed_mps:.1f} m/s"
            for f in grounded
        )
        QMessageBox.warning(
            self,
            "Not enough battery",
            f"{len(grounded)} of {total_count} drone(s) cannot complete the "
            f"{distance_km:.1f} km {label} and will not take off:\n\n{detail}",
        )

    def _on_emulate(self, drones: list) -> None:
        self.client.set_endpoints(self.base_url_edit.text().strip(), self.ws_url_edit.text().strip())
        for drone in drones:
            self.client.register_drone(drone)
        self.client.connect_telemetry()
        self.fault_injection.update_active_sysids([d.sysid for d in drones])

        if self._connected:
            self.statusBar().showMessage(
                f"Emulating {len(drones)} drone(s). Module 2 is driving telemetry."
            )
            return

        # No orchestrator: fly the picked course locally so the run is visible.
        if self._start_point is None or self._destination_point is None:
            self.statusBar().showMessage(
                "Emulating - set a Start Point and a Destination Point on the map to see the flight."
            )
            return

        # Pre-flight battery check: a drone whose profile would draw more than
        # it carries stays on the ground; the rest fly the mission.
        flights = plan_flights(drones, self._start_point, self._destination_point)
        cleared = [f for f in flights if f.feasible]
        grounded = [f for f in flights if not f.feasible]
        distance_km = flights[0].distance_m / 1000.0

        self._warn_grounded(grounded, len(flights), distance_km)

        if not cleared:
            # Nothing launched, so unlock the configuration inputs again.
            self.drone_management.set_running(False)
            self.fault_injection.update_active_sysids([])
            self.statusBar().showMessage(
                f"Flight not started - no drone has the battery for {distance_km:.1f} km."
            )
            return

        self.fault_injection.update_active_sysids([f.config.sysid for f in cleared])
        self.map_viewer.clear_paths()

        if self.flight_sim is self.thread_sim:
            # One thread per drone, commanded over UDP by an in-process GCS.
            if not self.thread_sim.start_mission(
                [f.config for f in cleared], self._start_point, self._destination_point
            ):
                self.drone_management.set_running(False)
                self.statusBar().showMessage("Could not start drone nodes - see the log.")
                return
            self.statusBar().showMessage(
                f"Launched {len(cleared)} drone thread(s); NAVIGATE sent over UDP "
                f"({self.thread_sim.time_scale:.0f}x speed)."
            )
            return

        if self.flight_sim is self.mavlink_sim:
            # Real vehicles: the same screened drones, flown over MAVLink.
            self.mavlink_sim.start_mission(
                [f.config for f in cleared], self._start_point, self._destination_point
            )
            self.statusBar().showMessage(
                f"Connecting to {len(cleared)} SITL vehicle(s) over MAVLink and starting the mission..."
            )
            return

        if not self.flight_sim.start(cleared):
            return

        altitudes = {f.config.cruise_altitude_m for f in cleared}
        altitude_text = (
            f"{next(iter(altitudes)):.0f} m" if len(altitudes) == 1 else f"{min(altitudes):.0f}-{max(altitudes):.0f} m"
        )
        grounded_text = f", {len(grounded)} grounded on battery" if grounded else ""
        self.statusBar().showMessage(
            f"Flying {len(cleared)} drone(s) {distance_km:.1f} km at {altitude_text}{grounded_text} "
            f"(Module 2 offline - local preview, {self.flight_sim.time_scale:.0f}x speed)."
        )

    def _on_flight_progress(self, elapsed_s: float, total_s: float) -> None:
        self.flight_label.setText(
            f"Flight {format_duration(elapsed_s)} / {format_duration(total_s)}"
            f"  (ETA {format_duration(total_s - elapsed_s)})"
        )

    def _on_fault_requested(self, command) -> None:
        """Faults go to Module 2 when it is there, and to the active flight
        source. Skipping the REST call while disconnected keeps its timeout
        failure from overwriting the decision this fault triggers."""
        if self._connected:
            self.client.inject_fault(command)
        if self.flight_sim.is_active():
            affected = self.flight_sim.inject_fault(command)
            if not affected:
                self.flight_sim.resume()

    def _on_gps_lost(self, sysid: int) -> None:
        """The drone reports GPS loss and is holding station pending orders."""
        flight = next((f for f in self.flight_sim.flights if f.config.sysid == sysid), None)
        home_note = ""
        if flight is not None:
            distance_km = haversine_m(flight.position, flight.start) / 1000.0
            reach = (
                "within reach on the remaining charge"
                if flight.can_reach(flight.start)
                else "TOO FAR on the remaining charge - it would land where it stands instead"
            )
            home_note = (
                f"\n\nStart point is {distance_km:.2f} km back - {reach}."
                f"\nBattery is at {flight.battery_pct:.0f}%."
            )

        granted = QMessageBox.question(
            self,
            "GPS lost - fall back?",
            f"SYSID {sysid} reports GPS LOSS and is holding position.{home_note}\n\n"
            "Grant permission to fall back to the start point and land?\n\n"
            "Yes - abandon the mission and return to the start point.\n"
            "No  - hold position until GPS returns (the battery keeps draining).",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )

        if granted == QMessageBox.Yes:
            landing = self.flight_sim.fall_back(sysid)
            if landing is None:
                self.statusBar().showMessage(f"SYSID {sysid}: unable to fall back.")
            elif landing.in_place:
                self.statusBar().showMessage(
                    f"SYSID {sysid}: start point out of range - landing immediately at "
                    f"({landing.target.lat:.5f}, {landing.target.lon:.5f})."
                )
            else:
                self.statusBar().showMessage(
                    f"SYSID {sysid} falling back {landing.distance_m / 1000:.2f} km to the start point."
                )
        else:
            self.statusBar().showMessage(
                f"SYSID {sysid}: fall-back denied - holding position until GPS returns."
            )
        self.flight_sim.resume()

    def _on_gps_restored(self, sysid: int) -> None:
        self.statusBar().showMessage(f"SYSID {sysid}: GPS reacquired - resuming the mission.", 8000)

    def _on_low_battery(self, sysid: int, battery_pct: float) -> None:
        """Battery hit the threshold: hold the mission and ask the controller.

        The simulator has already paused itself, so the drone is not draining
        while the dialog is up - at the compressed timescale the decision
        window would otherwise be a fraction of a second.
        """
        candidates = self._landing_candidates
        nearest_note = ""
        flight = next((f for f in self.flight_sim.flights if f.config.sysid == sysid), None)
        if flight is not None and candidates:
            target = nearest_point(flight.position, candidates)
            if target is not None:
                distance_km = haversine_m(flight.position, target) / 1000.0
                reach = (
                    "within reach on the remaining charge"
                    if flight.can_reach(target)
                    else "TOO FAR on the remaining charge - it would land where it stands instead"
                )
                nearest_note = (
                    f"\n\nNearest landing point is ({target.lat:.5f}, {target.lon:.5f}), "
                    f"{distance_km:.2f} km away - {reach}."
                )

        holding = flight is not None and flight.gps_hold
        if holding:
            title = "Low battery while holding - fall back?"
            headline = (
                f"SYSID {sysid} has been holding without GPS and is down to "
                f"{battery_pct:.0f}% battery (threshold {LOW_BATTERY_PCT:.0f}%)."
            )
            deny_line = "No  - keep holding for GPS; the drone will be lost when the battery runs flat."
        else:
            title = "Low battery - emergency landing?"
            headline = (
                f"SYSID {sysid} is down to {battery_pct:.0f}% battery "
                f"(threshold {LOW_BATTERY_PCT:.0f}%)."
            )
            deny_line = "No  - press on to the destination; the drone will be lost when the battery runs flat."

        granted = QMessageBox.question(
            self,
            title,
            f"{headline}{nearest_note}\n\n"
            "Grant permission to break off and land at the nearest point?\n\n"
            "Yes - divert and land now.\n"
            f"{deny_line}",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes,
        )

        if granted == QMessageBox.Yes:
            landing = self.flight_sim.emergency_land(sysid, candidates)
            if landing is None:
                self.statusBar().showMessage(
                    f"SYSID {sysid}: no landing point available - continuing on mission."
                )
            elif landing.in_place:
                self.statusBar().showMessage(
                    f"SYSID {sysid}: no landing point within range - landing immediately at "
                    f"({landing.target.lat:.5f}, {landing.target.lon:.5f})."
                )
            else:
                self.statusBar().showMessage(
                    f"SYSID {sysid} diverting {landing.distance_m / 1000:.2f} km to emergency "
                    f"landing at ({landing.target.lat:.5f}, {landing.target.lon:.5f})."
                )
        else:
            self.statusBar().showMessage(
                f"SYSID {sysid}: emergency landing denied - pressing on until the battery is flat."
            )
        self.flight_sim.resume()

    def _on_drone_lost(self, sysid: int) -> None:
        self.statusBar().showMessage(
            f"SYSID {sysid} LOST - battery exhausted in flight, drone sacrificed.", 15000
        )

    def _on_flight_finished(self, total_s: float) -> None:
        lost = [f.config.sysid for f in self.flight_sim.flights if f.is_lost]
        landed = [f.config.sysid for f in self.flight_sim.flights if not f.is_lost]
        self.flight_label.setText(f"Flight complete - {format_duration(total_s)}")
        summary = f"Mission over. Time consumed: {format_duration(total_s)} of flight time."
        if landed:
            summary += f" Landed: {', '.join(f'SYSID {s}' for s in landed)}."
        if lost:
            summary += f" Lost to flat battery: {', '.join(f'SYSID {s}' for s in lost)}."
        self.statusBar().showMessage(summary)

    def _on_stop(self) -> None:
        self.flight_sim.stop()
        self.flight_label.setText("No flight")
        # Markers are cleared below, so the points they stood for go too.
        self._landing_candidates.clear()
        self._start_point = None
        self._destination_point = None
        self._planning_mode = False
        self._pending_plan_name = None
        self._plan_source = None
        self._plan_destination = None
        self._plan_no_fly_zone = []
        self.client.stop_swarm()
        self.client.disconnect_telemetry()
        self.map_viewer.clear_markers()
        self.map_viewer.clear_paths()
        self.map_viewer.clear_restricted_area()
        self.fault_injection.update_active_sysids([])
        self.statusBar().showMessage("Swarm stopped. Configuration inputs unlocked.")

    # ---- Map wiring ----

    def _on_point_picked(self, role: str, lat: float, lon: float) -> None:
        point = LatLon(lat=lat, lon=lon)
        self._landing_candidates.append(point)

        if self._planning_mode:
            # Order-driven, not role-driven: this is the Nth click of the
            # plan's sequence regardless of what the map's "Click mode"
            # dropdown says, because this code is what drives that dropdown -
            # the user never has to operate it themselves mid-sequence.
            self._handle_plan_click(point)
            return

        if role != "destination":
            self._start_point = point
            self.statusBar().showMessage(f"Start point set at ({lat:.5f}, {lon:.5f}).")
            return

        self._destination_point = point

        # A destination placed mid-mission is a diversion order: the drones
        # already airborne turn for it from wherever they are.
        if self.flight_sim.is_active():
            diverted = self.flight_sim.retarget_all(point)
            self.flight_sim.resume()
            self.statusBar().showMessage(
                f"Re-routing {diverted} drone(s) to the new destination ({lat:.5f}, {lon:.5f})."
            )
            return

        sysids = self.drone_management.checked_sysids()
        if not sysids:
            QMessageBox.information(
                self, "No active drones", "Check drones in the Drone Management panel before issuing a flock command."
            )
            return
        if self._connected:
            self.client.flock(FlockCommand(sysids=sysids, destination=point))
        self.statusBar().showMessage(
            f"Destination set for {len(sysids)} drone(s) -> ({lat:.5f}, {lon:.5f})."
        )

    # ---- Mission planner wiring ----

    def _on_plan_requested(self, plan_name: str) -> None:
        """Arm the map: the next clicks belong to this plan, not to the
        ordinary flock-command flow - Start and Destination always, plus
        `NFZ_CORNER_COUNT` restricted-area corners if the panel's checkbox
        asks for one."""
        self._pending_plan_name = plan_name
        self._plan_source = None
        self._plan_destination = None
        self._plan_no_fly_zone = []
        self._plan_mark_nfz = self.mission_planner.mark_restricted_area()
        self._planning_mode = True
        self.map_viewer.clear_markers()
        self.map_viewer.clear_restricted_area()
        self.map_viewer.mode_combo.setCurrentText("Set Start Point")
        self.statusBar().showMessage(f"Plan '{plan_name}' armed - click the Start point on the map.")

    def _handle_plan_click(self, point: LatLon) -> None:
        """Consume one click of the armed plan's sequence: Start, then
        Destination, then (if requested) `NFZ_CORNER_COUNT` restricted-area
        corners - driving the map's Click mode as it goes, so the dropdown
        never has to be touched by hand."""
        name = self._pending_plan_name

        if self._plan_source is None:
            self._plan_source = point
            self.map_viewer.mode_combo.setCurrentText("Set Destination Point")
            self.statusBar().showMessage(
                f"Plan '{name}': start set at ({point.lat:.5f}, {point.lon:.5f}) - "
                f"now click the destination."
            )
            return

        if self._plan_destination is None:
            self._plan_destination = point
            if self._plan_mark_nfz:
                self.map_viewer.mode_combo.setCurrentText("Set Restricted Area")
                self.statusBar().showMessage(
                    f"Plan '{name}': destination set - click the restricted area's "
                    f"corner 1 of {NFZ_CORNER_COUNT}."
                )
                return
            self._planning_mode = False
            self._run_planned_mission()
            return

        # Collecting the restricted-area's corners.
        self._plan_no_fly_zone.append(point)
        self.map_viewer.set_restricted_area(self._plan_no_fly_zone)
        collected = len(self._plan_no_fly_zone)
        if collected < NFZ_CORNER_COUNT:
            self.statusBar().showMessage(
                f"Plan '{name}': restricted-area corner {collected} of {NFZ_CORNER_COUNT} set - "
                f"click the next corner."
            )
            return

        self._planning_mode = False
        self._run_planned_mission()

    def _run_planned_mission(self) -> None:
        if (
            self._plan_source is None
            or self._plan_destination is None
            or self._pending_plan_name is None
        ):
            return
        if not self.drone_management.checked_drones():
            QMessageBox.information(
                self, "No active drones",
                "Check drones in the Drone Management panel before planning a mission.",
            )
            return
        self.mission_planner.set_status(f"Running ENHSP for plan '{self._pending_plan_name}'...")
        self.statusBar().showMessage(f"Running ENHSP for plan '{self._pending_plan_name}'...")
        self.plan_service.run_async(
            self._pending_plan_name, self._plan_source, self._plan_destination, self._plan_no_fly_zone
        )

    def _on_plan_ready(self, result: PlanRunResult) -> None:
        self.mission_planner.set_plan_steps(result.plan_name, result.steps)

        drones = self.drone_management.checked_drones()
        if not drones:
            self.mission_planner.set_status(f"Plan '{result.plan_name}' ready, but no drones are checked.")
            self.statusBar().showMessage("Plan ready, but no drones are checked to fly it.")
            return

        flights = plan_route_flights(drones, result.waypoints)
        cleared = [f for f in flights if f.feasible]
        grounded = [f for f in flights if not f.feasible]
        distance_km = flights[0].distance_m / 1000.0 if flights else 0.0
        route_text = " -> ".join(result.location_names)

        self._warn_grounded(grounded, len(flights), distance_km, label="route")

        if not cleared:
            self.mission_planner.set_status(
                f"Plan '{result.plan_name}' ({route_text}, {distance_km:.1f} km): "
                f"no drone has the battery for it."
            )
            self.statusBar().showMessage(
                f"Plan '{result.plan_name}' ready, but no drone has the battery for the {distance_km:.1f} km route."
            )
            return

        # This mission always flies the "drone firmware" pipeline: Command ->
        # engine.mavlink_interpreter -> SimulatedFlightController.
        threads_index = self.backend_combo.findData("threads")
        if threads_index >= 0 and self.backend_combo.currentIndex() != threads_index:
            self.backend_combo.setCurrentIndex(threads_index)

        self.drone_management.set_running(True)
        self.fault_injection.update_active_sysids([f.config.sysid for f in cleared])
        self.map_viewer.clear_paths()

        if not self.thread_sim.start_mission_path([f.config for f in cleared], result.waypoints):
            self.drone_management.set_running(False)
            self.statusBar().showMessage("Could not start drone nodes for the planned mission - see the log.")
            return

        self.mission_planner.set_status(
            f"Flying plan '{result.plan_name}' ({route_text}, {distance_km:.1f} km) "
            f"with {len(cleared)} drone(s)."
        )
        self.statusBar().showMessage(
            f"Flying {len(cleared)} drone(s) on plan '{result.plan_name}': "
            f"{distance_km:.1f} km via {max(0, len(result.waypoints) - 2)} waypoint(s)."
        )
        self._pending_plan_name = None
        self._plan_source = None

    def _on_plan_failed(self, message: str) -> None:
        plan_name = self._pending_plan_name or "?"
        self.mission_planner.set_error(f"Plan '{plan_name}' failed:\n\n{message}")
        self.mission_planner.set_status("Planning failed - see the log below.")
        QMessageBox.warning(self, "Mission planning failed", message)
        self.statusBar().showMessage("Mission planning failed - see the dialog.", 8000)

    # ---- Telemetry wiring ----

    def _on_telemetry(self, batch) -> None:
        self.map_viewer.update_drones(batch.drones)
        self.telemetry_dashboard.update_drones(batch.drones)
        self.fault_injection.update_active_sysids([d.sysid for d in batch.drones])
