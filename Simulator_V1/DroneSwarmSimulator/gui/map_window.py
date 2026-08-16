from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Callable, Dict, List, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from PySide6.QtCore import QObject, QUrl, Signal, Slot
from PySide6.QtWebChannel import QWebChannel
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from PySide6.QtWebEngineCore import QWebEngineSettings
from PySide6.QtWebEngineWidgets import QWebEngineView

from engine.drone_runtime import (
    DroneState,
    build_default_mission,
    build_mavlink_plan,
    create_drone_state,
    interpolate_route,
    load_plan_from_file,
    route_points_for_drone,
)
from orchestration.drone_controller import DroneOrchestrator


class MapBridge(QObject):
    def __init__(self, window: "DroneMapWindow"):
        super().__init__()
        self.window = window

    @Slot(float, float)
    def setSelectedEndPoint(self, lat: float, lon: float):
        self.window.set_selected_end_point(lat, lon)


class DroneMapWindow(QMainWindow):
    status_signal = Signal(object)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Swarm Simulator")
        self.resize(1400, 900)

        self.drones: Dict[str, DroneState] = {}
        self.orchestrator = DroneOrchestrator()
        self._status_callback: Optional[Callable] = None
        self.map_bridge = MapBridge(self)
        self._map_ready = False

        self._apply_theme()
        self._build_ui()
        self._init_map()

    def _apply_theme(self):
        self.setStyleSheet(
            """
            QMainWindow {
                background: #0b1220;
                color: #edf2ff;
            }
            QWidget {
                color: #edf2ff;
                font-family: "Segoe UI", sans-serif;
            }
            QGroupBox {
                background: #111a2b;
                border: 1px solid #2b3a52;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 12px;
                color: #edf2ff;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px 0 6px;
                color: #9bd0ff;
                font-size: 11px;
                font-weight: 600;
            }
            QLineEdit, QTextEdit, QListWidget, QTableWidget, QComboBox {
                background: #151f2f;
                border: 1px solid #2b3a52;
                border-radius: 8px;
                color: #edf2ff;
                padding: 7px 10px;
            }
            QPushButton {
                border: 1px solid #2d6cdf;
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #1d4ed8, stop:1 #0f172a);
                color: white;
                border-radius: 8px;
                padding: 8px 12px;
                font-weight: 600;
            }
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #2563eb, stop:1 #111827);
            }
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #1e3a8a, stop:1 #0f172a);
            }
            QLabel {
                color: #dfe7ff;
            }
            QTableWidget {
                gridline-color: #26354f;
                alternate-background-color: #101a2a;
                background: #111b2a;
            }
            QHeaderView::section {
                background: #182338;
                color: #dfe7ff;
                border: 1px solid #2b3a52;
                padding: 6px;
            }
            """
        )

    def _build_ui(self):
        central = QWidget(self)
        self.setCentralWidget(central)
        central.setStyleSheet("background: #0b1220;")

        root_layout = QHBoxLayout(central)
        root_layout.setContentsMargins(12, 12, 12, 12)
        root_layout.setSpacing(12)

        left_panel = QVBoxLayout()
        right_panel = QVBoxLayout()

        control_panel = QGroupBox("Mission controls")
        control_layout = QVBoxLayout(control_panel)

        self.play_all_btn = QPushButton("Play all")
        self.stop_all_btn = QPushButton("Stop all")
        self.refresh_fleet_btn = QPushButton("Refresh fleet")
        self.select_on_map_btn = QPushButton("Select endpoint on map")
        self.play_all_btn.clicked.connect(self.play_all_drones)
        self.stop_all_btn.clicked.connect(self.stop_all_drones)
        self.refresh_fleet_btn.clicked.connect(self.refresh_fleet_view)
        self.select_on_map_btn.clicked.connect(self.select_map_endpoint)
        control_layout.addWidget(self.play_all_btn)
        control_layout.addWidget(self.stop_all_btn)
        control_layout.addWidget(self.refresh_fleet_btn)
        control_layout.addWidget(self.select_on_map_btn)
        left_panel.addWidget(control_panel)

        drone_form = QGroupBox("Drone creation")
        form_layout = QFormLayout(drone_form)

        self.drone_id_input = QLineEdit()
        self.drone_name_input = QLineEdit()
        self.features_input = QLineEdit("mapping,inspection")
        self.protocol_input = QLineEdit("MAVLINK")
        self.start_lat_input = QLineEdit("12.9716")
        self.start_lon_input = QLineEdit("77.5946")
        self.end_lat_input = QLineEdit("12.9750")
        self.end_lon_input = QLineEdit("77.6000")

        form_layout.addRow("Drone ID", self.drone_id_input)
        form_layout.addRow("Drone name", self.drone_name_input)
        form_layout.addRow("Features", self.features_input)
        form_layout.addRow("Protocol", self.protocol_input)
        form_layout.addRow("Start lat", self.start_lat_input)
        form_layout.addRow("Start lon", self.start_lon_input)
        form_layout.addRow("End lat", self.end_lat_input)
        form_layout.addRow("End lon", self.end_lon_input)

        btn_row = QHBoxLayout()
        add_button = QPushButton("Add drone")
        add_button.clicked.connect(self.add_drone)
        btn_row.addWidget(add_button)

        left_panel.addWidget(drone_form)
        left_panel.addLayout(btn_row)

        self.drone_list = QListWidget()
        self.drone_list.itemClicked.connect(self.on_drone_selected)
        left_panel.addWidget(self.drone_list)

        self.telemetry_table = QTableWidget(0, 5)
        self.telemetry_table.setHorizontalHeaderLabels(["Drone", "Status", "Lat", "Lon", "Alt"])
        self.telemetry_table.setMinimumHeight(180)
        left_panel.addWidget(self.telemetry_table)

        buttons = QHBoxLayout()
        self.load_plan_btn = QPushButton("Upload plan")
        self.play_btn = QPushButton("Play")
        self.stop_btn = QPushButton("Stop")

        self.load_plan_btn.clicked.connect(self.load_plan)
        self.play_btn.clicked.connect(self.play_selected_drone)
        self.stop_btn.clicked.connect(self.stop_selected_drone)

        buttons.addWidget(self.load_plan_btn)
        buttons.addWidget(self.play_btn)
        buttons.addWidget(self.stop_btn)
        left_panel.addLayout(buttons)

        self.command_view = QTextEdit()
        self.command_view.setReadOnly(True)
        self.command_view.setPlaceholderText("Mission commands will appear here")
        left_panel.addWidget(self.command_view)

        self.status_label = QLabel("Status: no drone selected")
        left_panel.addWidget(self.status_label)

        self.map_view = QWebEngineView()
        self.map_view.setMinimumWidth(800)

        settings = self.map_view.settings()
        settings.setAttribute(QWebEngineSettings.LocalContentCanAccessRemoteUrls, True)
        settings.setAttribute(QWebEngineSettings.LocalContentCanAccessFileUrls, True)
        settings.setAttribute(QWebEngineSettings.JavascriptEnabled, True)

        right_panel.addWidget(self.map_view)

        self.map_status = QLabel("Fleet status: idle")
        right_panel.addWidget(self.map_status)

        root_layout.addLayout(left_panel, 1)
        root_layout.addLayout(right_panel, 3)

        self.status_signal.connect(self.on_drone_status_update)
        self._status_callback = self.status_signal.emit
        self.map_view.page().loadFinished.connect(self._on_map_loaded)

    def _on_map_loaded(self, ok: bool):
        self._map_ready = ok
        if ok:
            for drone in self.drones.values():
                self._render_route(drone)

    def _init_map(self):
        html_path = PROJECT_ROOT / "resources" / "map.html"
        page = self.map_view.page()

        if page is not None:
            channel = QWebChannel(page)
            channel.registerObject("mapBridge", self.map_bridge)
            page.setWebChannel(channel)

        if html_path.exists():
            self.map_view.setUrl(QUrl.fromLocalFile(str(html_path)))
        else:
            self.map_view.setUrl(QUrl("https://www.openstreetmap.org/#map=14/12.9716/77.5946"))

    def _run_map_script(self, script: str):
        if self.map_view.page() is not None and self._map_ready:
            self.map_view.page().runJavaScript(script)

    def add_drone(self):
        drone_id = self.drone_id_input.text().strip()
        name = self.drone_name_input.text().strip() or drone_id or "Drone"

        if not drone_id:
            QMessageBox.warning(self, "Missing data", "Drone ID is required")
            return

        features = [item.strip() for item in self.features_input.text().split(",") if item.strip()]
        protocol = self.protocol_input.text().strip() or "MAVLINK"

        try:
            start_lat = float(self.start_lat_input.text())
            start_lon = float(self.start_lon_input.text())
            end_lat = float(self.end_lat_input.text())
            end_lon = float(self.end_lon_input.text())
        except ValueError:
            QMessageBox.warning(self, "Invalid coordinates", "Lat/lon values must be numeric")
            return

        drone = create_drone_state(
            drone_id=drone_id,
            name=name,
            features=features,
            protocol=protocol,
            start_lat=start_lat,
            start_lon=start_lon,
            end_lat=end_lat,
            end_lon=end_lon,
        )
        drone.plan = build_default_mission(start_lat, start_lon, end_lat, end_lon)

        self.drones[drone_id] = drone
        self.command_view.setPlainText("\n".join(build_mavlink_plan(drone.plan)))
        item = QListWidgetItem(f"{drone.drone_id} - {drone.name}")
        self.drone_list.addItem(item)
        self._render_route(drone)
        self.refresh_fleet_view()
        self.status_label.setText(f"Status: {drone.drone_id} created with default mission")
        self._clear_drone_form()

    def _clear_drone_form(self):
        self.drone_id_input.clear()
        self.drone_name_input.clear()
        self.features_input.setText("mapping,inspection")
        self.protocol_input.setText("MAVLINK")
        self.start_lat_input.setText("12.9716")
        self.start_lon_input.setText("77.5946")
        self.end_lat_input.setText("12.9750")
        self.end_lon_input.setText("77.6000")

    def on_drone_selected(self, item: QListWidgetItem):
        drone_id = item.text().split(" - ", 1)[0]
        drone = self.drones.get(drone_id)
        if drone:
            self.status_label.setText(f"Status: {drone.drone_id} | {drone.status} | {drone.current_lat}, {drone.current_lon}")

    def load_plan(self):
        selected = self.drone_list.currentItem()
        if not selected:
            QMessageBox.warning(self, "No drone selected", "Please select a drone before uploading a plan")
            return

        drone_id = selected.text().split(" - ", 1)[0]
        plan_path, _ = QFileDialog.getOpenFileName(self, "Select drone plan", str(Path.cwd()), "Text files (*.txt *.csv)")
        if not plan_path:
            return

        drone = self.drones.get(drone_id)
        if not drone:
            return

        drone.plan = load_plan_from_file(plan_path)
        rendered_plan = build_mavlink_plan(drone.plan)
        self.command_view.setPlainText("\n".join(rendered_plan))
        self._render_route(drone)
        self.refresh_fleet_view()
        self.status_label.setText(f"Status: {drone_id} plan loaded ({len(drone.plan)} commands)")

    def _ensure_drone_threads_for_play(self):
        for drone_id, drone in list(self.drones.items()):
            if not drone.plan:
                continue
            worker = self.orchestrator.create_drone_thread(drone, drone.plan)
            worker.set_status_callback(self._status_callback)

    def play_selected_drone(self):
        selected = self.drone_list.currentItem()
        if not selected:
            QMessageBox.warning(self, "No drone selected", "Choose a drone to play")
            return

        drone_id = selected.text().split(" - ", 1)[0]
        drone = self.drones.get(drone_id)
        if not drone:
            return

        if not drone.plan:
            QMessageBox.warning(self, "Empty plan", "Upload a mission plan before clicking Play")
            return

        self._ensure_drone_threads_for_play()
        worker = self.orchestrator.create_drone_thread(drone, drone.plan)
        worker.set_status_callback(self._status_callback)
        self.orchestrator.start_drone(drone_id)
        self._render_route(drone)
        self.refresh_fleet_view()
        self.status_label.setText(f"Status: {drone_id} running")

    def stop_selected_drone(self):
        selected = self.drone_list.currentItem()
        if not selected:
            return

        drone_id = selected.text().split(" - ", 1)[0]
        self.orchestrator.stop_drone(drone_id)
        self.refresh_fleet_view()
        self.status_label.setText(f"Status: {drone_id} stopped")

    def play_all_drones(self):
        if not self.drones:
            QMessageBox.warning(self, "No drones", "Add at least one drone before pressing Play all")
            return

        for drone in self.drones.values():
            if not drone.plan:
                QMessageBox.warning(self, "Mission missing", f"Drone {drone.drone_id} has no uploaded mission plan")
                return

        self._ensure_drone_threads_for_play()
        for drone_id, drone in self.drones.items():
            worker = self.orchestrator.create_drone_thread(drone, drone.plan)
            worker.set_status_callback(self._status_callback)
            self.orchestrator.start_drone(drone_id)
            self._render_route(drone)

        self.map_status.setText("Fleet status: all drones launched")
        self.refresh_fleet_view()

    def stop_all_drones(self):
        self.orchestrator.stop_all()
        self.map_status.setText("Fleet status: all drones stopped")
        self.refresh_fleet_view()

    def refresh_fleet_view(self):
        telemetry = self.orchestrator.telemetry_snapshot(self.drones)
        self.telemetry_table.setRowCount(len(telemetry))
        for row_index, item in enumerate(telemetry):
            self.telemetry_table.setItem(row_index, 0, QTableWidgetItem(item.drone_id))
            self.telemetry_table.setItem(row_index, 1, QTableWidgetItem(item.status))
            self.telemetry_table.setItem(row_index, 2, QTableWidgetItem(f"{item.current_lat:.5f}"))
            self.telemetry_table.setItem(row_index, 3, QTableWidgetItem(f"{item.current_lon:.5f}"))
            self.telemetry_table.setItem(row_index, 4, QTableWidgetItem(f"{item.current_alt:.2f}"))

        self.map_status.setText(f"Fleet status: {len(telemetry)} drone(s) online")

    @Slot(object)
    def on_drone_status_update(self, status):
        drone = self.drones.get(status.drone_id)
        if drone is not None:
            drone.status = status.status
            drone.current_lat = status.current_lat
            drone.current_lon = status.current_lon
            drone.current_alt = status.current_alt
        self.status_label.setText(
            f"Status: {status.drone_id} | {status.status} | lat={status.current_lat}, lon={status.current_lon}, alt={status.current_alt}m | {status.message}"
        )
        self.refresh_fleet_view()
        self._update_map_marker(status)

    def set_selected_end_point(self, lat: float, lon: float):
        selected = self.drone_list.currentItem()
        if not selected:
            return

        drone_id = selected.text().split(" - ", 1)[0]
        drone = self.drones.get(drone_id)
        if drone is not None:
            drone.end_lat = float(lat)
            drone.end_lon = float(lon)
            self._render_route(drone)
            self.status_label.setText(f"Status: {drone_id} endpoint set to {lat}, {lon}")

    def _render_route(self, drone: DroneState):
        points = route_points_for_drone({
            "start_lat": drone.start_lat,
            "start_lon": drone.start_lon,
            "end_lat": drone.end_lat,
            "end_lon": drone.end_lon,
            "plan": drone.plan,
        })
        payload = {"points": [[lat, lon] for lat, lon in points]}
        script = f"window.renderRoute({json.dumps(payload)});"
        self._run_map_script(script)

    def select_map_endpoint(self):
        selected = self.drone_list.currentItem()
        if not selected:
            QMessageBox.warning(self, "No drone selected", "Select a drone before choosing an endpoint")
            return

        drone_id = selected.text().split(" - ", 1)[0]
        drone = self.drones.get(drone_id)
        if not drone:
            return

        script = "window.requestMapClick();"
        self._run_map_script(script)
        QMessageBox.information(self, "Map click", "Click a point on the map to set the drone end point.")

    def _update_map_marker(self, status):
        payload = {
            "drone_id": status.drone_id,
            "lat": status.current_lat,
            "lon": status.current_lon,
            "alt": status.current_alt,
            "status": status.status,
        }
        script = f"window.renderDroneMarker({json.dumps(payload)});"
        self._run_map_script(script)


def main():
    app = QApplication([])
    window = DroneMapWindow()
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
