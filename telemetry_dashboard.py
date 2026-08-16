"""Live Telemetry Dashboard (SRS §3.2.3): Global State Matrix + Individual
Drone Inspector. Only the currently selected drone's inspector is actively
rendered, regardless of swarm size.
"""
from __future__ import annotations

from typing import Optional

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPlainTextEdit,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from contracts.gui_orchestration import DroneStatus, DroneTelemetry
from gui.artificial_horizon import ArtificialHorizon

COLUMNS = ["SYSID", "Status", "Altitude (m)", "Battery %", "Link Quality %", "Active Faults"]
FAULT_ROW_COLOR = QColor("#5c1e1e")
NORMAL_ROW_COLOR = QColor(Qt.transparent)
MAX_LOG_LINES = 500


class GlobalStateMatrix(QTableWidget):
    """Table of all active drones (SRS §3.2.3) with visual fault indication
    (SRS §3.2.4)."""

    drone_selected = Signal(int)

    def __init__(self, parent=None):
        super().__init__(0, len(COLUMNS), parent)
        self.setHorizontalHeaderLabels(COLUMNS)
        self.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.setSelectionBehavior(QTableWidget.SelectRows)
        self.setSelectionMode(QTableWidget.SingleSelection)
        self.setEditTriggers(QTableWidget.NoEditTriggers)
        self.itemSelectionChanged.connect(self._on_selection_changed)
        self._row_by_sysid: dict[int, int] = {}

    def update_drones(self, drones: list[DroneTelemetry]) -> None:
        selected_sysid = self.selected_sysid()
        self.setRowCount(len(drones))
        self._row_by_sysid.clear()
        for row, drone in enumerate(sorted(drones, key=lambda d: d.sysid)):
            self._row_by_sysid[drone.sysid] = row
            in_fault = bool(drone.active_faults) or drone.status == DroneStatus.FAILSAFE
            values = [
                str(drone.sysid),
                drone.status.value,
                f"{drone.altitude_m:.1f}",
                f"{drone.battery_pct:.0f}",
                f"{drone.link_quality_pct:.0f}",
                ", ".join(f.value for f in drone.active_faults) or "-",
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setBackground(FAULT_ROW_COLOR if in_fault else NORMAL_ROW_COLOR)
                self.setItem(row, col, item)
        if selected_sysid is not None and selected_sysid in self._row_by_sysid:
            self.selectRow(self._row_by_sysid[selected_sysid])

    def selected_sysid(self) -> Optional[int]:
        items = self.selectedItems()
        if not items:
            return None
        item = self.item(items[0].row(), 0)
        return int(item.text()) if item else None

    def _on_selection_changed(self) -> None:
        sysid = self.selected_sysid()
        if sysid is not None:
            self.drone_selected.emit(sysid)


class DroneInspector(QWidget):
    """Individual Drone Inspector - horizon, heading, ground speed, raw
    MAVLink log for exactly one selected drone (SRS §3.2.3). Only rendered
    for the currently selected drone, not all active drones at once."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._sysid: Optional[int] = None

        self.title_label = QLabel("No drone selected")
        self.title_label.setStyleSheet("font-weight: bold;")

        self.horizon = ArtificialHorizon()

        self.heading_label = QLabel("-")
        self.speed_label = QLabel("-")
        self.altitude_label = QLabel("-")
        self.battery_label = QLabel("-")

        stats_form = QGridLayout()
        for row, (caption, widget) in enumerate(
            [
                ("Heading", self.heading_label),
                ("Ground speed", self.speed_label),
                ("Altitude", self.altitude_label),
                ("Battery", self.battery_label),
            ]
        ):
            stats_form.addWidget(QLabel(caption + ":"), row, 0)
            stats_form.addWidget(widget, row, 1)

        top_row = QHBoxLayout()
        top_row.addWidget(self.horizon)
        top_row.addLayout(stats_form)
        top_row.addStretch(1)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(MAX_LOG_LINES)
        self.log_view.setPlaceholderText("Raw MAVLink message log for the selected drone...")

        layout = QVBoxLayout(self)
        layout.addWidget(self.title_label)
        layout.addLayout(top_row)
        layout.addWidget(QLabel("Raw MAVLink message log:"))
        layout.addWidget(self.log_view, stretch=1)

    def set_selected_sysid(self, sysid: int) -> None:
        if sysid != self._sysid:
            self.log_view.clear()
        self._sysid = sysid
        self.title_label.setText(f"Drone Inspector - SYSID {sysid}")

    def apply_telemetry(self, drone: DroneTelemetry) -> None:
        if drone.sysid != self._sysid:
            return
        self.horizon.set_attitude(drone.roll_deg, drone.pitch_deg)
        self.heading_label.setText(f"{drone.heading_deg:.0f} deg")
        self.speed_label.setText(f"{drone.ground_speed_mps:.1f} m/s")
        self.altitude_label.setText(f"{drone.altitude_m:.1f} m")
        self.battery_label.setText(f"{drone.battery_pct:.0f} %")
        if drone.raw_mavlink:
            self.log_view.appendPlainText(drone.raw_mavlink)


class TelemetryDashboard(QWidget):
    """Combines the Global State Matrix and the Individual Drone Inspector."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.matrix = GlobalStateMatrix()
        self.inspector = DroneInspector()
        self.matrix.drone_selected.connect(self.inspector.set_selected_sysid)

        splitter = QSplitter(Qt.Vertical)
        matrix_box = QGroupBox("Global State Matrix")
        matrix_layout = QVBoxLayout(matrix_box)
        matrix_layout.addWidget(self.matrix)

        inspector_box = QGroupBox("Individual Drone Inspector")
        inspector_layout = QVBoxLayout(inspector_box)
        inspector_layout.addWidget(self.inspector)

        splitter.addWidget(matrix_box)
        splitter.addWidget(inspector_box)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 1)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(splitter)

    def update_drones(self, drones: list[DroneTelemetry]) -> None:
        self.matrix.update_drones(drones)
        selected = self.matrix.selected_sysid()
        if selected is None and drones:
            self.matrix.selectRow(0)
            selected = self.matrix.selected_sysid()
        if selected is not None:
            self.inspector.set_selected_sysid(selected)
        for drone in drones:
            if drone.sysid == selected:
                self.inspector.apply_telemetry(drone)
