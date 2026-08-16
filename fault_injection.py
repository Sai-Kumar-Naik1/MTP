"""Fault Injection Controls (SRS §3.2.4, §9): per-drone or swarm-wide GPS
loss, communications loss, and battery failure injection.
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from contracts.gui_orchestration import FaultSeverity, FaultType, InjectFault

SYSID_ROLE = Qt.UserRole + 1

FAULT_LABELS = {
    FaultType.GPS_LOSS: "GPS Loss",
    FaultType.COMMS_LOSS: "Communications Loss",
    FaultType.BATTERY_FAIL: "Battery Failure",
}


class FaultInjectionPanel(QWidget):
    """Targets one or more active drones (SYSIDs) with an injected fault."""

    fault_requested = Signal(object)  # InjectFault
    status_message = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.drone_list = QListWidget()
        self.drone_list.setSelectionMode(QAbstractItemView.NoSelection)

        self.select_all_btn = QPushButton("Select All")
        self.select_none_btn = QPushButton("Select None")
        self.select_all_btn.clicked.connect(lambda: self._set_all_checked(True))
        self.select_none_btn.clicked.connect(lambda: self._set_all_checked(False))

        select_row = QHBoxLayout()
        select_row.addWidget(self.select_all_btn)
        select_row.addWidget(self.select_none_btn)

        target_group = QGroupBox("Target Drones")
        target_layout = QVBoxLayout(target_group)
        target_layout.addWidget(self.drone_list)
        target_layout.addLayout(select_row)

        self.fault_combo = QComboBox()
        for fault_type, label in FAULT_LABELS.items():
            self.fault_combo.addItem(label, fault_type)

        self.severity_combo = QComboBox()
        for severity in FaultSeverity:
            self.severity_combo.addItem(severity.value.capitalize(), severity)

        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(0, 3600)
        self.duration_spin.setSuffix(" s")
        self.duration_spin.setSpecialValueText("Indefinite")
        self.duration_spin.setValue(0)

        form = QFormLayout()
        form.addRow("Fault type", self.fault_combo)
        form.addRow("Severity", self.severity_combo)
        form.addRow("Duration (0 = until reset)", self.duration_spin)

        self.inject_btn = QPushButton("Inject Fault")
        self.inject_btn.setStyleSheet("font-weight: bold; color: #e74c3c;")
        self.inject_btn.clicked.connect(self._on_inject)

        fault_group = QGroupBox("Fault Injection")
        fault_layout = QVBoxLayout(fault_group)
        fault_layout.addLayout(form)
        fault_layout.addWidget(self.inject_btn)

        layout = QVBoxLayout(self)
        layout.addWidget(target_group, stretch=1)
        layout.addWidget(fault_group)

    def update_active_sysids(self, sysids: list[int]) -> None:
        checked = {
            self.drone_list.item(i).data(SYSID_ROLE)
            for i in range(self.drone_list.count())
            if self.drone_list.item(i).checkState() == Qt.Checked
        }
        self.drone_list.clear()
        for sysid in sorted(sysids):
            item = QListWidgetItem(f"SYSID {sysid}")
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Checked if sysid in checked else Qt.Unchecked)
            item.setData(SYSID_ROLE, sysid)
            self.drone_list.addItem(item)

    def _set_all_checked(self, checked: bool) -> None:
        state = Qt.Checked if checked else Qt.Unchecked
        for i in range(self.drone_list.count()):
            self.drone_list.item(i).setCheckState(state)

    def _checked_sysids(self) -> list[int]:
        return [
            self.drone_list.item(i).data(SYSID_ROLE)
            for i in range(self.drone_list.count())
            if self.drone_list.item(i).checkState() == Qt.Checked
        ]

    def _on_inject(self) -> None:
        sysids = self._checked_sysids()
        if not sysids:
            QMessageBox.information(self, "No target selected", "Check at least one active drone to target.")
            return
        command = InjectFault(
            sysids=sysids,
            fault_type=self.fault_combo.currentData(),
            severity=self.severity_combo.currentData(),
            duration_s=self.duration_spin.value() or None,
        )
        # Announce the injection first: handling the fault may itself post a
        # status message (a fall-back decision, say), and that outcome is the
        # more useful thing to leave on screen.
        label = FAULT_LABELS[command.fault_type]
        self.status_message.emit(f"Injected '{label}' on SYSID(s): {', '.join(map(str, sysids))}.")
        self.fault_requested.emit(command)
