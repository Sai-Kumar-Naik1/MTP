"""Mission Planner panel: pick a named PDDL plan, arm the map for a
start/destination click, and show ENHSP's parsed action sequence once one
comes back.
"""
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from engine.pddl_planner import PlanStep
from services.plan_service import list_plans


class MissionPlannerPanel(QWidget):
    """Select a plan, request it, and read back its parsed step list."""

    plan_requested = Signal(str)  # plan name

    def __init__(self, parent=None):
        super().__init__(parent)

        self.plan_combo = QComboBox()
        self.refresh_btn = QPushButton("Refresh")
        self.plan_btn = QPushButton("Plan Mission")
        self.plan_btn.setStyleSheet("font-weight: bold;")
        self.refresh_btn.clicked.connect(self.reload_plans)
        self.plan_btn.clicked.connect(self._on_plan_clicked)

        pick_row = QHBoxLayout()
        pick_row.addWidget(self.plan_combo, stretch=1)
        pick_row.addWidget(self.refresh_btn)

        self.restricted_check = QCheckBox("Mark a restricted area (3rd click)")
        self.restricted_check.setToolTip(
            "Unchecked: the mission always flies direct.\n"
            "Checked: click a Start point, a Destination, then switch the map's "
            "Click mode to \"Set Restricted Area\" and click the area to avoid - "
            "the mission only detours around it if it actually sits in the way."
        )

        plan_group = QGroupBox("Named Plan (plans/<name>/)")
        plan_layout = QVBoxLayout(plan_group)
        plan_layout.addLayout(pick_row)
        plan_layout.addWidget(self.restricted_check)
        plan_layout.addWidget(self.plan_btn)

        self.status_label = QLabel("Idle.")
        self.status_label.setWordWrap(True)

        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setPlaceholderText(
            "Pick a plan, press \"Plan Mission\", then click a Start point and "
            "a Destination on the map. ENHSP's action sequence appears here."
        )

        layout = QVBoxLayout(self)
        layout.addWidget(plan_group)
        layout.addWidget(self.status_label)
        layout.addWidget(self.output, stretch=1)

        self.reload_plans()

    def reload_plans(self) -> None:
        current = self.plan_combo.currentText()
        self.plan_combo.clear()
        self.plan_combo.addItems(list_plans())
        index = self.plan_combo.findText(current)
        if index >= 0:
            self.plan_combo.setCurrentIndex(index)

    def _on_plan_clicked(self) -> None:
        name = self.plan_combo.currentText()
        if not name:
            self.set_status("No plans found under plans/ - add a domain.pddl + problem.pddl folder.")
            return
        self.plan_requested.emit(name)

    def mark_restricted_area(self) -> bool:
        return self.restricted_check.isChecked()

    def set_status(self, text: str) -> None:
        self.status_label.setText(text)

    def set_plan_steps(self, plan_name: str, steps: list[PlanStep]) -> None:
        lines = [f"Plan '{plan_name}' - {len(steps)} step(s):", ""]
        lines.extend(str(step) for step in sorted(steps, key=lambda s: s.index))
        self.output.setPlainText("\n".join(lines))

    def set_error(self, text: str) -> None:
        self.output.setPlainText(text)
