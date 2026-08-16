"""Create/edit dialog for a drone configuration profile (SRS §3.2.1)."""
from __future__ import annotations

from typing import Optional

from PySide6.QtWidgets import (
    QCheckBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLineEdit,
    QSpinBox,
    QVBoxLayout,
)

from contracts.gui_orchestration import DroneConfig, SensorType


class DroneConfigDialog(QDialog):
    """Modal form for creating or editing a single drone profile."""

    def __init__(self, drone: Optional[DroneConfig] = None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Edit Drone Profile" if drone else "New Drone Profile")
        self.setMinimumWidth(360)
        self._original_name = drone.name if drone else None

        self.name_edit = QLineEdit(drone.name if drone else "")
        self.sysid_spin = QSpinBox()
        self.sysid_spin.setRange(1, 255)
        self.sysid_spin.setValue(drone.sysid if drone else 1)

        self.mass_spin = QDoubleSpinBox()
        self.mass_spin.setRange(0.01, 10_000.0)
        self.mass_spin.setDecimals(2)
        self.mass_spin.setSuffix(" kg")
        self.mass_spin.setValue(drone.mass_kg if drone else 1.5)

        self.max_velocity_spin = QDoubleSpinBox()
        self.max_velocity_spin.setRange(0.1, 200.0)
        self.max_velocity_spin.setDecimals(1)
        self.max_velocity_spin.setSuffix(" m/s")
        self.max_velocity_spin.setValue(drone.max_velocity_mps if drone else 15.0)

        self.battery_spin = QDoubleSpinBox()
        self.battery_spin.setRange(1.0, 100_000.0)
        self.battery_spin.setDecimals(0)
        self.battery_spin.setSuffix(" mAh")
        self.battery_spin.setValue(drone.battery_capacity_mah if drone else 5200.0)

        self.altitude_spin = QDoubleSpinBox()
        self.altitude_spin.setRange(1.0, 10_000.0)
        self.altitude_spin.setDecimals(1)
        self.altitude_spin.setSuffix(" m")
        self.altitude_spin.setValue(drone.cruise_altitude_m if drone else 50.0)

        form = QFormLayout()
        form.addRow("Name", self.name_edit)
        form.addRow("MAVLink SYSID", self.sysid_spin)
        form.addRow("Mass", self.mass_spin)
        form.addRow("Max velocity", self.max_velocity_spin)
        form.addRow("Cruise altitude", self.altitude_spin)
        form.addRow("Battery capacity", self.battery_spin)

        sensor_group = QGroupBox("Sensor set")
        sensor_layout = QVBoxLayout(sensor_group)
        selected = set(drone.sensors) if drone else {SensorType.GPS, SensorType.IMU}
        self.sensor_checks: dict[SensorType, QCheckBox] = {}
        for sensor in SensorType:
            box = QCheckBox(sensor.value.upper())
            box.setChecked(sensor in selected)
            sensor_layout.addWidget(box)
            self.sensor_checks[sensor] = box

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)

        layout = QVBoxLayout(self)
        layout.addLayout(form)
        layout.addWidget(sensor_group)
        layout.addWidget(buttons)

    def drone_config(self) -> DroneConfig:
        sensors = [sensor for sensor, box in self.sensor_checks.items() if box.isChecked()]
        return DroneConfig(
            name=self.name_edit.text().strip() or f"drone-{self.sysid_spin.value()}",
            sysid=self.sysid_spin.value(),
            mass_kg=self.mass_spin.value(),
            max_velocity_mps=self.max_velocity_spin.value(),
            battery_capacity_mah=self.battery_spin.value(),
            cruise_altitude_m=self.altitude_spin.value(),
            sensors=sensors,
        )

    @property
    def original_name(self) -> Optional[str]:
        return self._original_name
