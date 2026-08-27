"""Drone Management Panel (SRS §3.2.1).

Create/edit/save/delete drone configuration profiles; select a saved swarm
preset; "Emulate" locks configuration inputs and hands the active swarm
selection off to Module 2; "Stop/Reset" tears the run down and unlocks
configuration inputs again.
"""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QGroupBox,
    QHBoxLayout,
    QInputDialog,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from contracts.gui_orchestration import DroneConfig, SwarmPreset
from gui.drone_config_dialog import DroneConfigDialog
from services.storage import ProfileStore

SYSID_ROLE = Qt.UserRole + 1


class DroneManagementPanel(QWidget):
    emulate_requested = Signal(list)   # list[DroneConfig]
    stop_requested = Signal()
    status_message = Signal(str)

    def __init__(self, store: ProfileStore, parent=None):
        super().__init__(parent)
        self.store = store
        self._profiles: dict[str, DroneConfig] = {}
        self._running = False

        self.profile_list = QListWidget()
        self.profile_list.setSelectionMode(QAbstractItemView.SingleSelection)
        # Clicking the check indicator toggles the box but does not make the row
        # current, which would leave Edit/Delete with no target. Treat a click
        # anywhere on the row as selecting it.
        self.profile_list.itemClicked.connect(self.profile_list.setCurrentItem)

        self.new_btn = QPushButton("New")
        self.edit_btn = QPushButton("Edit")
        self.delete_btn = QPushButton("Delete")
        self.new_btn.clicked.connect(self._on_new)
        self.edit_btn.clicked.connect(self._on_edit)
        self.delete_btn.clicked.connect(self._on_delete)

        crud_row = QHBoxLayout()
        crud_row.addWidget(self.new_btn)
        crud_row.addWidget(self.edit_btn)
        crud_row.addWidget(self.delete_btn)

        profile_group = QGroupBox("Drone Configuration Profiles")
        profile_layout = QVBoxLayout(profile_group)
        profile_layout.addWidget(QLabel("Check drones to include in the active swarm:"))
        profile_layout.addWidget(self.profile_list)
        profile_layout.addLayout(crud_row)

        self.preset_combo = QComboBox()
        self.load_preset_btn = QPushButton("Load")
        self.save_preset_btn = QPushButton("Save Current As...")
        self.load_preset_btn.clicked.connect(self._on_load_preset)
        self.save_preset_btn.clicked.connect(self._on_save_preset)

        preset_row = QHBoxLayout()
        preset_row.addWidget(self.preset_combo, stretch=1)
        preset_row.addWidget(self.load_preset_btn)

        preset_group = QGroupBox("Swarm Selector (Preset)")
        preset_layout = QVBoxLayout(preset_group)
        preset_layout.addLayout(preset_row)
        preset_layout.addWidget(self.save_preset_btn)

        self.emulate_btn = QPushButton("Emulate")
        self.emulate_btn.setStyleSheet("font-weight: bold;")
        self.stop_btn = QPushButton("Stop / Reset")
        self.stop_btn.setEnabled(False)
        self.emulate_btn.clicked.connect(self._on_emulate)
        self.stop_btn.clicked.connect(self._on_stop)

        run_row = QHBoxLayout()
        run_row.addWidget(self.emulate_btn)
        run_row.addWidget(self.stop_btn)

        layout = QVBoxLayout(self)
        layout.addWidget(profile_group, stretch=1)
        layout.addWidget(preset_group)
        layout.addLayout(run_row)

        self.reload_profiles()
        self.reload_presets()

    # ---- Data loading ----

    def reload_profiles(self) -> None:
        # Rebuilding the rows would drop the swarm selection, so carry the
        # checked drones across the refresh (deleting one profile must not
        # silently deselect the rest).
        checked = {
            self.profile_list.item(i).data(SYSID_ROLE)
            for i in range(self.profile_list.count())
            if self.profile_list.item(i).checkState() == Qt.Checked
        }
        self.profile_list.clear()
        self._profiles.clear()
        for drone in self.store.list_profiles():
            self._profiles[drone.name] = drone
            self._add_list_item(drone, checked=drone.name in checked)

    def _add_list_item(self, drone: DroneConfig, checked: bool = False) -> None:
        item = QListWidgetItem(f"{drone.name}  (SYSID {drone.sysid})")
        item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
        item.setCheckState(Qt.Checked if checked else Qt.Unchecked)
        item.setData(SYSID_ROLE, drone.name)
        self.profile_list.addItem(item)

    def reload_presets(self) -> None:
        self.preset_combo.clear()
        for preset in self.store.list_presets():
            self.preset_combo.addItem(preset.name, preset)

    # ---- CRUD ----

    def _on_new(self) -> None:
        dialog = DroneConfigDialog(parent=self)
        if dialog.exec():
            drone = dialog.drone_config()
            if drone.name in self._profiles:
                QMessageBox.warning(self, "Duplicate name", f"A profile named '{drone.name}' already exists.")
                return
            self.store.save_profile(drone)
            self.reload_profiles()
            self.status_message.emit(f"Saved profile '{drone.name}'.")

    def _selected_profile_name(self, action: str) -> str | None:
        """Name of the profile Edit/Delete should act on, or None (after telling
        the user why) when the list has no current row."""
        item = self.profile_list.currentItem()
        if item is None:
            selected = self.profile_list.selectedItems()
            item = selected[0] if selected else None
        if item is None:
            QMessageBox.information(
                self,
                "No profile selected",
                f"Select a drone profile in the list first, then press {action}.\n\n"
                "Note: ticking a checkbox only marks a drone for the active swarm.",
            )
            return None
        return item.data(SYSID_ROLE)

    def _on_edit(self) -> None:
        name = self._selected_profile_name("Edit")
        if name is None:
            return
        drone = self._profiles.get(name)
        if drone is None:
            return
        dialog = DroneConfigDialog(drone=drone, parent=self)
        if dialog.exec():
            updated = dialog.drone_config()
            self.store.save_profile(updated, previous_name=dialog.original_name)
            self.reload_profiles()
            self.status_message.emit(f"Updated profile '{updated.name}'.")

    def _on_delete(self) -> None:
        name = self._selected_profile_name("Delete")
        if name is None:
            return
        if QMessageBox.question(self, "Delete profile", f"Delete drone profile '{name}'?") != QMessageBox.Yes:
            return
        self.store.delete_profile(name)
        self.reload_profiles()
        self.status_message.emit(f"Deleted profile '{name}'.")

    # ---- Presets ----

    def _checked_drones(self) -> list[DroneConfig]:
        drones = []
        for i in range(self.profile_list.count()):
            item = self.profile_list.item(i)
            if item.checkState() == Qt.Checked:
                drones.append(self._profiles[item.data(SYSID_ROLE)])
        return drones

    def _on_load_preset(self) -> None:
        preset: SwarmPreset = self.preset_combo.currentData()
        if preset is None:
            return
        preset_names = {d.name for d in preset.drones}
        for i in range(self.profile_list.count()):
            item = self.profile_list.item(i)
            item.setCheckState(Qt.Checked if item.data(SYSID_ROLE) in preset_names else Qt.Unchecked)
        self.status_message.emit(f"Loaded swarm preset '{preset.name}'.")

    def _on_save_preset(self) -> None:
        drones = self._checked_drones()
        if not drones:
            QMessageBox.information(self, "No drones selected", "Check at least one drone profile first.")
            return
        name, ok = QInputDialog.getText(self, "Save Swarm Preset", "Preset name (e.g. 'Alpha Swarm - 5 Units'):")
        if not ok or not name.strip():
            return
        self.store.save_preset(SwarmPreset(name=name.strip(), drones=drones))
        self.reload_presets()
        self.status_message.emit(f"Saved swarm preset '{name.strip()}'.")

    # ---- Emulate / Stop ----

    def _on_emulate(self) -> None:
        drones = self._checked_drones()
        if not drones:
            QMessageBox.information(self, "No drones selected", "Check at least one drone profile to emulate.")
            return
        self.set_running(True)
        self.emulate_requested.emit(drones)

    def _on_stop(self) -> None:
        self.stop_requested.emit()
        self.set_running(False)

    def set_running(self, running: bool) -> None:
        self._running = running
        self.emulate_btn.setEnabled(not running)
        self.stop_btn.setEnabled(running)
        for widget in (
            self.profile_list,
            self.new_btn,
            self.edit_btn,
            self.delete_btn,
            self.preset_combo,
            self.load_preset_btn,
            self.save_preset_btn,
        ):
            widget.setEnabled(not running)

    def checked_sysids(self) -> list[int]:
        return [d.sysid for d in self._checked_drones()]

    def checked_drones(self) -> list[DroneConfig]:
        return self._checked_drones()
