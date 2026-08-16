"""Map Viewer panel (SRS §3.2.2): QtLocation/QML map with offline tile
support, bounding-box region selection, click-to-set start/destination
markers, live drone position rendering and flight-path overlays.
"""
from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import QEvent, QSizeF, QTimer, QUrl, Signal
from PySide6.QtQuickWidgets import QQuickWidget
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from contracts.gui_orchestration import DroneTelemetry
from gui.map_models import DroneMarkerModel, PathModel, PointMarkerModel

QML_PATH = Path(__file__).resolve().parent.parent / "qml" / "Map.qml"
TILE_CACHE_DIR = Path(__file__).resolve().parent.parent / "data" / "tiles"


class MapViewer(QWidget):
    """Central map panel. Emits `point_picked` when the user clicks the map
    while in "Set Start" or "Set Destination" mode."""

    point_picked = Signal(str, float, float)  # "start" | "destination", lat, lon
    status_message = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        TILE_CACHE_DIR.mkdir(parents=True, exist_ok=True)

        self.drone_model = DroneMarkerModel(self)
        self.marker_model = PointMarkerModel(self)
        self.path_model = PathModel(self)

        self.quick_widget = QQuickWidget()
        self.quick_widget.setResizeMode(QQuickWidget.SizeRootObjectToView)
        self.quick_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.quick_widget.setMinimumHeight(200)
        # Growing the widget (maximising the window, dragging the splitter) can
        # leave the newly exposed band showing the previous frame, which reads as
        # a dark seam across the map. Repaint the whole surface on every resize.
        self.quick_widget.installEventFilter(self)
        context = self.quick_widget.rootContext()
        context.setContextProperty("droneModel", self.drone_model)
        context.setContextProperty("markerModel", self.marker_model)
        context.setContextProperty("pathModel", self.path_model)
        context.setContextProperty("tileCacheDir", str(TILE_CACHE_DIR))
        context.setContextProperty("offlineMode", False)
        self.quick_widget.setSource(QUrl.fromLocalFile(str(QML_PATH)))

        self._root_object = self.quick_widget.rootObject()
        if self._root_object is not None:
            self._root_object.mapClicked.connect(self._on_map_clicked)

        # ---- Toolbar: interaction mode ----
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["View", "Set Start Point", "Set Destination Point"])

        self.offline_check = QCheckBox("Offline mode (use cached tiles only)")
        self.offline_check.toggled.connect(self._on_offline_toggled)

        self.zoom_in_btn = QPushButton("Zoom In")
        self.zoom_out_btn = QPushButton("Zoom Out")
        self.zoom_in_btn.clicked.connect(self._on_zoom_in)
        self.zoom_out_btn.clicked.connect(self._on_zoom_out)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Click mode:"))
        mode_row.addWidget(self.mode_combo)
        mode_row.addStretch(1)
        mode_row.addWidget(self.zoom_out_btn)
        mode_row.addWidget(self.zoom_in_btn)
        mode_row.addWidget(self.offline_check)

        # ---- Bounding box region selection (§3.2.2). Built here because the
        # handlers drive this map, but handed to the main window via
        # `region_panel()` so it can live in a full-width strip under every
        # panel instead of stealing height from the map. ----
        self.min_lat = self._make_spin(-90, 90, 8.0)
        self.max_lat = self._make_spin(-90, 90, 37.0)
        self.min_lon = self._make_spin(-180, 180, 68.0)
        self.max_lon = self._make_spin(-180, 180, 97.0)

        self.set_region_btn = QPushButton("Set Region")
        self.cache_region_btn = QPushButton("Download / Cache This Region")
        self.set_region_btn.clicked.connect(self._on_set_region)
        self.cache_region_btn.clicked.connect(self._on_cache_region)

        # Untitled: the dock that hosts this panel carries the caption.
        self.region_group = QGroupBox()
        self.region_group.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        region_layout = QHBoxLayout(self.region_group)
        for caption, spin in (
            ("Min Lat", self.min_lat),
            ("Max Lat", self.max_lat),
            ("Min Lon", self.min_lon),
            ("Max Lon", self.max_lon),
        ):
            region_layout.addWidget(QLabel(caption))
            region_layout.addWidget(spin)
        region_layout.addStretch(1)
        region_layout.addWidget(self.set_region_btn)
        region_layout.addWidget(self.cache_region_btn)

        # The map gets the whole central area; the region controls are docked
        # full-width by the main window.
        layout = QVBoxLayout(self)
        layout.addLayout(mode_row)
        layout.addWidget(self.quick_widget, stretch=1)

    def region_panel(self) -> QGroupBox:
        """The Working Area group box, for the main window to dock full-width."""
        return self.region_group

        self._cache_timer = QTimer(self)
        self._cache_timer.timeout.connect(self._cache_next_tile_point)
        self._cache_queue: list[tuple[float, float]] = []

    def eventFilter(self, watched, event):
        """Force a full repaint of the QML surface whenever it is resized, so a
        widened/heightened map cannot keep showing a stale frame in the band it
        just grew into."""
        if watched is self.quick_widget and event.type() == QEvent.Resize:
            root = self._root_object
            if root is not None:
                root.setSize(QSizeF(self.quick_widget.size()))
            self.quick_widget.update()
        return super().eventFilter(watched, event)

    @staticmethod
    def _make_spin(lo: float, hi: float, default: float) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(lo, hi)
        spin.setDecimals(4)
        spin.setValue(default)
        spin.setMaximumWidth(110)
        return spin

    # ---- Interaction ----

    def _on_map_clicked(self, lat: float, lon: float) -> None:
        mode = self.mode_combo.currentText()
        if mode == "Set Start Point":
            self.marker_model.set_point("start", lat, lon)
            self.point_picked.emit("start", lat, lon)
        elif mode == "Set Destination Point":
            self.marker_model.set_point("destination", lat, lon)
            self.point_picked.emit("destination", lat, lon)

    def current_mode(self) -> str:
        return self.mode_combo.currentText()

    def _on_zoom_in(self) -> None:
        if self._root_object is not None:
            self._root_object.zoomIn()

    def _on_zoom_out(self) -> None:
        if self._root_object is not None:
            self._root_object.zoomOut()

    def _on_offline_toggled(self, checked: bool) -> None:
        self.quick_widget.rootContext().setContextProperty("offlineMode", checked)
        self.status_message.emit(
            "Offline mode enabled - rendering from cached tiles only."
            if checked
            else "Offline mode disabled - live tile fetch allowed."
        )

    # ---- Region ----

    def _on_set_region(self) -> None:
        min_lat, max_lat = self.min_lat.value(), self.max_lat.value()
        min_lon, max_lon = self.min_lon.value(), self.max_lon.value()
        if self._root_object is not None:
            self._root_object.setRegion(min_lat, min_lon, max_lat, max_lon)
        self.status_message.emit(
            f"Region set to [{min_lat:.3f}, {min_lon:.3f}] - [{max_lat:.3f}, {max_lon:.3f}]."
        )

    def _on_cache_region(self) -> None:
        """Warms the local OSM tile disk cache for the selected bounding box
        by panning the map across a grid of points (SRS §3.2.2 - offline tile
        support). Requires a live internet connection at cache time; once
        cached, the region renders with no network dependency.
        """
        min_lat, max_lat = self.min_lat.value(), self.max_lat.value()
        min_lon, max_lon = self.min_lon.value(), self.max_lon.value()
        steps = 4
        self._cache_queue = [
            (
                min_lat + (max_lat - min_lat) * row / (steps - 1),
                min_lon + (max_lon - min_lon) * col / (steps - 1),
            )
            for row in range(steps)
            for col in range(steps)
        ]
        self.status_message.emit(f"Caching {len(self._cache_queue)} region tiles for offline use...")
        self._cache_timer.start(400)

    def _cache_next_tile_point(self) -> None:
        if not self._cache_queue or self._root_object is None:
            self._cache_timer.stop()
            self.status_message.emit("Region caching complete.")
            return
        lat, lon = self._cache_queue.pop(0)
        self._root_object.panTo(lat, lon, 12)

    # ---- Live telemetry ----

    def update_drones(self, drones: list[DroneTelemetry]) -> None:
        self.drone_model.set_drones(drones)
        for drone in drones:
            self.path_model.append_point(drone.sysid, drone.lat, drone.lon)

    def clear_markers(self) -> None:
        self.marker_model.clear()

    def clear_paths(self) -> None:
        self.path_model.clear()
