"""Qt list models exposing drone telemetry and map annotations to Map.qml
(SRS §3.2.2, §3.2.3)."""
from __future__ import annotations

from PySide6.QtCore import QAbstractListModel, QModelIndex, Qt

from contracts.gui_orchestration import DroneTelemetry


class DroneMarkerModel(QAbstractListModel):
    """Live drone positions rendered as moving map markers."""

    SYSID = Qt.UserRole + 1
    LAT = Qt.UserRole + 2
    LON = Qt.UserRole + 3
    HEADING = Qt.UserRole + 4
    STATUS = Qt.UserRole + 5
    HAS_FAULT = Qt.UserRole + 6

    _ROLE_NAMES = {
        SYSID: b"sysid",
        LAT: b"lat",
        LON: b"lon",
        HEADING: b"heading",
        STATUS: b"status",
        HAS_FAULT: b"hasFault",
    }

    def __init__(self, parent=None):
        super().__init__(parent)
        self._drones: list[DroneTelemetry] = []

    def roleNames(self):
        return dict(self._ROLE_NAMES)

    def rowCount(self, parent=QModelIndex()) -> int:
        return len(self._drones)

    def data(self, index, role):
        if not index.isValid() or not (0 <= index.row() < len(self._drones)):
            return None
        drone = self._drones[index.row()]
        if role == self.SYSID:
            return drone.sysid
        if role == self.LAT:
            return drone.lat
        if role == self.LON:
            return drone.lon
        if role == self.HEADING:
            return drone.heading_deg
        if role == self.STATUS:
            return drone.status.value
        if role == self.HAS_FAULT:
            return bool(drone.active_faults)
        return None

    def set_drones(self, drones: list[DroneTelemetry]) -> None:
        self.beginResetModel()
        self._drones = list(drones)
        self.endResetModel()


class PointMarkerModel(QAbstractListModel):
    """Click-to-set start (green) / destination (red) markers."""

    ROLE_KIND = Qt.UserRole + 1
    LAT = Qt.UserRole + 2
    LON = Qt.UserRole + 3

    _ROLE_NAMES = {ROLE_KIND: b"role", LAT: b"lat", LON: b"lon"}

    def __init__(self, parent=None):
        super().__init__(parent)
        self._points: list[tuple[str, float, float]] = []

    def roleNames(self):
        return dict(self._ROLE_NAMES)

    def rowCount(self, parent=QModelIndex()) -> int:
        return len(self._points)

    def data(self, index, role):
        if not index.isValid() or not (0 <= index.row() < len(self._points)):
            return None
        kind, lat, lon = self._points[index.row()]
        if role == self.ROLE_KIND:
            return kind
        if role == self.LAT:
            return lat
        if role == self.LON:
            return lon
        return None

    def set_point(self, kind: str, lat: float, lon: float) -> None:
        self.beginResetModel()
        self._points = [p for p in self._points if p[0] != kind] + [(kind, lat, lon)]
        self.endResetModel()

    def clear(self) -> None:
        self.beginResetModel()
        self._points = []
        self.endResetModel()


class PathModel(QAbstractListModel):
    """Per-drone flight-path vector overlays (a trailing polyline of recent positions)."""

    SYSID = Qt.UserRole + 1
    POINTS = Qt.UserRole + 2

    _ROLE_NAMES = {SYSID: b"sysid", POINTS: b"points"}

    def __init__(self, parent=None, max_points: int = 200):
        super().__init__(parent)
        self._paths: dict[int, list[tuple[float, float]]] = {}
        self._order: list[int] = []
        self._max_points = max_points

    def roleNames(self):
        return dict(self._ROLE_NAMES)

    def rowCount(self, parent=QModelIndex()) -> int:
        return len(self._order)

    def data(self, index, role):
        if not index.isValid() or not (0 <= index.row() < len(self._order)):
            return None
        sysid = self._order[index.row()]
        if role == self.SYSID:
            return sysid
        if role == self.POINTS:
            return [{"latitude": lat, "longitude": lon} for lat, lon in self._paths[sysid]]
        return None

    def append_point(self, sysid: int, lat: float, lon: float) -> None:
        self.beginResetModel()
        if sysid not in self._paths:
            self._paths[sysid] = []
            self._order.append(sysid)
        points = self._paths[sysid]
        points.append((lat, lon))
        if len(points) > self._max_points:
            del points[: len(points) - self._max_points]
        self.endResetModel()

    def clear(self) -> None:
        self.beginResetModel()
        self._paths = {}
        self._order = []
        self.endResetModel()
