from __future__ import annotations

from dataclasses import dataclass, field
from threading import Lock
from typing import Any, Dict, List, Optional, Tuple


@dataclass
class DronePlanCommand:
    protocol: str
    command_type: str
    params: Dict[str, Any] = field(default_factory=dict)


@dataclass
class DroneState:
    drone_id: str
    name: str
    features: List[str]
    protocol: str
    start_lat: float
    start_lon: float
    end_lat: float
    end_lon: float
    current_lat: float
    current_lon: float
    current_alt: float
    status: str = "IDLE"
    plan: List[DronePlanCommand] = field(default_factory=list)


class DroneMemory:
    def __init__(self):
        self._lock = Lock()
        self._items: Dict[str, DroneState] = {}

    def add_or_update(self, drone: DroneState) -> None:
        with self._lock:
            self._items[drone.drone_id] = drone

    def get(self, drone_id: str) -> Optional[DroneState]:
        with self._lock:
            return self._items.get(drone_id)

    def all(self) -> List[DroneState]:
        with self._lock:
            return list(self._items.values())


def load_plan_from_file(path: str) -> List[DronePlanCommand]:
    commands: List[DronePlanCommand] = []

    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            row = line.strip()
            if not row:
                continue

            parts = row.split()
            cmd = parts[0].upper()
            params: Dict[str, Any] = {}

            if cmd == "MOVE":
                if len(parts) >= 4:
                    try:
                        params = {
                            "lat": float(parts[1]),
                            "lon": float(parts[2]),
                            "alt": float(parts[3]),
                        }
                    except ValueError:
                        params = {"raw": " ".join(parts[1:])}
            elif cmd == "TAKEOFF":
                if len(parts) >= 2:
                    try:
                        params = {"alt": float(parts[1])}
                    except ValueError:
                        params = {"raw": " ".join(parts[1:])}
            elif cmd == "LAND":
                params = {}
            else:
                params = {"raw": " ".join(parts[1:])}

            commands.append(DronePlanCommand(protocol="MAVLINK", command_type=cmd, params=params))

    return commands


def create_drone_state(
    drone_id: str,
    name: str,
    features: List[str],
    protocol: str,
    start_lat: float,
    start_lon: float,
    end_lat: float,
    end_lon: float,
) -> DroneState:
    return DroneState(
        drone_id=drone_id,
        name=name,
        features=features,
        protocol=protocol,
        start_lat=start_lat,
        start_lon=start_lon,
        end_lon=end_lon,
        end_lat=end_lat,
        current_lat=start_lat,
        current_lon=start_lon,
        current_alt=0.0,
    )


def build_default_mission(start_lat: float, start_lon: float, end_lat: float, end_lon: float, altitude: float = 15.0) -> List[DronePlanCommand]:
    return [
        DronePlanCommand("MAVLINK", "TAKEOFF", {"alt": altitude}),
        DronePlanCommand("MAVLINK", "MOVE", {"lat": end_lat, "lon": end_lon, "alt": altitude}),
        DronePlanCommand("MAVLINK", "LAND", {}),
    ]


def route_points_for_drone(drone: Dict[str, Any]) -> List[Tuple[float, float]]:
    points: List[Tuple[float, float]] = [(float(drone["start_lat"]), float(drone["start_lon"]))]

    for command in drone.get("plan", []):
        if command.command_type == "MOVE":
            lat = command.params.get("lat")
            lon = command.params.get("lon")
            if lat is not None and lon is not None:
                points.append((float(lat), float(lon)))

    points.append((float(drone["end_lat"]), float(drone["end_lon"])))
    return points


def build_mavlink_plan(commands: List[DronePlanCommand]) -> List[str]:
    mavlink_messages: List[str] = []
    for command in commands:
        cmd_type = command.command_type.upper()
        if cmd_type == "TAKEOFF":
            altitude = command.params.get("alt", 10)
            mavlink_messages.append(f"MAV_CMD_NAV_TAKEOFF alt={altitude}")
        elif cmd_type == "MOVE":
            lat = command.params.get("lat", 0.0)
            lon = command.params.get("lon", 0.0)
            altitude = command.params.get("alt", 0.0)
            mavlink_messages.append(f"MAV_CMD_NAV_WAYPOINT lat={lat} lon={lon} alt={altitude}")
        elif cmd_type == "LAND":
            mavlink_messages.append("MAV_CMD_NAV_LAND")
        else:
            mavlink_messages.append(f"MAV_CMD_CUSTOM_{cmd_type}")
    return mavlink_messages


def interpolate_route(start_lat: float, start_lon: float, end_lat: float, end_lon: float, steps: int = 10) -> List[Tuple[float, float]]:
    if steps <= 1:
        return [(end_lat, end_lon)]

    points: List[Tuple[float, float]] = []
    for i in range(steps + 1):
        ratio = i / steps
        lat = start_lat + (end_lat - start_lat) * ratio
        lon = start_lon + (end_lon - start_lon) * ratio
        points.append((lat, lon))
    return points


def telemetry_points_for_drone(drone: Dict[str, Any]) -> List[Tuple[float, float, float]]:
    points: List[Tuple[float, float, float]] = []
    current = (float(drone["start_lat"]), float(drone["start_lon"]), 0.0)
    points.append(current)

    for command in drone.get("plan", []):
        if command.command_type == "MOVE":
            lat = float(command.params.get("lat", current[0]))
            lon = float(command.params.get("lon", current[1]))
            altitude = float(command.params.get("alt", current[2]))
            points.append((lat, lon, altitude))
        elif command.command_type == "TAKEOFF":
            altitude = float(command.params.get("alt", current[2]))
            current = (current[0], current[1], altitude)
            points.append((current[0], current[1], altitude))
        elif command.command_type == "LAND":
            points.append((current[0], current[1], 0.0))
    points.append((float(drone["end_lat"]), float(drone["end_lon"]), 0.0))
    return points
