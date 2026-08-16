from pathlib import Path

from PySide6.QtWidgets import QApplication

from engine.drone_runtime import (
    DronePlanCommand,
    build_mavlink_plan,
    create_drone_state,
    load_plan_from_file,
    route_points_for_drone,
)
from gui.map_window import DroneMapWindow


def test_load_plan_from_file_parses_commands(tmp_path: Path):
    plan = tmp_path / "drone_plan.txt"
    plan.write_text(
        "TAKEOFF\nMOVE 0.001 0.002 10\nLAND\n",
        encoding="utf-8",
    )

    commands = load_plan_from_file(str(plan))

    assert len(commands) == 3
    assert commands[0].command_type == "TAKEOFF"
    assert commands[1].command_type == "MOVE"
    assert commands[2].command_type == "LAND"
    assert commands[1].params["lat"] == 0.001
    assert commands[1].params["lon"] == 0.002
    assert commands[1].params["alt"] == 10


def test_drone_command_basic_fields():
    cmd = DronePlanCommand("MAVLINK", "TAKEOFF", {"alt": 20})

    assert cmd.protocol == "MAVLINK"
    assert cmd.command_type == "TAKEOFF"
    assert cmd.params["alt"] == 20


def test_build_mavlink_plan_generates_mavlink_commands():
    commands = [
        DronePlanCommand("MAVLINK", "TAKEOFF", {"alt": 10}),
        DronePlanCommand("MAVLINK", "MOVE", {"lat": 12.973, "lon": 77.596, "alt": 15}),
        DronePlanCommand("MAVLINK", "LAND", {}),
    ]

    mavlink_commands = build_mavlink_plan(commands)

    assert len(mavlink_commands) == 3
    assert "MAV_CMD_NAV_TAKEOFF" in mavlink_commands[0]
    assert "MAV_CMD_NAV_WAYPOINT" in mavlink_commands[1]
    assert "MAV_CMD_NAV_LAND" in mavlink_commands[2]


def test_route_points_include_start_and_waypoints():
    drone = {
        "start_lat": 12.9716,
        "start_lon": 77.5946,
        "end_lat": 12.9750,
        "end_lon": 77.6000,
        "plan": [
            DronePlanCommand("MAVLINK", "MOVE", {"lat": 12.9730, "lon": 77.5960, "alt": 15}),
        ],
    }

    points = route_points_for_drone(drone)

    assert points[0] == (12.9716, 77.5946)
    assert points[1] == (12.9730, 77.5960)
    assert points[2] == (12.9750, 77.6000)


def test_map_html_uses_local_leaflet_bundle_for_offline_map_rendering():
    map_html_path = Path(__file__).resolve().parents[1] / "resources" / "map.html"
    content = map_html_path.read_text(encoding="utf-8")

    assert "resources/leaflet/leaflet.js" in content
    assert "resources/leaflet/leaflet.css" in content


def test_play_all_prepares_threads_for_each_drone():
    app = QApplication.instance() or QApplication([])
    window = DroneMapWindow()
    drone = create_drone_state(
        drone_id="d2",
        name="Alpha",
        features=["mapping"],
        protocol="MAVLINK",
        start_lat=12.9716,
        start_lon=77.5946,
        end_lat=12.9750,
        end_lon=77.6000,
    )
    drone.plan = [
        DronePlanCommand("MAVLINK", "TAKEOFF", {"alt": 15}),
        DronePlanCommand("MAVLINK", "MOVE", {"lat": 12.9730, "lon": 77.5960, "alt": 15}),
        DronePlanCommand("MAVLINK", "LAND", {}),
    ]
    window.drones[drone.drone_id] = drone

    window._ensure_drone_threads_for_play()

    assert drone.drone_id in window.orchestrator.all_threads()
    app.quit()
