"""Demo: `python -m engine` - spawn N drone threads and fly one mission.

Shows each drone starting as an independent node, holding its own state, and
completing the source-to-destination task on its own thread.
"""
from __future__ import annotations

import argparse
import sys
import time

from engine.drone_thread import Command, CommandType, Position
from engine.fleet import DroneFleet

SOURCE = Position(lat=17.660, lon=75.900, alt_m=0.0)
DESTINATION = Position(lat=17.670, lon=75.912, alt_m=0.0)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Independent drone thread demo")
    parser.add_argument("-n", "--drones", type=int, default=3, help="how many drones")
    parser.add_argument("--time-scale", type=float, default=20.0,
                        help="simulated seconds per real second")
    parser.add_argument("--timeout", type=float, default=60.0, help="give up after N seconds")
    args = parser.parse_args(argv)

    fleet = DroneFleet()
    # Each drone gets its own speed and battery, to make independence visible.
    for i in range(1, args.drones + 1):
        fleet.spawn(
            drone_id=i,
            name=f"d{i}",
            cruise_speed_mps=12.0 + 3.0 * i,
            battery_capacity_mah=5200.0,
            start=SOURCE,
            time_scale=args.time_scale,
        )
        time.sleep(0.05)  # so the startup lines print in order

    time.sleep(0.3)
    print()
    print(f"Fleet: {fleet.alive_count()} live threads")
    print(f"Mission: ({SOURCE.lat}, {SOURCE.lon}) -> ({DESTINATION.lat}, {DESTINATION.lon})")
    print()

    fleet.start_mission(DESTINATION)

    deadline = time.monotonic() + args.timeout
    last_print = 0.0
    while not fleet.all_finished() and time.monotonic() < deadline:
        time.sleep(0.25)
        now = time.monotonic()
        if now - last_print >= 1.0:
            last_print = now
            for drone_id, snap in sorted(fleet.latest().items()):
                print(
                    f"  drone {drone_id}: {snap.mission_state.value:<9} "
                    f"pos=({snap.position.lat:.5f}, {snap.position.lon:.5f}) "
                    f"alt={snap.position.alt_m:5.1f}m "
                    f"spd={snap.velocity.ground_speed_mps:4.1f}m/s "
                    f"batt={snap.battery:5.1f}% {snap.health_status.value}"
                )
            print()

    print("--- final state (each drone independent) ---")
    for drone_id, snap in sorted(fleet.latest().items()):
        print(
            f"  drone {drone_id} ({snap.name}): {snap.mission_state.value} "
            f"battery={snap.battery:.1f}% elapsed={snap.elapsed_s:.1f}s "
            f"thread={'alive' if fleet.get(drone_id).is_alive() else 'stopped'}"
        )

    print()
    fleet.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
