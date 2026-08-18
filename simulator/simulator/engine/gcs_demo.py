"""Demo: `python -m engine.gcs_demo` - GCS talking to drone threads over UDP.

Spawns N networked drones, then either runs a scripted sequence (`--script`)
or drops into a console where you can type commands:

    NAVIGATE 1          send drone 1 on the mission
    NAVIGATE all        send every drone
    HOLD 2 / RESUME 2   hold and release one drone
    GPS 3 off / on      inject and clear a GPS fault
    RTL 1               order a return to launch
    STATUS              print what the GCS has heard from each drone
    QUIT
"""
from __future__ import annotations

import argparse
import sys
import time

from engine.drone_link import DroneLink
from engine.drone_thread import Position
from engine.fleet import DroneFleet
from engine.gcs import GroundControlStation
from engine.protocol import CommandName

SOURCE = [17.660, 75.900, 0.0]
DESTINATION = [17.670, 75.912, 50.0]


def build(count: int, time_scale: float, verbose: bool = True):
    """Spawn a networked swarm and a GCS that knows how to reach it."""
    gcs = GroundControlStation(verbose=verbose)
    gcs.start()

    fleet = DroneFleet()
    for sysid in range(1, count + 1):
        link = DroneLink(sysid=sysid, gcs_address=gcs.address, verbose=verbose)
        fleet.spawn(
            drone_id=sysid,
            name=f"d{sysid}",
            cruise_speed_mps=12.0 + 3.0 * sysid,
            start=Position(lat=SOURCE[0], lon=SOURCE[1]),
            time_scale=time_scale,
            link=link,
        )
        time.sleep(0.05)
    gcs.register_many(range(1, count + 1))
    return gcs, fleet


def print_status(gcs: GroundControlStation) -> None:
    print("  --- GCS view (from telemetry alone) ---")
    for drone_id, record in sorted(gcs.all_drones().items()):
        t = record.telemetry
        if not t:
            print(f"    drone {drone_id}: no telemetry yet  link={'up' if record.link_alive else 'DOWN'}")
            continue
        pos = t["position"]
        print(
            f"    drone {drone_id}: {t['mission_state']:<9} "
            f"pos=({pos[0]:.5f}, {pos[1]:.5f}) alt={pos[2]:5.1f}m "
            f"batt={t['battery']:5.1f}% gps={'ok' if t['gps']['available'] else 'LOST':<4} "
            f"{t['health']:<8} hb#{record.heartbeat_seq} link={'up' if record.link_alive else 'DOWN'}"
        )


def run_console(gcs: GroundControlStation, count: int) -> None:
    print("\nType NAVIGATE 1 (or HELP, STATUS, QUIT).\n")
    while True:
        try:
            line = input("gcs> ").strip()
        except (EOFError, KeyboardInterrupt):
            return
        if not line:
            continue
        parts = line.split()
        verb = parts[0].upper()
        target = parts[1] if len(parts) > 1 else ""

        if verb in ("QUIT", "EXIT"):
            return
        if verb == "HELP":
            print(__doc__)
            continue
        if verb == "STATUS":
            print_status(gcs)
            continue

        ids = list(range(1, count + 1)) if target.lower() == "all" else (
            [int(target)] if target.isdigit() else []
        )
        if not ids:
            print("  which drone? e.g. NAVIGATE 1")
            continue

        for drone_id in ids:
            if verb == "NAVIGATE":
                gcs.navigate(drone_id, SOURCE, DESTINATION)
            elif verb == "HOLD":
                gcs.send(drone_id, CommandName.HOLD)
            elif verb == "RESUME":
                gcs.send(drone_id, CommandName.RESUME)
            elif verb == "RTL":
                gcs.send(drone_id, CommandName.RETURN_TO_LAUNCH)
            elif verb == "LAND":
                gcs.send(drone_id, CommandName.LAND)
            elif verb == "GPS":
                state = (parts[2].lower() if len(parts) > 2 else "on") in ("on", "true", "1")
                gcs.send(drone_id, CommandName.SET_GPS, available=state)
            else:
                print(f"  unknown command {verb!r}")
                break


def run_script(gcs: GroundControlStation, fleet: DroneFleet, count: int) -> None:
    """Non-interactive sequence, so the demo is reproducible."""
    print()
    print("=== GCS sends NAVIGATE to drone 1 only ===")
    gcs.navigate(1, SOURCE, DESTINATION)
    time.sleep(2.0)
    print_status(gcs)

    print()
    print("=== drones 2 and 3 carry on independently; now they get theirs ===")
    for drone_id in range(2, count + 1):
        gcs.navigate(drone_id, SOURCE, DESTINATION)
    time.sleep(3.0)
    print_status(gcs)

    print()
    print("=== HOLD drone 2 - the others keep flying ===")
    gcs.send(2, CommandName.HOLD)
    time.sleep(2.5)
    print_status(gcs)

    print()
    print("=== GPS fault on drone 3, resume drone 2 ===")
    gcs.send(3, CommandName.SET_GPS, available=False)
    gcs.send(2, CommandName.RESUME)
    time.sleep(2.5)
    print_status(gcs)

    print()
    print("=== restore GPS on drone 3 ===")
    gcs.send(3, CommandName.SET_GPS, available=True)
    time.sleep(2.5)
    print_status(gcs)

    print()
    print("=== bidirectional link summary ===")
    for drone_id, record in sorted(gcs.all_drones().items()):
        link = fleet.link(drone_id)
        print(
            f"    drone {drone_id}: commands received={link.commands_received} "
            f"telemetry sent={link.telemetry_sent} heartbeats={link.heartbeats_sent} "
            f"| GCS got {record.telemetry_count} telemetry, {len(record.acks)} acks"
        )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="GCS <-> drone UDP demo")
    parser.add_argument("-n", "--drones", type=int, default=3)
    parser.add_argument("--time-scale", type=float, default=20.0)
    parser.add_argument("--script", action="store_true", help="run a fixed sequence and exit")
    parser.add_argument("--quiet", action="store_true", help="suppress per-message printing")
    args = parser.parse_args(argv)

    gcs, fleet = build(args.drones, args.time_scale, verbose=not args.quiet)
    print()
    print(f"GCS listening on {gcs.address[0]}:{gcs.address[1]}")
    for sysid in range(1, args.drones + 1):
        print(f"  drone {sysid} (SYSID {sysid}) on udp:127.0.0.1:{14700 + sysid}")

    if not gcs.wait_for_heartbeats(range(1, args.drones + 1), timeout_s=5.0):
        print("  warning: not every drone reported a heartbeat")

    try:
        if args.script:
            run_script(gcs, fleet, args.drones)
        else:
            run_console(gcs, args.drones)
    finally:
        print()
        fleet.shutdown()
        gcs.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
