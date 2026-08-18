"""Demo: the full chain down to real firmware.

    GCS -> Drone Thread -> Command Interpreter -> MAVLink -> ArduPilot/PX4 SITL
                                                                  -> drone moves

Run ArduPilot SITL first, streaming to this machine:

    sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14550

then:

    python -m engine.sitl_demo --navigate

Without SITL running the demo reports that nothing answered rather than
pretending to fly; use `--simulated` to run the same chain against the built-in
ArduPilot-shaped firmware instead.
"""
from __future__ import annotations

import argparse
import sys
import time

from engine.drone_thread import Command, CommandType, DroneThread, Position
from engine.flight_controller import (
    MavlinkFlightController,
    SimulatedFlightController,
)

# The worked example from the task specification.
SOURCE = Position(lat=16.30, lon=80.43, alt_m=20.0)
DESTINATION = Position(lat=16.31, lon=80.44, alt_m=20.0)


def wait_for_autopilot(fc: MavlinkFlightController, timeout_s: float) -> bool:
    """Block until the autopilot's first HEARTBEAT arrives."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        fc.update(0.1)
        fc.receive()
        if fc.connected:
            return True
        time.sleep(0.2)
    return False


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="GCS -> drone thread -> MAVLink -> SITL")
    parser.add_argument("--port", type=int, default=14550,
                        help="local UDP port SITL streams to (default 14550)")
    parser.add_argument("--simulated", action="store_true",
                        help="use the built-in firmware instead of SITL")
    parser.add_argument("--navigate", action="store_true",
                        help="send the NAVIGATE mission after connecting")
    parser.add_argument("--timeout", type=float, default=15.0,
                        help="seconds to wait for the autopilot")
    parser.add_argument("--duration", type=float, default=120.0,
                        help="seconds to follow the flight")
    parser.add_argument("--sysid", type=int, default=1)
    args = parser.parse_args(argv)

    if args.simulated:
        fc = SimulatedFlightController(
            sysid=args.sysid, home_lat=SOURCE.lat, home_lon=SOURCE.lon,
            cruise_speed_mps=15.0,
        )
        print("Flight controller: built-in ArduPilot-shaped firmware")
    else:
        fc = MavlinkFlightController(bind_port=args.port)
        print(f"Flight controller: real MAVLink, listening on udp:0.0.0.0:{args.port}")
        print("Waiting for an autopilot HEARTBEAT...")
        if not wait_for_autopilot(fc, args.timeout):
            print()
            print(f"  No autopilot answered on port {args.port} within {args.timeout:.0f}s.")
            print("  Start SITL first, for example:")
            print("    sim_vehicle.py -v ArduCopter --console --map "
                  f"--out=udp:127.0.0.1:{args.port}")
            print("  Or run this demo with --simulated to use the built-in firmware.")
            fc.close()
            return 1
        print(f"  Autopilot found at {fc.endpoint[0]}:{fc.endpoint[1]}")

    drone = DroneThread(
        drone_id=args.sysid, name=f"d{args.sysid}",
        start=SOURCE, cruise_altitude_m=DESTINATION.alt_m,
        cruise_speed_mps=15.0, flight_controller=fc,
        time_scale=1.0 if not args.simulated else 20.0,
    )
    drone.start()
    time.sleep(0.5)

    if args.navigate:
        print()
        print("NAVIGATE")
        print(f"  source      = ({SOURCE.lat}, {SOURCE.lon}, {SOURCE.alt_m:.0f})")
        print(f"  destination = ({DESTINATION.lat}, {DESTINATION.lon}, {DESTINATION.alt_m:.0f})")
        print()
        drone.send(Command(CommandType.GOTO, {
            "destination": DESTINATION, "altitude_m": DESTINATION.alt_m,
        }))

    deadline = time.monotonic() + args.duration
    last_state = None
    last_print = 0.0
    try:
        while time.monotonic() < deadline:
            time.sleep(0.25)
            snap = drone.snapshot()
            now = time.monotonic()
            if snap.mission_state != last_state or now - last_print >= 2.0:
                last_state, last_print = snap.mission_state, now
                st = fc.state()
                print(
                    f"  {snap.mission_state.value:<9} mode={st.mode_name:<9} "
                    f"armed={str(st.armed):<5} pos=({snap.position.lat:.5f}, {snap.position.lon:.5f}) "
                    f"alt={snap.position.alt_m:5.1f}m batt={snap.battery:5.1f}%"
                )
            if snap.mission_state.value in ("LANDED", "FAILED"):
                break
    except KeyboardInterrupt:
        pass

    snap = drone.snapshot()
    print()
    print(f"Final: {snap.mission_state.value} at "
          f"({snap.position.lat:.5f}, {snap.position.lon:.5f}) alt={snap.position.alt_m:.1f}m")
    print(f"Target: ({DESTINATION.lat}, {DESTINATION.lon})")
    if isinstance(fc, MavlinkFlightController):
        print(f"Link: {fc.frames_sent} frames sent, {fc.messages_seen} messages received")

    drone.shutdown()
    drone.join(timeout=2.0)
    if hasattr(fc, "close"):
        fc.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
