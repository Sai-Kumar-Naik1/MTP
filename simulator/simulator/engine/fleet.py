"""Fleet manager - spawns and supervises one `DroneThread` per UAV.

    Main Program
       ├── DroneThread(1)
       ├── DroneThread(2)
       └── DroneThread(N)

Owns no drone state of its own: it starts threads, fans commands out to them,
and collects the snapshots they publish. Every drone remains the sole owner and
sole mutator of its own state, so this class is safe to call from any thread.
"""
from __future__ import annotations

import queue
import threading
from typing import Iterable, Optional

from engine.drone_thread import (
    Command,
    CommandType,
    DroneSnapshot,
    DroneThread,
    Position,
)


class DroneFleet:
    """Spawns a thread per drone and gathers their telemetry."""

    def __init__(self, telemetry_queue: Optional["queue.Queue[DroneSnapshot]"] = None):
        self._drones: dict[int, DroneThread] = {}
        self._latest: dict[int, DroneSnapshot] = {}
        self._links: dict[int, object] = {}
        self._lock = threading.Lock()
        self.telemetry_queue: "queue.Queue[DroneSnapshot]" = telemetry_queue or queue.Queue()

    # ---- Lifecycle ----

    def spawn(
        self,
        drone_id: int,
        name: str = "",
        *,
        cruise_speed_mps: float = 15.0,
        cruise_altitude_m: float = 50.0,
        battery_capacity_mah: float = 5200.0,
        start: Optional[Position] = None,
        tick_s: float = 0.1,
        time_scale: float = 1.0,
        link: Optional[object] = None,
        flight_controller: Optional[object] = None,
    ) -> DroneThread:
        """Create and start one drone node.

        `link` is an optional transport (see `engine.drone_link.DroneLink`)
        exposing `publish(snapshot)` and `poll_commands()`. When given, the
        drone also answers to the network - the fleet keeps collecting
        telemetry either way.

        `flight_controller` is an optional firmware (see
        `engine.flight_controller`). With one attached the drone translates its
        commands to MAVLink and the controller flies the airframe.
        """
        if drone_id in self._drones:
            raise ValueError(f"drone {drone_id} already exists")

        if link is None:
            sink = self._collect
            source = None
        else:
            def sink(snapshot: DroneSnapshot, _link=link) -> None:
                self._collect(snapshot)
                _link.publish(snapshot)

            source = link.poll_commands
            self._links[drone_id] = link

        drone = DroneThread(
            drone_id=drone_id,
            name=name,
            cruise_speed_mps=cruise_speed_mps,
            cruise_altitude_m=cruise_altitude_m,
            battery_capacity_mah=battery_capacity_mah,
            start=start,
            telemetry_sink=sink,
            command_source=source,
            flight_controller=flight_controller,
            tick_s=tick_s,
            time_scale=time_scale,
        )
        self._drones[drone_id] = drone
        drone.start()
        return drone

    def spawn_many(self, count: int, **kwargs) -> list[DroneThread]:
        """`DroneThread(1) .. DroneThread(N)`."""
        return [self.spawn(drone_id=i, **kwargs) for i in range(1, count + 1)]

    def shutdown(self, timeout_s: float = 2.0) -> None:
        for drone in self._drones.values():
            drone.shutdown()
        for drone in self._drones.values():
            drone.join(timeout=timeout_s)
        for link in self._links.values():
            close = getattr(link, "close", None)
            if close is not None:
                close()
        self._drones.clear()
        self._links.clear()

    def link(self, drone_id: int) -> Optional[object]:
        return self._links.get(drone_id)

    # ---- Command fan-out ----

    def send(self, drone_id: int, command: Command) -> bool:
        drone = self._drones.get(drone_id)
        if drone is None:
            return False
        drone.send(command)
        return True

    def broadcast(self, command: Command, drone_ids: Optional[Iterable[int]] = None) -> int:
        targets = self._drones.values() if drone_ids is None else [
            self._drones[i] for i in drone_ids if i in self._drones
        ]
        for drone in targets:
            drone.send(command)
        return len(list(targets))

    def start_mission(self, destination: Position, drone_ids: Optional[Iterable[int]] = None) -> int:
        """Arm, take off, and route to `destination` - the one supported task."""
        count = self.broadcast(Command(CommandType.ARM), drone_ids)
        self.broadcast(Command(CommandType.TAKEOFF), drone_ids)
        self.broadcast(
            Command(CommandType.GOTO, {"destination": destination}), drone_ids
        )
        return count

    # ---- Telemetry ----

    def _collect(self, snapshot: DroneSnapshot) -> None:
        """Telemetry sink - runs on each drone's own thread."""
        with self._lock:
            self._latest[snapshot.drone_id] = snapshot
        self.telemetry_queue.put(snapshot)

    def latest(self) -> dict[int, DroneSnapshot]:
        with self._lock:
            return dict(self._latest)

    def drain(self, limit: int = 1000) -> list[DroneSnapshot]:
        """Pull queued snapshots without blocking (for a GUI poll timer)."""
        out: list[DroneSnapshot] = []
        for _ in range(limit):
            try:
                out.append(self.telemetry_queue.get_nowait())
            except queue.Empty:
                break
        return out

    # ---- Introspection ----

    @property
    def drones(self) -> list[DroneThread]:
        return list(self._drones.values())

    def get(self, drone_id: int) -> Optional[DroneThread]:
        return self._drones.get(drone_id)

    def all_finished(self) -> bool:
        return bool(self._drones) and all(d.mission_finished for d in self._drones.values())

    def alive_count(self) -> int:
        return sum(1 for d in self._drones.values() if d.is_alive())
