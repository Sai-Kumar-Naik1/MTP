from __future__ import annotations

import time
from dataclasses import dataclass
from threading import Event, Thread
from typing import Any, Dict, List, Optional

from engine.drone_runtime import DronePlanCommand, DroneState


@dataclass
class DroneFlightStatus:
    drone_id: str
    status: str
    current_lat: float
    current_lon: float
    current_alt: float
    message: str


@dataclass
class FleetTelemetry:
    drone_id: str
    name: str
    status: str
    current_lat: float
    current_lon: float
    current_alt: float
    features: List[str]
    protocol: str


class DroneThread(Thread):
    def __init__(self, drone: DroneState, command_queue: Optional[List[DronePlanCommand]] = None):
        super().__init__(daemon=True)
        self.drone = drone
        self.command_queue = command_queue if command_queue is not None else []
        self._stop_event = Event()
        self._status_callback: Optional[Any] = None

    def set_status_callback(self, callback):
        self._status_callback = callback

    def stop(self):
        self._stop_event.set()

    def run(self):
        self.drone.status = "ARMED"
        self._emit_status("Drone armed and waiting for plan execution")

        for command in self.command_queue:
            if self._stop_event.is_set():
                self.drone.status = "STOPPED"
                self._emit_status("Drone execution stopped")
                return

            self._execute_command(command)
            time.sleep(0.5)

        self.drone.status = "LANDED"
        self._emit_status("Drone completed path and landed")

    def _execute_command(self, command: DronePlanCommand) -> None:
        if command.command_type == "TAKEOFF":
            alt = command.params.get("alt", 10)
            self.drone.current_alt = float(alt)
            self.drone.status = "TAKING_OFF"
            self._emit_status(f"TAKEOFF to {alt}m")
        elif command.command_type == "MOVE":
            lat = float(command.params.get("lat", self.drone.current_lat))
            lon = float(command.params.get("lon", self.drone.current_lon))
            alt = float(command.params.get("alt", self.drone.current_alt))
            self.drone.current_lat = lat
            self.drone.current_lon = lon
            self.drone.current_alt = alt
            self.drone.status = "FLYING"
            self._emit_status(f"MOVE to ({lat}, {lon}, {alt})")
        elif command.command_type == "LAND":
            self.drone.current_alt = 0.0
            self.drone.status = "LANDING"
            self._emit_status("LAND")
        else:
            self.drone.status = "EXECUTING"
            self._emit_status(f"UNKNOWN command {command.command_type}")

    def _emit_status(self, message: str) -> None:
        if self._status_callback:
            self._status_callback(
                DroneFlightStatus(
                    drone_id=self.drone.drone_id,
                    status=self.drone.status,
                    current_lat=self.drone.current_lat,
                    current_lon=self.drone.current_lon,
                    current_alt=self.drone.current_alt,
                    message=message,
                )
            )


class DroneOrchestrator:
    def __init__(self):
        self._threads: Dict[str, DroneThread] = {}

    def create_drone_thread(self, drone: DroneState, command_queue: List[DronePlanCommand]) -> DroneThread:
        existing = self._threads.get(drone.drone_id)
        if existing is not None and existing.is_alive():
            return existing

        worker = DroneThread(drone, command_queue)
        self._threads[drone.drone_id] = worker
        return worker

    def start_drone(self, drone_id: str):
        thread = self._threads.get(drone_id)
        if thread is not None and not thread.is_alive():
            thread.start()

    def stop_drone(self, drone_id: str):
        thread = self._threads.get(drone_id)
        if thread is not None:
            thread.stop()

    def all_threads(self) -> Dict[str, DroneThread]:
        return dict(self._threads)

    def start_all(self):
        for drone_id in list(self._threads):
            self.start_drone(drone_id)

    def stop_all(self):
        for drone_id in list(self._threads):
            self.stop_drone(drone_id)

    def telemetry_snapshot(self, drones: Dict[str, DroneState]) -> List[FleetTelemetry]:
        snapshot: List[FleetTelemetry] = []
        for drone_id, drone in drones.items():
            snapshot.append(
                FleetTelemetry(
                    drone_id=drone.drone_id,
                    name=drone.name,
                    status=drone.status,
                    current_lat=drone.current_lat,
                    current_lon=drone.current_lon,
                    current_alt=drone.current_alt,
                    features=list(drone.features),
                    protocol=drone.protocol,
                )
            )
        return snapshot
