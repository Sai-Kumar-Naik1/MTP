"""Ground Control Station - the other end of the link.

    GCS
     |  UDP
     +------> Drone 1   (SYSID 1, udp:14701)
     +------> Drone 2   (SYSID 2, udp:14702)
     +------> Drone N   (SYSID N, udp:1470N)

One socket addresses the whole swarm: commands are routed by SYSID to the
matching port, and every drone reports back to this single socket. A receiver
thread keeps the latest telemetry and heartbeat per drone, so link loss is
observable rather than assumed.
"""
from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable, Optional

from engine.protocol import (
    DRONE_BASE_PORT,
    GCS_PORT,
    CommandMessage,
    CommandName,
    MAX_DATAGRAM,
    MessageType,
    ProtocolError,
    drone_port,
)

HEARTBEAT_TIMEOUT_S = 3.0


@dataclass
class DroneRecord:
    """What the GCS knows about one drone, from its reports alone."""

    drone_id: int
    address: tuple[str, int]
    telemetry: dict[str, Any] = field(default_factory=dict)
    last_heartbeat_at: float = 0.0
    heartbeat_seq: int = 0
    acks: list[dict[str, Any]] = field(default_factory=list)
    telemetry_count: int = 0

    @property
    def link_alive(self) -> bool:
        if not self.last_heartbeat_at:
            return False
        return (time.monotonic() - self.last_heartbeat_at) < HEARTBEAT_TIMEOUT_S

    @property
    def silence_s(self) -> float:
        if not self.last_heartbeat_at:
            return float("inf")
        return time.monotonic() - self.last_heartbeat_at


class GroundControlStation:
    """Sends commands to drones by SYSID and collects their telemetry."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = GCS_PORT,
        drone_host: str = "127.0.0.1",
        drone_base_port: int = DRONE_BASE_PORT,
        verbose: bool = False,
    ):
        self.drone_host = drone_host
        self.drone_base_port = drone_base_port
        self.verbose = verbose

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.settimeout(0.2)
        self.address = self._sock.getsockname()

        self._drones: dict[int, DroneRecord] = {}
        self._lock = threading.Lock()
        self._seq = 0
        self._running = threading.Event()
        self._receiver: Optional[threading.Thread] = None
        self.on_telemetry: Optional[Callable[[int, dict], None]] = None

    # ---- Lifecycle ----

    def start(self) -> None:
        """Begin listening for telemetry."""
        if self._running.is_set():
            return
        self._running.set()
        self._receiver = threading.Thread(target=self._receive_loop, name="GCS-RX", daemon=True)
        self._receiver.start()

    def stop(self) -> None:
        self._running.clear()
        if self._receiver is not None:
            self._receiver.join(timeout=1.0)
            self._receiver = None
        self._sock.close()

    def register(self, drone_id: int) -> DroneRecord:
        """Note a drone's existence and where to reach it."""
        with self._lock:
            record = self._drones.get(drone_id)
            if record is None:
                record = DroneRecord(
                    drone_id=drone_id,
                    address=(self.drone_host, drone_port(drone_id, self.drone_base_port)),
                )
                self._drones[drone_id] = record
            return record

    def register_many(self, drone_ids: Iterable[int]) -> None:
        for drone_id in drone_ids:
            self.register(drone_id)

    # ---- Outbound: commands ----

    def send(
        self,
        drone_id: int,
        command: CommandName,
        source: Optional[list[float]] = None,
        destination: Optional[list[float]] = None,
        **params: Any,
    ) -> int:
        """Address one drone by SYSID. Returns the sequence number used."""
        record = self.register(drone_id)
        with self._lock:
            self._seq += 1
            seq = self._seq
        message = CommandMessage(
            drone_id=drone_id,
            command=command,
            source=source,
            destination=destination,
            params=params,
            seq=seq,
        )
        self._sock.sendto(message.to_json(), record.address)
        if self.verbose:
            print(f"GCS -> drone {drone_id}: {command.value} (seq {seq})")
        return seq

    def navigate(
        self,
        drone_id: int,
        source: list[float],
        destination: list[float],
    ) -> int:
        """The mission command:

            {"drone_id": 1, "command": "NAVIGATE",
             "source": [lat1, lon1, alt1], "destination": [lat2, lon2, alt2]}
        """
        return self.send(drone_id, CommandName.NAVIGATE, source=source, destination=destination)

    def broadcast(self, command: CommandName, drone_ids: Optional[Iterable[int]] = None, **params) -> int:
        targets = list(drone_ids) if drone_ids is not None else list(self._drones)
        for drone_id in targets:
            self.send(drone_id, command, **params)
        return len(targets)

    # ---- Inbound: telemetry ----

    def _receive_loop(self) -> None:
        while self._running.is_set():
            try:
                raw, sender = self._sock.recvfrom(MAX_DATAGRAM)
            except (socket.timeout, TimeoutError):
                continue
            except OSError:
                return
            try:
                body = self._decode(raw)
            except ProtocolError:
                continue
            self._apply(body, sender)

    @staticmethod
    def _decode(raw: bytes) -> dict[str, Any]:
        from engine.protocol import decode_uplink

        return decode_uplink(raw)

    def _apply(self, body: dict[str, Any], sender: tuple[str, int]) -> None:
        drone_id = body.get("drone_id")
        if not isinstance(drone_id, int):
            return
        record = self.register(drone_id)
        kind = body.get("type")

        with self._lock:
            record.address = record.address or sender
            if kind == MessageType.TELEMETRY.value:
                record.telemetry = body
                record.telemetry_count += 1
            elif kind == MessageType.HEARTBEAT.value:
                record.last_heartbeat_at = time.monotonic()
                record.heartbeat_seq = int(body.get("seq", 0))
            elif kind == MessageType.ACK.value:
                record.acks.append(body)
                if len(record.acks) > 100:
                    del record.acks[:50]

        if kind == MessageType.TELEMETRY.value and self.on_telemetry is not None:
            self.on_telemetry(drone_id, body)

    # ---- Queries ----

    def drone(self, drone_id: int) -> Optional[DroneRecord]:
        with self._lock:
            return self._drones.get(drone_id)

    def all_drones(self) -> dict[int, DroneRecord]:
        with self._lock:
            return dict(self._drones)

    def telemetry(self, drone_id: int) -> dict[str, Any]:
        record = self.drone(drone_id)
        return dict(record.telemetry) if record else {}

    def alive_ids(self) -> list[int]:
        return sorted(i for i, r in self.all_drones().items() if r.link_alive)

    def lost_ids(self) -> list[int]:
        return sorted(i for i, r in self.all_drones().items() if not r.link_alive)

    def wait_for_heartbeats(self, drone_ids: Iterable[int], timeout_s: float = 5.0) -> bool:
        """Block until every listed drone has reported in, or time out."""
        wanted = set(drone_ids)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if wanted <= set(self.alive_ids()):
                return True
            time.sleep(0.05)
        return False
