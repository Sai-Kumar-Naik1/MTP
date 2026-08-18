"""Drone-side UDP endpoint.

Owns one socket per drone, bound to a port derived from its SYSID. Translates
wire messages into the drone's internal `Command` structure on the way in, and
snapshots into telemetry/heartbeat datagrams on the way out.

    GCS ----- COMMAND -----> udp:1470<SYSID>
    GCS <-- TELEMETRY/HEARTBEAT/ACK -- drone

The drone thread itself never sees a socket: this class is handed to it as a
`command_source` callable and a `telemetry_sink` callable, so the node stays
transport-agnostic and still needs only its own single thread.
"""
from __future__ import annotations

import socket
import threading
import time
from typing import Optional

from engine.drone_thread import (
    Command,
    CommandType,
    DroneSnapshot,
    Position,
)
from engine.protocol import (
    GCS_PORT,
    AckMessage,
    CommandMessage,
    CommandName,
    HeartbeatMessage,
    MAX_DATAGRAM,
    ProtocolError,
    TelemetryMessage,
    drone_port,
)

HEARTBEAT_INTERVAL_S = 1.0
TELEMETRY_INTERVAL_S = 0.5

# Wire command -> internal command. NAVIGATE expands to a small sequence,
# handled separately below.
_DIRECT_COMMANDS = {
    CommandName.ARM: CommandType.ARM,
    CommandName.DISARM: CommandType.DISARM,
    CommandName.TAKEOFF: CommandType.TAKEOFF,
    CommandName.HOLD: CommandType.HOLD,
    CommandName.RESUME: CommandType.RESUME,
    CommandName.RETURN_TO_LAUNCH: CommandType.RETURN_TO_LAUNCH,
    CommandName.LAND: CommandType.LAND,
    CommandName.ABORT: CommandType.ABORT,
    CommandName.PING: CommandType.HEARTBEAT,
    CommandName.SHUTDOWN: CommandType.SHUTDOWN,
}


def _to_position(triple: Optional[list]) -> Optional[Position]:
    """`[lat, lon, alt]` -> Position. Altitude is optional."""
    if not triple or len(triple) < 2:
        return None
    alt = float(triple[2]) if len(triple) > 2 else 0.0
    return Position(lat=float(triple[0]), lon=float(triple[1]), alt_m=alt)


class DroneLink:
    """One drone's UDP presence on the network."""

    def __init__(
        self,
        sysid: int,
        gcs_address: tuple[str, int] = ("127.0.0.1", GCS_PORT),
        host: str = "127.0.0.1",
        base_port: int = 14700,
        verbose: bool = True,
    ):
        self.sysid = sysid
        self.gcs_address = gcs_address
        self.port = drone_port(sysid, base_port)
        self.verbose = verbose

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, self.port))
        self._sock.setblocking(False)

        self._started_at = time.monotonic()
        self._last_heartbeat = 0.0
        self._last_telemetry = 0.0
        self._heartbeat_seq = 0
        self._lock = threading.Lock()

        # Counters, useful for proving independence in tests.
        self.commands_received = 0
        self.telemetry_sent = 0
        self.heartbeats_sent = 0
        self.last_command: Optional[str] = None

    def close(self) -> None:
        self._sock.close()

    # ---- Inbound: GCS -> drone ----

    def poll_commands(self) -> list[Command]:
        """Drain waiting datagrams and translate them. Never blocks.

        This is the `command_source` the drone thread polls each tick.
        """
        commands: list[Command] = []
        while True:
            try:
                raw, sender = self._sock.recvfrom(MAX_DATAGRAM)
            except BlockingIOError:
                return commands
            except OSError:
                return commands

            try:
                message = CommandMessage.from_json(raw)
            except ProtocolError as exc:
                self._send(AckMessage(self.sysid, "?", 0, False, f"malformed: {exc}").to_json())
                continue

            if message.drone_id != self.sysid:
                # Addressed to a different drone; ignore rather than obey.
                continue

            with self._lock:
                self.commands_received += 1
                self.last_command = message.command.value

            if self.verbose:
                print(f"Drone {self.sysid} received {message.command.value} command")

            translated = self._translate(message)
            commands.extend(translated)
            self._send(
                AckMessage(
                    self.sysid,
                    message.command.value,
                    message.seq,
                    accepted=bool(translated),
                    detail="" if translated else "no effect",
                ).to_json()
            )
        return commands

    def _translate(self, message: CommandMessage) -> list[Command]:
        """Wire message -> the drone's internal command structure."""
        name = message.command

        if name is CommandName.NAVIGATE:
            # The one mission operation: fly source -> destination. Expands to
            # arm, take off, and route, which the drone's state machine already
            # sequences correctly regardless of its current phase.
            destination = _to_position(message.destination)
            if destination is None:
                return []
            source = _to_position(message.source)
            out = [Command(CommandType.ARM)]
            altitude = destination.alt_m or (source.alt_m if source else 0.0)
            payload = {"altitude_m": altitude} if altitude else {}
            out.append(Command(CommandType.TAKEOFF, payload))
            out.append(Command(CommandType.GOTO, {"destination": destination}))
            return out

        if name is CommandName.SET_GPS:
            available = bool(message.params.get("available", True))
            return [Command(CommandType.SET_GPS, {"available": available})]

        internal = _DIRECT_COMMANDS.get(name)
        if internal is None:
            return []
        payload = dict(message.params)
        if name is CommandName.TAKEOFF and message.destination:
            payload.setdefault("altitude_m", _to_position(message.destination).alt_m)
        return [Command(internal, payload)]

    # ---- Outbound: drone -> GCS ----

    def publish(self, snapshot: DroneSnapshot) -> None:
        """Telemetry sink for the drone thread. Rate-limits the two streams."""
        now = time.monotonic()

        if now - self._last_telemetry >= TELEMETRY_INTERVAL_S:
            self._last_telemetry = now
            self._send(self._telemetry_of(snapshot).to_json())
            with self._lock:
                self.telemetry_sent += 1

        if now - self._last_heartbeat >= HEARTBEAT_INTERVAL_S:
            self._last_heartbeat = now
            self._heartbeat_seq += 1
            self._send(
                HeartbeatMessage(
                    drone_id=self.sysid,
                    seq=self._heartbeat_seq,
                    uptime_s=now - self._started_at,
                    mission_state=snapshot.mission_state.value,
                ).to_json()
            )
            with self._lock:
                self.heartbeats_sent += 1

    def _telemetry_of(self, snapshot: DroneSnapshot) -> TelemetryMessage:
        destination = None
        if snapshot.destination is not None:
            destination = [
                snapshot.destination.lat,
                snapshot.destination.lon,
                snapshot.destination.alt_m,
            ]
        return TelemetryMessage(
            drone_id=self.sysid,
            position=[snapshot.position.lat, snapshot.position.lon, snapshot.position.alt_m],
            velocity={
                "ground_speed_mps": snapshot.velocity.ground_speed_mps,
                "heading_deg": snapshot.velocity.heading_deg,
                "vertical_speed_mps": snapshot.velocity.vertical_speed_mps,
            },
            battery=round(snapshot.battery, 2),
            gps={"available": snapshot.gps_available},
            health=snapshot.health_status.value,
            mission_state=snapshot.mission_state.value,
            armed=snapshot.armed_status,
            operation=snapshot.current_operation.value,
            communication=snapshot.communication_status.value,
            destination=destination,
            elapsed_s=round(snapshot.elapsed_s, 2),
            note=snapshot.note,
        )

    def _send(self, payload: bytes) -> None:
        try:
            self._sock.sendto(payload, self.gcs_address)
        except OSError:
            pass  # the GCS is not listening; the drone keeps flying regardless
