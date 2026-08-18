"""GCS <-> drone wire protocol.

JSON over UDP, one datagram per message. Deliberately simple and human
readable: this is the transport that later gets swapped for MAVLink, and
keeping the schema explicit makes that substitution a translation exercise
rather than a rewrite.

    GCS ----- Command -----> Drone      (COMMAND)
    GCS <---- Telemetry ---- Drone      (TELEMETRY, HEARTBEAT, ACK)

Every message carries `drone_id`, which is the drone's MAVLink SYSID, so a
single GCS socket can address and demultiplex a whole swarm.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional

# Each drone listens on its own port so the swarm needs no demultiplexing on
# the way down; SYSID 1 -> 14701, SYSID 2 -> 14702, and so on.
DRONE_BASE_PORT = 14700
GCS_PORT = 14690
MAX_DATAGRAM = 65535


def drone_port(sysid: int, base_port: int = DRONE_BASE_PORT) -> int:
    return base_port + sysid


class MessageType(str, Enum):
    COMMAND = "COMMAND"
    TELEMETRY = "TELEMETRY"
    HEARTBEAT = "HEARTBEAT"
    ACK = "ACK"


class CommandName(str, Enum):
    """Commands the GCS can issue. NAVIGATE is the one mission operation."""

    NAVIGATE = "NAVIGATE"
    ARM = "ARM"
    DISARM = "DISARM"
    TAKEOFF = "TAKEOFF"
    HOLD = "HOLD"
    RESUME = "RESUME"
    RETURN_TO_LAUNCH = "RETURN_TO_LAUNCH"
    LAND = "LAND"
    ABORT = "ABORT"
    SET_GPS = "SET_GPS"
    PING = "PING"
    SHUTDOWN = "SHUTDOWN"


class ProtocolError(ValueError):
    """A datagram was not a message we understand."""


@dataclass
class CommandMessage:
    """GCS -> drone.

    Wire form, matching the agreed schema:

        {"drone_id": 1, "command": "NAVIGATE",
         "source": [lat, lon, alt], "destination": [lat, lon, alt]}
    """

    drone_id: int
    command: CommandName
    source: Optional[list[float]] = None
    destination: Optional[list[float]] = None
    params: dict[str, Any] = field(default_factory=dict)
    seq: int = 0
    timestamp: float = field(default_factory=time.time)

    def to_json(self) -> bytes:
        body: dict[str, Any] = {
            "type": MessageType.COMMAND.value,
            "drone_id": self.drone_id,
            "command": self.command.value,
            "seq": self.seq,
            "timestamp": self.timestamp,
        }
        if self.source is not None:
            body["source"] = self.source
        if self.destination is not None:
            body["destination"] = self.destination
        if self.params:
            body["params"] = self.params
        return json.dumps(body).encode("utf-8")

    @classmethod
    def from_json(cls, raw: bytes) -> "CommandMessage":
        try:
            body = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ProtocolError(f"undecodable datagram: {exc}") from exc
        if not isinstance(body, dict):
            raise ProtocolError("message body is not an object")
        try:
            command = CommandName(body["command"])
        except KeyError as exc:
            raise ProtocolError("command message has no 'command' field") from exc
        except ValueError as exc:
            raise ProtocolError(f"unknown command {body.get('command')!r}") from exc
        try:
            drone_id = int(body["drone_id"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ProtocolError("command message has no usable 'drone_id'") from exc

        return cls(
            drone_id=drone_id,
            command=command,
            source=body.get("source"),
            destination=body.get("destination"),
            params=body.get("params", {}),
            seq=int(body.get("seq", 0)),
            timestamp=float(body.get("timestamp", time.time())),
        )


@dataclass
class TelemetryMessage:
    """Drone -> GCS: position, battery, GPS, health, mission state."""

    drone_id: int
    position: list[float]
    velocity: dict[str, float]
    battery: float
    gps: dict[str, Any]
    health: str
    mission_state: str
    armed: bool
    operation: str
    communication: str
    destination: Optional[list[float]] = None
    elapsed_s: float = 0.0
    note: str = ""
    timestamp: float = field(default_factory=time.time)

    def to_json(self) -> bytes:
        return json.dumps(
            {
                "type": MessageType.TELEMETRY.value,
                "drone_id": self.drone_id,
                "position": self.position,
                "velocity": self.velocity,
                "battery": self.battery,
                "gps": self.gps,
                "health": self.health,
                "mission_state": self.mission_state,
                "armed": self.armed,
                "operation": self.operation,
                "communication": self.communication,
                "destination": self.destination,
                "elapsed_s": self.elapsed_s,
                "note": self.note,
                "timestamp": self.timestamp,
            }
        ).encode("utf-8")


@dataclass
class HeartbeatMessage:
    """Drone -> GCS: proof of life, independent of mission activity."""

    drone_id: int
    seq: int
    uptime_s: float
    mission_state: str
    timestamp: float = field(default_factory=time.time)

    def to_json(self) -> bytes:
        return json.dumps(
            {
                "type": MessageType.HEARTBEAT.value,
                "drone_id": self.drone_id,
                "seq": self.seq,
                "uptime_s": self.uptime_s,
                "mission_state": self.mission_state,
                "timestamp": self.timestamp,
            }
        ).encode("utf-8")


@dataclass
class AckMessage:
    """Drone -> GCS: this command was received and accepted (or refused)."""

    drone_id: int
    command: str
    seq: int
    accepted: bool
    detail: str = ""
    timestamp: float = field(default_factory=time.time)

    def to_json(self) -> bytes:
        return json.dumps(
            {
                "type": MessageType.ACK.value,
                "drone_id": self.drone_id,
                "command": self.command,
                "seq": self.seq,
                "accepted": self.accepted,
                "detail": self.detail,
                "timestamp": self.timestamp,
            }
        ).encode("utf-8")


def decode_uplink(raw: bytes) -> dict[str, Any]:
    """Parse anything a drone sends to the GCS."""
    try:
        body = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError(f"undecodable datagram: {exc}") from exc
    if not isinstance(body, dict) or "type" not in body:
        raise ProtocolError("uplink message has no 'type'")
    return body
