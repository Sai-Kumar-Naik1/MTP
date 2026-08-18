"""Command interpreter - the translation layer inside the drone.

    Drone Thread -> Command Interpreter -> MAVLink Message -> Flight Controller

Takes the drone's internal `Command` structure and emits real MAVLink v2
frames. Navigation is mapped to waypoint execution, using the actual MAVLink
mission protocol rather than a single position target:

    SET_MODE GUIDED
    COMPONENT_ARM_DISARM
    MISSION_COUNT(2)
        <- MISSION_REQUEST_INT(0)  ->  MISSION_ITEM_INT(0)  NAV_TAKEOFF
        <- MISSION_REQUEST_INT(1)  ->  MISSION_ITEM_INT(1)  NAV_WAYPOINT
        <- MISSION_ACK
    SET_MODE AUTO                       # the flight controller flies the plan

The upload is request-driven, so the interpreter keeps the pending mission and
answers each request as the flight controller asks for it - exactly as a real
GCS does. The output is bytes on the wire: whether those bytes reach a
simulated firmware or an actual ArduPilot over UDP is not this layer's concern.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from pymavlink.dialects.v20 import ardupilotmega as mav2

from engine.drone_thread import Command, CommandType, Position

# ArduPilot Copter flight modes (custom_mode in SET_MODE / HEARTBEAT).
MODE_STABILIZE = 0
MODE_AUTO = 3
MODE_GUIDED = 4
MODE_LOITER = 5
MODE_RTL = 6
MODE_LAND = 9
MODE_BRAKE = 17

MODE_NAMES = {
    MODE_STABILIZE: "STABILIZE",
    MODE_AUTO: "AUTO",
    MODE_GUIDED: "GUIDED",
    MODE_LOITER: "LOITER",
    MODE_RTL: "RTL",
    MODE_LAND: "LAND",
    MODE_BRAKE: "BRAKE",
}

GCS_SYSTEM = 255
GCS_COMPONENT = 190  # MAV_COMP_ID_MISSIONPLANNER


@dataclass
class MissionPlan:
    """A waypoint mission awaiting upload, item by item."""

    items: list[object] = field(default_factory=list)
    uploading: bool = False
    acked: bool = False

    @property
    def count(self) -> int:
        return len(self.items)


class CommandInterpreter:
    """Internal commands in, MAVLink frames out."""

    def __init__(self, sysid: int, target_system: int = 1, target_component: int = 1):
        self.sysid = sysid
        self.target_system = target_system
        self.target_component = target_component
        self._mav = mav2.MAVLink(None, srcSystem=GCS_SYSTEM, srcComponent=GCS_COMPONENT)
        self._mav.robust_parsing = True
        self.pending_mission = MissionPlan()
        self.sent_count = 0
        self.last_translation: list[str] = []

    # ---- Framing helpers ----

    def _frame(self, message) -> bytes:
        self.sent_count += 1
        self.last_translation.append(message.get_type())
        return message.pack(self._mav)

    def _set_mode(self, custom_mode: int) -> bytes:
        return self._frame(
            self._mav.command_long_encode(
                self.target_system, self.target_component,
                mav2.MAV_CMD_DO_SET_MODE, 0,
                mav2.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, custom_mode, 0, 0, 0, 0, 0,
            )
        )

    def _command(self, command_id: int, *params: float) -> bytes:
        padded = list(params) + [0.0] * (7 - len(params))
        return self._frame(
            self._mav.command_long_encode(
                self.target_system, self.target_component, command_id, 0, *padded[:7]
            )
        )

    # ---- Translation ----

    def interpret(self, command: Command) -> list[bytes]:
        """One internal command -> the MAVLink frames that carry it out."""
        self.last_translation = []
        kind, payload = command.type, command.payload

        if kind is CommandType.ARM:
            return [self._set_mode(MODE_GUIDED),
                    self._command(mav2.MAV_CMD_COMPONENT_ARM_DISARM, 1)]

        if kind is CommandType.DISARM:
            return [self._command(mav2.MAV_CMD_COMPONENT_ARM_DISARM, 0)]

        if kind is CommandType.TAKEOFF:
            altitude = float(payload.get("altitude_m", 0.0))
            if altitude <= 0:
                return []
            return [self._command(
                mav2.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude)]

        if kind is CommandType.GOTO:
            destination = payload.get("destination")
            if destination is None:
                return []
            return self.build_waypoint_mission(
                destination, altitude_m=float(payload.get("altitude_m", destination.alt_m or 50.0))
            )

        if kind is CommandType.HOLD:
            return [self._set_mode(MODE_BRAKE)]

        if kind is CommandType.RESUME:
            return [self._set_mode(MODE_AUTO)]

        if kind is CommandType.RETURN_TO_LAUNCH:
            return [self._command(mav2.MAV_CMD_NAV_RETURN_TO_LAUNCH)]

        if kind is CommandType.LAND:
            return [self._set_mode(MODE_LAND)]

        if kind is CommandType.ABORT:
            return [self._set_mode(MODE_LAND),
                    self._command(mav2.MAV_CMD_COMPONENT_ARM_DISARM, 0)]

        return []  # SET_GPS, HEARTBEAT and SHUTDOWN are not firmware commands

    def build_waypoint_mission(self, destination: Position, altitude_m: float = 50.0) -> list[bytes]:
        """Navigation as waypoint execution: stage the plan and offer the count.

        The items themselves go out as the flight controller requests them.
        """
        takeoff = self._mav.mission_item_int_encode(
            self.target_system, self.target_component,
            0,                                        # seq
            mav2.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mav2.MAV_CMD_NAV_TAKEOFF,
            0, 1,                                     # current, autocontinue
            0, 0, 0, 0,
            0, 0, altitude_m,
        )
        waypoint = self._mav.mission_item_int_encode(
            self.target_system, self.target_component,
            1,
            mav2.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mav2.MAV_CMD_NAV_WAYPOINT,
            0, 1,
            0, 0, 0, 0,
            int(destination.lat * 1e7), int(destination.lon * 1e7), altitude_m,
        )
        self.pending_mission = MissionPlan(items=[takeoff, waypoint], uploading=True)

        frames = [
            self._set_mode(MODE_GUIDED),
            self._command(mav2.MAV_CMD_COMPONENT_ARM_DISARM, 1),
            self._frame(
                self._mav.mission_count_encode(
                    self.target_system, self.target_component,
                    self.pending_mission.count, mav2.MAV_MISSION_TYPE_MISSION,
                )
            ),
        ]
        return frames

    # ---- Uplink handling (the mission protocol is a conversation) ----

    def on_message(self, message) -> list[bytes]:
        """Answer whatever the flight controller asks of us."""
        kind = message.get_type()

        if kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            seq = message.seq
            if not self.pending_mission.uploading or seq >= self.pending_mission.count:
                return []
            item = self.pending_mission.items[seq]
            return [self._frame(item)]

        if kind == "MISSION_ACK":
            if self.pending_mission.uploading:
                self.pending_mission.uploading = False
                self.pending_mission.acked = True
                # Plan is aboard: hand control to it.
                return [self._set_mode(MODE_AUTO)]
            return []

        return []
