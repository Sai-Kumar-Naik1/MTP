"""Module 1 <-> Module 2 contract (SRS §6.1).

Shared Pydantic schemas for the REST commands and WebSocket telemetry stream
exchanged between the GUI (Module 1) and the Orchestration & Swarm Router
(Module 2). Module 1 depends only on this contract (§3.3) - it has no direct
knowledge of Module 3 or of MAVLink encoding internals.
"""
from __future__ import annotations

from enum import Enum
from typing import Optional

from pydantic import BaseModel, Field


class SensorType(str, Enum):
    GPS = "gps"
    IMU = "imu"
    BAROMETER = "barometer"
    MAGNETOMETER = "magnetometer"
    CAMERA = "camera"
    LIDAR = "lidar"


class LatLon(BaseModel):
    lat: float = Field(..., ge=-90, le=90)
    lon: float = Field(..., ge=-180, le=180)


class DroneConfig(BaseModel):
    """Drone configuration profile (SRS §3.2.1)."""

    name: str
    sysid: int = Field(..., ge=1, le=255)
    mass_kg: float = Field(..., gt=0)
    max_velocity_mps: float = Field(..., gt=0)
    battery_capacity_mah: float = Field(..., gt=0)
    # Cruise altitude flown between waypoints. Defaulted so profiles saved
    # before this field existed still load.
    cruise_altitude_m: float = Field(default=50.0, gt=0)
    sensors: list[SensorType] = Field(default_factory=list)


class SwarmPreset(BaseModel):
    """A named, saved group of drone profiles (SRS §3.2.1 - swarm selector)."""

    name: str
    drones: list[DroneConfig] = Field(default_factory=list)


# --------------------------------------------------------------------------
# Commands: Module 1 -> Module 2 (SRS §6.1, §4.2)
# --------------------------------------------------------------------------


class RegisterDrone(BaseModel):
    drone: DroneConfig


class Takeoff(BaseModel):
    sysid: int
    target_altitude_m: float = Field(..., gt=0)


class Goto(BaseModel):
    sysid: int
    destination: LatLon
    altitude_m: Optional[float] = None


class FlockCommand(BaseModel):
    sysids: list[int]
    destination: LatLon
    altitude_m: Optional[float] = None


class FaultType(str, Enum):
    GPS_LOSS = "gps_loss"
    COMMS_LOSS = "comms_loss"
    BATTERY_FAIL = "battery_fail"


class FaultSeverity(str, Enum):
    PARTIAL = "partial"
    FULL = "full"


class InjectFault(BaseModel):
    sysids: list[int]
    fault_type: FaultType
    severity: FaultSeverity = FaultSeverity.FULL
    duration_s: Optional[float] = None


class StopSwarm(BaseModel):
    reason: Optional[str] = None


# --------------------------------------------------------------------------
# Telemetry: Module 2 -> Module 1 (SRS §6.1)
# --------------------------------------------------------------------------


class DroneStatus(str, Enum):
    STANDBY = "STANDBY"
    ARMED = "ARMED"
    TAKING_OFF = "TAKING_OFF"
    IN_FLIGHT = "IN_FLIGHT"
    LANDING = "LANDING"
    LANDED = "LANDED"
    FAILSAFE = "FAILSAFE"


class DroneTelemetry(BaseModel):
    sysid: int
    status: DroneStatus
    lat: float
    lon: float
    altitude_m: float
    heading_deg: float
    ground_speed_mps: float
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    battery_pct: float = Field(..., ge=0, le=100)
    link_quality_pct: float = Field(..., ge=0, le=100)
    active_faults: list[FaultType] = Field(default_factory=list)
    raw_mavlink: Optional[str] = None


class SwarmTelemetryBatch(BaseModel):
    tick: int
    timestamp: float
    drones: list[DroneTelemetry] = Field(default_factory=list)
