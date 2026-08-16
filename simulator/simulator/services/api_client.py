"""REST + WebSocket client for the Module 1 <-> Module 2 contract (SRS §4.2, §6.1).

Module 1 depends only on Module 2's documented REST/WebSocket contract (§3.3).
This client performs REST commands on a background thread pool (so a slow or
absent Module 2 never blocks the GUI event loop) and subscribes to the
`/telemetry` WebSocket stream, re-emitting parsed `SwarmTelemetryBatch`
objects as a Qt signal.
"""
from __future__ import annotations

from typing import Optional

import requests
from PySide6.QtCore import QObject, QRunnable, QThreadPool, QUrl, Signal, Slot
from PySide6.QtWebSockets import QWebSocket

from contracts.gui_orchestration import (
    DroneConfig,
    FlockCommand,
    InjectFault,
    LatLon,
    SwarmTelemetryBatch,
)

DEFAULT_BASE_URL = "http://127.0.0.1:8000"
DEFAULT_WS_URL = "ws://127.0.0.1:8000/telemetry"
REQUEST_TIMEOUT_S = 2.0


class _RequestSignals(QObject):
    succeeded = Signal(str, object)
    failed = Signal(str, str)


class _RequestJob(QRunnable):
    """A single REST call, run off the GUI thread."""

    def __init__(self, tag: str, method: str, url: str, json_body: Optional[dict], signals: _RequestSignals):
        super().__init__()
        self.tag = tag
        self.method = method
        self.url = url
        self.json_body = json_body
        self.signals = signals

    @Slot()
    def run(self) -> None:
        try:
            response = requests.request(
                self.method, self.url, json=self.json_body, timeout=REQUEST_TIMEOUT_S
            )
            response.raise_for_status()
            data = response.json() if response.content else None
            self.signals.succeeded.emit(self.tag, data)
        except Exception as exc:  # noqa: BLE001 - surfaced to the GUI as a status message
            self.signals.failed.emit(self.tag, str(exc))


class OrchestrationClient(QObject):
    """Talks to Module 2 (Orchestration & Swarm Router)."""

    request_succeeded = Signal(str, object)
    request_failed = Signal(str, str)
    telemetry_received = Signal(object)  # SwarmTelemetryBatch
    connection_state_changed = Signal(bool)

    def __init__(self, base_url: str = DEFAULT_BASE_URL, ws_url: str = DEFAULT_WS_URL, parent=None):
        super().__init__(parent)
        self.base_url = base_url.rstrip("/")
        self.ws_url = ws_url
        self._pool = QThreadPool.globalInstance()
        self._signals = _RequestSignals()
        self._signals.succeeded.connect(self.request_succeeded)
        self._signals.failed.connect(self.request_failed)

        self._socket = QWebSocket()
        self._socket.connected.connect(lambda: self.connection_state_changed.emit(True))
        self._socket.disconnected.connect(lambda: self.connection_state_changed.emit(False))
        self._socket.textMessageReceived.connect(self._on_telemetry_message)
        self._socket.errorOccurred.connect(lambda *_: self.connection_state_changed.emit(False))

    def set_endpoints(self, base_url: str, ws_url: str) -> None:
        self.base_url = base_url.rstrip("/")
        self.ws_url = ws_url

    # ---- Telemetry stream (WS /telemetry) ----

    def connect_telemetry(self) -> None:
        self._socket.open(QUrl(self.ws_url))

    def disconnect_telemetry(self) -> None:
        self._socket.close()

    def _on_telemetry_message(self, message: str) -> None:
        try:
            batch = SwarmTelemetryBatch.model_validate_json(message)
        except Exception:
            return
        self.telemetry_received.emit(batch)

    # ---- Commands ----

    def _submit(self, tag: str, method: str, path: str, body: Optional[dict] = None) -> None:
        job = _RequestJob(tag, method, f"{self.base_url}{path}", body, self._signals)
        self._pool.start(job)

    def register_drone(self, drone: DroneConfig) -> None:
        """POST /drones"""
        self._submit("register_drone", "POST", "/drones", {"drone": drone.model_dump(mode="json")})

    def refresh_registry(self) -> None:
        """GET /drones"""
        self._submit("registry", "GET", "/drones")

    def takeoff(self, sysid: int, target_altitude_m: float) -> None:
        """POST /drones/{sysid}/takeoff"""
        self._submit(
            f"takeoff:{sysid}",
            "POST",
            f"/drones/{sysid}/takeoff",
            {"sysid": sysid, "target_altitude_m": target_altitude_m},
        )

    def goto(self, sysid: int, destination: LatLon, altitude_m: Optional[float] = None) -> None:
        """POST /drones/{sysid}/goto"""
        self._submit(
            f"goto:{sysid}",
            "POST",
            f"/drones/{sysid}/goto",
            {"sysid": sysid, "destination": destination.model_dump(), "altitude_m": altitude_m},
        )

    def flock(self, command: FlockCommand) -> None:
        """POST /swarm/flock"""
        self._submit("flock", "POST", "/swarm/flock", command.model_dump(mode="json"))

    def inject_fault(self, command: InjectFault) -> None:
        """POST /drones/{sysid}/inject_fault, fanned out per targeted sysid (SRS §9)."""
        for sysid in command.sysids:
            payload = command.model_dump(mode="json")
            payload["sysids"] = [sysid]
            self._submit(f"inject_fault:{sysid}", "POST", f"/drones/{sysid}/inject_fault", payload)

    def stop_swarm(self, reason: Optional[str] = None) -> None:
        """Teardown request unlocking configuration inputs (SRS §3.2.1)."""
        self._submit("stop_swarm", "POST", "/swarm/stop", {"reason": reason})
