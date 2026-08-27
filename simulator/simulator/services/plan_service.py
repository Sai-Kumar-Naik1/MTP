"""GUI-facing PDDL mission planning.

    plans/<name>/problem.pddl (template)
        -> substitute the picked source/destination
        -> ENHSP (engine.pddl_planner)
        -> the ordered waypoint route a drone should fly

Runs on a worker thread - `run_enhsp` shells out to Java and blocks - and
reports back through Qt signals, the same shape as `MavlinkSwarmBackend`'s
worker in `services/mavlink_backend.py`.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

from typing import Optional

from PySide6.QtCore import QObject, QThread, Signal, Slot

from contracts.gui_orchestration import LatLon
from engine.pddl_planner import PlanNotFound, PlannerError, PlanStep, extract_route, run_enhsp
from engine.pddl_problem import LatLon as PddlLatLon
from engine.pddl_problem import read_locations, retarget_problem

PLANS_DIR = Path(__file__).resolve().parent.parent / "plans"
PLAN_DRONE_OBJECT = "drone1"  # ENHSP solves the route for this reference drone


def list_plans(plans_dir: Path = PLANS_DIR) -> list[str]:
    """Names of plan folders that have both a domain and a problem file."""
    if not plans_dir.is_dir():
        return []
    return sorted(
        p.name
        for p in plans_dir.iterdir()
        if p.is_dir() and (p / "domain.pddl").is_file() and (p / "problem.pddl").is_file()
    )


@dataclass
class PlanRunResult:
    plan_name: str
    steps: list[PlanStep]
    waypoints: list[LatLon]
    location_names: list[str]
    run_dir: Path


class _Worker(QObject):
    finished = Signal(object)  # PlanRunResult
    failed = Signal(str)

    @Slot(str, object, object, object)
    def run(self, plan_name: str, source: object, destination: object, no_fly_zone: object) -> None:
        try:
            result = self._run(plan_name, source, destination, no_fly_zone)
        except PlanNotFound as exc:
            self._save_stdout(plan_name, exc.stdout)
            self.failed.emit(str(exc))
        except PlannerError as exc:
            self.failed.emit(str(exc))
        except OSError as exc:
            self.failed.emit(f"Could not read/write plan files: {exc}")
        else:
            self.finished.emit(result)

    def _save_stdout(self, plan_name: str, stdout: str) -> None:
        """Best-effort: persist ENHSP's raw output for inspection even when it
        found no plan. `_last_run_dir` is set by `_run` before ENHSP is
        invoked, so it is available here regardless of how `_run` failed."""
        run_dir = getattr(self, "_last_run_dir", None)
        if run_dir is None or not stdout:
            return
        try:
            (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")
        except OSError:
            pass

    def _run(
        self,
        plan_name: str,
        source: LatLon,
        destination: LatLon,
        no_fly_zone: Optional[list[LatLon]],
    ) -> PlanRunResult:
        plan_dir = PLANS_DIR / plan_name
        domain_path = plan_dir / "domain.pddl"
        template_path = plan_dir / "problem.pddl"
        if not domain_path.is_file() or not template_path.is_file():
            raise PlannerError(f"Plan '{plan_name}' is missing domain.pddl/problem.pddl")

        template_text = template_path.read_text(encoding="utf-8")
        concrete_text = retarget_problem(
            template_text,
            PddlLatLon(lat=source.lat, lon=source.lon),
            PddlLatLon(lat=destination.lat, lon=destination.lon),
            no_fly_zone=(
                [PddlLatLon(lat=p.lat, lon=p.lon) for p in no_fly_zone] if no_fly_zone else None
            ),
        )

        run_dir = plan_dir / "runs" / time.strftime("%Y-%m-%d_%H%M%S")
        run_dir.mkdir(parents=True, exist_ok=True)
        self._last_run_dir = run_dir
        problem_path = run_dir / "problem.pddl"
        problem_path.write_text(concrete_text, encoding="utf-8")

        steps, stdout = run_enhsp(domain_path, problem_path)
        (run_dir / "plan.txt").write_text(stdout, encoding="utf-8")

        location_names = extract_route(steps, PLAN_DRONE_OBJECT)
        if not location_names:
            raise PlanNotFound(
                f"ENHSP found a plan but it contains no '{PLAN_DRONE_OBJECT}' travel legs."
            )
        locations = read_locations(concrete_text)
        try:
            waypoints = [
                LatLon(lat=locations[name].lat, lon=locations[name].lon)
                for name in location_names
            ]
        except KeyError as exc:
            raise PlanNotFound(
                f"Plan references location {exc} with no coordinates in problem.pddl"
            ) from exc

        return PlanRunResult(
            plan_name=plan_name,
            steps=steps,
            waypoints=waypoints,
            location_names=location_names,
            run_dir=run_dir,
        )


class PlanService(QObject):
    """GUI-facing handle: call `run_async`, get `finished`/`failed`."""

    finished = Signal(object)  # PlanRunResult
    failed = Signal(str)

    _run_requested = Signal(str, object, object, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._thread = QThread(self)
        self._worker = _Worker()
        self._worker.moveToThread(self._thread)
        self._worker.finished.connect(self.finished)
        self._worker.failed.connect(self.failed)
        self._run_requested.connect(self._worker.run)
        self._thread.start()

    def run_async(
        self,
        plan_name: str,
        source: LatLon,
        destination: LatLon,
        no_fly_zone: Optional[list[LatLon]] = None,
    ) -> None:
        self._run_requested.emit(plan_name, source, destination, no_fly_zone)

    def shutdown(self) -> None:
        self._thread.quit()
        self._thread.wait(2000)
