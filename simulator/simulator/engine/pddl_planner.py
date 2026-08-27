"""ENHSP wrapper - runs the Expressive Numeric Heuristic Search Planner
against a PDDL domain + problem and parses the action sequence it finds.

    domain.pddl + problem.pddl -> ENHSP (subprocess) -> ordered PlanStep list

ENHSP prints its plan to stdout as

    0.0: (check-distance drone1 source waypoint1)
    1.0: (travel drone1 source waypoint1)
    ...

surrounded by search-log noise (grounding stats, heuristic values, timing).
Parsing anchors on that one line shape and ignores everything else, so it is
unaffected by which heuristic/search ENHSP used or how verbose it was.
"""
from __future__ import annotations

import os
import re
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

ENHSP_ROOT = Path(__file__).resolve().parent.parent / "tools" / "enhsp"
DEFAULT_JAR = ENHSP_ROOT / "enhsp-dist" / "enhsp.jar"

_PLAN_LINE = re.compile(r"^\s*(?:[\d.]+\s*:\s*)?\(([^)]+)\)\s*$")


class PlannerError(RuntimeError):
    """ENHSP could not be run at all (missing jar, missing Java, timeout)."""


class PlanNotFound(RuntimeError):
    """ENHSP ran, but the problem has no plan (infeasible route/battery)."""

    def __init__(self, message: str, stdout: str = ""):
        super().__init__(message)
        self.stdout = stdout


@dataclass(frozen=True)
class PlanStep:
    index: float
    action: str
    args: list[str]

    def __str__(self) -> str:
        return f"{self.index:g}: ({self.action} {' '.join(self.args)})"


def find_enhsp_jar() -> Path:
    """The built ENHSP jar - overridable with the ENHSP_JAR env var."""
    override = os.environ.get("ENHSP_JAR")
    if override:
        path = Path(override)
        if not path.is_file():
            raise PlannerError(f"ENHSP_JAR is set but no jar exists at {path}")
        return path
    if not DEFAULT_JAR.is_file():
        raise PlannerError(
            f"ENHSP jar not found at {DEFAULT_JAR}. Build it with "
            f"scripts/setup_enhsp.ps1, or set the ENHSP_JAR environment "
            f"variable to point at an existing enhsp.jar."
        )
    return DEFAULT_JAR


def _find_java() -> str:
    """`java` on PATH, falling back to the Temurin install location - a
    winget install does not refresh PATH for processes already running."""
    import shutil

    on_path = shutil.which("java")
    if on_path:
        return on_path
    for base in (
        Path(os.environ.get("ProgramFiles", r"C:\Program Files")) / "Eclipse Adoptium",
        Path(os.environ.get("ProgramFiles", r"C:\Program Files")) / "Java",
    ):
        if not base.is_dir():
            continue
        for candidate in sorted(base.glob("jdk-*"), reverse=True):
            exe = candidate / "bin" / "java.exe"
            if exe.is_file():
                return str(exe)
    raise PlannerError(
        "Could not find a `java` executable. Install a JDK (15+) and ensure "
        "it is on PATH, e.g. via scripts/setup_enhsp.ps1."
    )


def parse_plan(stdout: str) -> list[PlanStep]:
    """Pull the ordered action lines out of ENHSP's console output."""
    steps: list[PlanStep] = []
    for line in stdout.splitlines():
        match = _PLAN_LINE.match(line)
        if not match:
            continue
        # Lines like "Plan-Length:9" or "h(I):25.0" also end without a
        # trailing paren mismatch but never start with a real "(" body -
        # the regex above only matches a full "(...)" or "N: (...)" line,
        # so log lines are already excluded by construction.
        index_text = line.split(":", 1)[0].strip()
        try:
            index = float(index_text)
        except ValueError:
            index = float(len(steps))
        body = match.group(1).split()
        if not body:
            continue
        steps.append(PlanStep(index=index, action=body[0], args=body[1:]))
    return steps


def run_enhsp(
    domain: Path,
    problem: Path,
    *,
    timeout_s: float = 30.0,
    extra_args: Optional[list[str]] = None,
) -> tuple[list[PlanStep], str]:
    """Run ENHSP on `domain` + `problem`; return (parsed steps, raw stdout).

    Raises `PlannerError` if ENHSP could not be run, `PlanNotFound` if it ran
    but reported no plan for the problem.
    """
    jar = find_enhsp_jar()
    java = _find_java()
    args = [java, "-jar", str(jar), "-o", str(domain), "-f", str(problem)]
    if extra_args:
        args.extend(extra_args)

    try:
        result = subprocess.run(
            args, capture_output=True, text=True, timeout=timeout_s
        )
    except subprocess.TimeoutExpired as exc:
        raise PlannerError(f"ENHSP timed out after {timeout_s:.0f}s") from exc
    except OSError as exc:
        raise PlannerError(f"Could not launch ENHSP: {exc}") from exc

    stdout = result.stdout or ""
    steps = parse_plan(stdout)
    if not steps:
        detail = (result.stderr or stdout or "no output").strip()[-2000:]
        raise PlanNotFound(f"ENHSP found no plan for {problem.name}:\n{detail}", stdout=stdout)
    return steps, stdout


def extract_route(steps: list[PlanStep], drone: str) -> list[str]:
    """The ordered chain of location names a `travel` sequence visits.

    `travel` actions are `(travel <drone> <from> <to>)`; consecutive legs for
    the same drone share a from/to boundary, so the route is just the first
    leg's `from` followed by every leg's `to`, in plan order.
    """
    legs = [
        step.args[1:3]
        for step in sorted(steps, key=lambda s: s.index)
        if step.action == "travel" and step.args and step.args[0] == drone
    ]
    if not legs:
        return []
    route = [legs[0][0]]
    route.extend(to for _from, to in legs)
    return route
