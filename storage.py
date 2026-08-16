"""Local JSON persistence for drone configuration profiles and swarm presets
(SRS §3.2.1, §10). Profiles are reloaded automatically on next launch.
"""
from __future__ import annotations

from pathlib import Path

from contracts.gui_orchestration import DroneConfig, SwarmPreset

DATA_DIR = Path(__file__).resolve().parent.parent / "data"
PROFILES_DIR = DATA_DIR / "profiles"
PRESETS_DIR = DATA_DIR / "presets"


def _slugify(name: str) -> str:
    safe = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in name.strip())
    return safe or "unnamed"


class ProfileStore:
    """Reads/writes drone profiles and swarm presets as one JSON file each."""

    def __init__(self, profiles_dir: Path = PROFILES_DIR, presets_dir: Path = PRESETS_DIR):
        self.profiles_dir = profiles_dir
        self.presets_dir = presets_dir
        self.profiles_dir.mkdir(parents=True, exist_ok=True)
        self.presets_dir.mkdir(parents=True, exist_ok=True)

    # ---- Drone profiles ----

    def list_profiles(self) -> list[DroneConfig]:
        profiles = []
        for path in sorted(self.profiles_dir.glob("*.json")):
            profiles.append(DroneConfig.model_validate_json(path.read_text(encoding="utf-8")))
        return profiles

    def save_profile(self, drone: DroneConfig, previous_name: str | None = None) -> None:
        if previous_name and previous_name != drone.name:
            self.delete_profile(previous_name)
        path = self.profiles_dir / f"{_slugify(drone.name)}.json"
        path.write_text(drone.model_dump_json(indent=2), encoding="utf-8")

    def delete_profile(self, name: str) -> None:
        path = self.profiles_dir / f"{_slugify(name)}.json"
        if path.exists():
            path.unlink()

    # ---- Swarm presets ----

    def list_presets(self) -> list[SwarmPreset]:
        presets = []
        for path in sorted(self.presets_dir.glob("*.json")):
            presets.append(SwarmPreset.model_validate_json(path.read_text(encoding="utf-8")))
        return presets

    def save_preset(self, preset: SwarmPreset) -> None:
        path = self.presets_dir / f"{_slugify(preset.name)}.json"
        path.write_text(preset.model_dump_json(indent=2), encoding="utf-8")

    def delete_preset(self, name: str) -> None:
        path = self.presets_dir / f"{_slugify(name)}.json"
        if path.exists():
            path.unlink()
