"""Load ``config/physics.yaml`` and ``config/visuals.yaml``.

Both files are optional and every key in them is optional: anything missing
falls back to the defaults written in ``DEFAULT_PHYSICS`` / ``DEFAULT_VISUALS``
below (the same values as the commented files in ``config/``). Unknown keys
are rejected so a typo never passes silently.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_PHYSICS_PATH = PROJECT_ROOT / "config" / "physics.yaml"
DEFAULT_VISUALS_PATH = PROJECT_ROOT / "config" / "visuals.yaml"

SEGMENT_KINDS = ("stem", "branch", "petiole", "leaf_rachis", "truss_rachis", "pedicel")

_VEGETATIVE = {
    "joint": "d6",
    "young_modulus": 50.64e6,
    "damping_ratio": 0.8,
    "density": 1000.0,
    "bend_limit_deg": 30.0,
    "drive_stiffness_scale": 1.0,
}

DEFAULT_PHYSICS: dict[str, Any] = {
    "global_scale": 2.0,
    "simulation": {
        "physics_hz": 480,
        "interactive_physics_hz": 60,
        "solver_position_iterations": 32,
        "solver_velocity_iterations": 4,
        "fruit_solver_position_iterations": 32,
        "fruit_solver_velocity_iterations": 1,
        "gpu_dynamics": True,
    },
    "joints": {
        "joint_target": 220,
        "joint_max": 230,
        "optimize": "none",
        "merge_stiffness": "blend",
    },
    "segments": {
        "stem": {**_VEGETATIVE, "joint": "fixed"},
        "branch": dict(_VEGETATIVE),
        "petiole": {**_VEGETATIVE, "damping_ratio": 0.3},
        "leaf_rachis": {**_VEGETATIVE, "damping_ratio": 0.3},
        "truss_rachis": {**_VEGETATIVE, "young_modulus": 20.0e9, "damping_ratio": 4.0, "density": 2000.0},
        "pedicel": {
            **_VEGETATIVE,
            "young_modulus": 4.0e9,
            "damping_ratio": 4.0,
            "density": 2000.0,
            "bend_limit_deg": 25.0,
            "drive_stiffness_scale": 0.2,
        },
    },
    "fruits": {
        "mode": "visual",
        "detachable": True,
        "break_force_n": 6.0,
        "density": 1000.0,
    },
    "collisions": {
        "initial_overlaps": "filter",
        "branches": True,
    },
}

DEFAULT_VISUALS: dict[str, Any] = {
    "quality": "realistic",
    "pose": "v2-aesthetic",
    "stems": {
        "generator": "organic_segmented",
        "junction_bulge": True,
        "terminal_forks": True,
        "color": [0.075, 0.16, 0.035],
    },
    "leaves": {
        "generator": "baked",
        "seed": 42,
        "one_per_leaflet": True,
        "baked_set": None,
        "droop_deg": 22.0,
        "droop_jitter_deg": 10.0,
        "roll_jitter_deg": 12.0,
    },
    "pedicels": {
        "gravity_curve": True,
    },
    "trusses": {
        # Visual only: fraction of the skeleton radius (x global_scale) used to
        # draw truss stalks; colliders keep the full radius.
        "rachis_thickness": 0.5,
        "pedicel_thickness": 0.4,
        # Pedicels this many times longer than in the skeleton, so tomatoes
        # hang below the rachis; the tomato and its collider move with them.
        "pedicel_length_scale": 1.75,
    },
    "fruits": {
        "generator": "sphere",
        # Visual only: the example radii x global_scale look too big next to the
        # plant, so the default draws them at half size with a calyx.
        "size_scale": 0.5,
        "calyx": True,
    },
}

_PHYSICS_CHOICES = {
    ("joints", "optimize"): ("none", "lock", "full"),
    ("joints", "merge_stiffness"): ("load", "tip", "blend", "series", "keep"),
    ("fruits", "mode"): ("none", "visual", "physical"),
    ("collisions", "initial_overlaps"): ("filter", "error"),
    ("simulation", "physics_hz"): (60, 120, 240, 480, 960),
    ("simulation", "interactive_physics_hz"): (60, 120, 240, 480, 960),
}
_VISUALS_CHOICES = {
    ("quality",): ("realistic", "performance"),
    ("pose",): ("v2-aesthetic", "canonical"),
    ("stems", "generator"): ("organic_segmented",),
    ("leaves", "generator"): ("baked", "simple"),
    ("fruits", "generator"): ("sphere",),
}
_SEGMENT_CHOICES = {"joint": ("d6", "fixed")}


class ConfigError(ValueError):
    pass


def _merge(defaults: Mapping[str, Any], override: Mapping[str, Any], where: str) -> dict[str, Any]:
    if not isinstance(override, Mapping):
        raise ConfigError(f"{where or 'file'} must be a mapping")
    result = copy.deepcopy(dict(defaults))
    for key, value in override.items():
        path = f"{where}.{key}" if where else str(key)
        if key not in defaults:
            raise ConfigError(f"unknown setting {path!r}; allowed: {sorted(defaults)}")
        if isinstance(defaults[key], Mapping):
            result[key] = _merge(defaults[key], value or {}, path)
        else:
            result[key] = value
    return result


def _coerce_numbers(values: dict[str, Any], defaults: Mapping[str, Any], where: str) -> None:
    """Accept numbers YAML read as text (PyYAML reads ``30e6`` as a string)."""

    for key, default in defaults.items():
        path = f"{where}.{key}"
        value = values[key]
        if key == "global_scale" and value == "auto":
            continue
        if isinstance(default, Mapping):
            _coerce_numbers(value, default, path)
        elif isinstance(default, (int, float)) and not isinstance(default, bool):
            if isinstance(value, str):
                try:
                    value = float(value)
                except ValueError:
                    raise ConfigError(f"{path} must be a number, got {value!r}") from None
                values[key] = int(value) if isinstance(default, int) and value.is_integer() else value
            elif isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ConfigError(f"{path} must be a number, got {value!r}")


def _check_choices(values: Mapping[str, Any], choices: Mapping[tuple, tuple], name: str) -> None:
    for path, allowed in choices.items():
        value: Any = values
        for part in path:
            value = value[part]
        if value not in allowed:
            raise ConfigError(f"{name}: {'.'.join(path)} must be one of {list(allowed)}, got {value!r}")


def _read(path: Path | None) -> Mapping[str, Any]:
    if path is None or not Path(path).exists():
        return {}
    try:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigError(f"{path} is not valid YAML: {exc}") from exc
    return data or {}


@dataclass(frozen=True)
class PhysicsConfig:
    values: dict[str, Any]
    source: str

    @property
    def global_scale(self) -> float | str:
        return self.values["global_scale"]

    @property
    def simulation(self) -> dict[str, Any]:
        return self.values["simulation"]

    @property
    def joints(self) -> dict[str, Any]:
        return self.values["joints"]

    @property
    def fruits(self) -> dict[str, Any]:
        return self.values["fruits"]

    @property
    def collisions(self) -> dict[str, Any]:
        return self.values["collisions"]

    def segment(self, kind: str) -> dict[str, Any]:
        return self.values["segments"][kind]


@dataclass(frozen=True)
class VisualsConfig:
    values: dict[str, Any]
    source: str

    @property
    def quality(self) -> str:
        return self.values["quality"]

    @property
    def pose(self) -> str:
        return self.values["pose"]

    @property
    def stems(self) -> dict[str, Any]:
        return self.values["stems"]

    @property
    def leaves(self) -> dict[str, Any]:
        return self.values["leaves"]

    @property
    def pedicels(self) -> dict[str, Any]:
        return self.values["pedicels"]

    @property
    def trusses(self) -> dict[str, Any]:
        return self.values["trusses"]

    @property
    def fruits(self) -> dict[str, Any]:
        return self.values["fruits"]


def physics_config_from_dict(data: Mapping[str, Any] | None = None, source: str = "dict") -> PhysicsConfig:
    """Defaults overridden by ``data`` (same keys as config/physics.yaml)."""

    values = _merge(DEFAULT_PHYSICS, data or {}, "")
    _coerce_numbers(values, DEFAULT_PHYSICS, "physics")
    _check_choices(values, _PHYSICS_CHOICES, "physics")
    scale = values["global_scale"]
    if scale != "auto" and (isinstance(scale, bool) or not isinstance(scale, (int, float)) or scale <= 0):
        raise ConfigError("physics: global_scale must be a positive number or 'auto'")
    for kind in SEGMENT_KINDS:
        segment = values["segments"][kind]
        for key, allowed in _SEGMENT_CHOICES.items():
            if segment[key] not in allowed:
                raise ConfigError(f"physics: segments.{kind}.{key} must be one of {list(allowed)}")
        for key in ("young_modulus", "damping_ratio", "density", "bend_limit_deg", "drive_stiffness_scale"):
            if float(segment[key]) <= 0.0:
                raise ConfigError(f"physics: segments.{kind}.{key} must be positive")
    if not isinstance(values["collisions"]["branches"], bool):
        raise ConfigError("physics: collisions.branches must be true or false")
    if values["joints"]["joint_target"] > values["joints"]["joint_max"]:
        raise ConfigError("physics: joints.joint_target cannot exceed joints.joint_max")
    return PhysicsConfig(values, source)


def load_physics_config(path: str | Path | None = DEFAULT_PHYSICS_PATH) -> PhysicsConfig:
    """Read config/physics.yaml; a missing file means all defaults."""

    exists = path is not None and Path(path).exists()
    return physics_config_from_dict(_read(Path(path) if path else None), str(path) if exists else "defaults")


def visuals_config_from_dict(data: Mapping[str, Any] | None = None, source: str = "dict") -> VisualsConfig:
    """Defaults overridden by ``data`` (same keys as config/visuals.yaml)."""

    values = _merge(DEFAULT_VISUALS, data or {}, "")
    _check_choices(values, _VISUALS_CHOICES, "visuals")
    if type(values["leaves"]["seed"]) is not int or values["leaves"]["seed"] < 0:
        raise ConfigError("visuals: leaves.seed must be a non-negative integer")
    size_scale = values["fruits"]["size_scale"]
    if isinstance(size_scale, str):
        try:
            size_scale = values["fruits"]["size_scale"] = float(size_scale)
        except ValueError:
            pass
    if isinstance(size_scale, bool) or not isinstance(size_scale, (int, float)) or size_scale <= 0:
        raise ConfigError(f"visuals: fruits.size_scale must be a positive number, got {size_scale!r}")
    for key in ("rachis_thickness", "pedicel_thickness"):
        value = values["trusses"][key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not 0 < value <= 1:
            raise ConfigError(f"visuals: trusses.{key} must be a number in (0, 1], got {value!r}")
    length_scale = values["trusses"]["pedicel_length_scale"]
    if isinstance(length_scale, bool) or not isinstance(length_scale, (int, float)) or not 0.5 <= length_scale <= 3:
        raise ConfigError(f"visuals: trusses.pedicel_length_scale must be a number in [0.5, 3], got {length_scale!r}")
    for path in (("fruits", "calyx"), ("leaves", "one_per_leaflet")):
        if not isinstance(values[path[0]][path[1]], bool):
            raise ConfigError(f"visuals: {'.'.join(path)} must be true or false")
    leaves = values["leaves"]
    for key in ("droop_deg", "droop_jitter_deg", "roll_jitter_deg"):
        try:
            leaves[key] = float(leaves[key])
        except (TypeError, ValueError):
            raise ConfigError(f"visuals: leaves.{key} must be a number, got {leaves[key]!r}") from None
    if leaves["baked_set"] is not None and not isinstance(leaves["baked_set"], str):
        raise ConfigError("visuals: leaves.baked_set must be a folder path or null")
    color = values["stems"]["color"]
    if color is not None:
        try:
            color = values["stems"]["color"] = [float(c) for c in color]
        except (TypeError, ValueError):
            color = None
        if color is None or len(color) != 3 or not all(0.0 <= c <= 1.0 for c in color):
            raise ConfigError("visuals: stems.color must be [r, g, b] with values in 0..1, or null")
    return VisualsConfig(values, source)


def load_visuals_config(path: str | Path | None = DEFAULT_VISUALS_PATH) -> VisualsConfig:
    """Read config/visuals.yaml; a missing file means all defaults."""

    exists = path is not None and Path(path).exists()
    return visuals_config_from_dict(_read(Path(path) if path else None), str(path) if exists else "defaults")
