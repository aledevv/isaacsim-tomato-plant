"""
Quantitative cantilever validation for exporterV2 plant physics.

This script deliberately separates two claims:
  1. pre/post behavior: legacy_current vs new_physics under identical inputs
  2. physical realism: measured deflection vs a declared beam-theory benchmark

Pure formula/audit checks can run with:
    uv run python src/exporterV2/tests/cantilever_bending_experiment/cantilever_validation.py formula-check

Generation and simulation must run with Isaac Sim:
    ~/isaacsim/python.sh src/exporterV2/tests/cantilever_bending_experiment/cantilever_validation.py all
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[3]
SRC_DIR = REPO_ROOT / "src"
DATA_DIR = REPO_ROOT / "data" / "usd_models" / "physics_tests"
RESULTS_DIR = SCRIPT_DIR / "results"
DOCS_DIR = SCRIPT_DIR / "docs"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

GRAVITY = 9.81
DEG_GAIN = math.pi / 180.0
DEFAULT_N_LINKS = (3, 5, 10, 15, 20)
DEFAULT_MODELS = ("legacy_current", "new_physics")
DEFAULT_SUPPORTS = ("fixed",)
DEFAULT_JOINT_MODELS = ("d6_biaxial",)
VALID_SUPPORTS = {"fixed", "half_cell"}
VALID_JOINT_MODELS = {"d6_biaxial", "d6_planar", "revolute_planar", "fixed_chain"}
VALID_FORCE_POINTS = {"geometric_tip", "com"}
DEFAULT_SOLVER_POSITION_ITERATIONS = 32
DEFAULT_SOLVER_VELOCITY_ITERATIONS = 4


@dataclass(frozen=True)
class Benchmark:
    name: str
    world_length_m: float
    outer_radius_m: float
    inner_radius_m: float
    young_modulus_pa: float
    density_kg_m3: float
    reference: str

    @property
    def area_m2(self) -> float:
        return math.pi * (self.outer_radius_m**2 - self.inner_radius_m**2)

    @property
    def second_moment_m4(self) -> float:
        return math.pi * (self.outer_radius_m**4 - self.inner_radius_m**4) / 4.0

    @property
    def flexural_rigidity_nm2(self) -> float:
        return self.young_modulus_pa * self.second_moment_m4

    @property
    def distributed_weight_npm(self) -> float:
        return self.density_kg_m3 * self.area_m2 * GRAVITY

    @property
    def expected_self_weight_mm(self) -> float:
        return (
            self.distributed_weight_npm
            * self.world_length_m**4
            / (8.0 * self.flexural_rigidity_nm2)
            * 1000.0
        )

    def expected_tip_force_mm(self, force_n: float) -> float:
        return force_n * self.world_length_m**3 / (3.0 * self.flexural_rigidity_nm2) * 1000.0

    def _discrete_hinges(
        self,
        n_links: int,
        support: str,
        joint_model: str,
    ) -> list[tuple[float, float]]:
        """Return ``(x, stiffness_rad)`` for the exact linearized chain topology."""
        if joint_model == "fixed_chain":
            return []
        link_length = self.world_length_m / n_links
        k_rad = self.flexural_rigidity_nm2 / link_length
        hinges = [(index * link_length, k_rad) for index in range(1, n_links)]
        if support == "half_cell":
            hinges.insert(0, (0.0, 2.0 * k_rad))
        elif support != "fixed":
            raise ValueError(f"Unknown support: {support}")
        return hinges

    def expected_discrete_self_weight_mm(
        self,
        n_links: int,
        support: str = "fixed",
        joint_model: str = "d6_biaxial",
    ) -> float:
        """Small-angle reference for the generated rigid-link chain under gravity."""
        link_length = self.world_length_m / n_links
        delta_m = 0.0
        for hinge_x, k_rad in self._discrete_hinges(n_links, support, joint_model):
            moment = 0.0
            for link_index in range(n_links):
                load_x = (link_index + 0.5) * link_length
                if load_x > hinge_x:
                    moment += self.distributed_weight_npm * link_length * (load_x - hinge_x)
            theta = moment / k_rad
            delta_m += theta * (self.world_length_m - hinge_x)
        return delta_m * 1000.0

    def expected_discrete_tip_force_mm(
        self,
        n_links: int,
        force_n: float,
        support: str = "fixed",
        force_point: str = "geometric_tip",
        joint_model: str = "d6_biaxial",
    ) -> float:
        """Small-angle tip displacement for the exact load and support topology."""
        link_length = self.world_length_m / n_links
        if force_point == "geometric_tip":
            load_x = self.world_length_m
        elif force_point == "com":
            load_x = self.world_length_m - 0.5 * link_length
        else:
            raise ValueError(f"Unknown force point: {force_point}")
        delta_m = 0.0
        for hinge_x, k_rad in self._discrete_hinges(n_links, support, joint_model):
            if hinge_x >= load_x:
                continue
            theta = abs(force_n) * (load_x - hinge_x) / k_rad
            delta_m += theta * (self.world_length_m - hinge_x)
        return delta_m * 1000.0

    def expected_point_force_mm(self, n_links: int, force_n: float, force_point: str) -> float:
        """Euler-Bernoulli free-end displacement for a point load at ``force_point``."""
        link_length = self.world_length_m / n_links
        load_x = self.world_length_m if force_point == "geometric_tip" else self.world_length_m - 0.5 * link_length
        return (
            abs(force_n)
            * load_x**2
            * (3.0 * self.world_length_m - load_x)
            / (6.0 * self.flexural_rigidity_nm2)
            * 1000.0
        )

    def expected_internal_stiffness_usd(self, n_links: int) -> float:
        segment_length = self.world_length_m / n_links
        return self.flexural_rigidity_nm2 / segment_length * DEG_GAIN

    def expected_base_stiffness_usd(self, n_links: int) -> float:
        segment_length = self.world_length_m / n_links
        return 2.0 * self.flexural_rigidity_nm2 / segment_length * DEG_GAIN


BENCHMARKS: dict[str, Benchmark] = {
    "synthetic_solid_40cm": Benchmark(
        name="synthetic_solid_40cm",
        world_length_m=0.400,
        outer_radius_m=0.010,
        inner_radius_m=0.0,
        young_modulus_pa=100.0e6,
        density_kg_m3=1000.0,
        reference="continuity benchmark from the earlier solid-cylinder test",
    ),
    "tomato_gao_20cm": Benchmark(
        name="tomato_gao_20cm",
        world_length_m=0.200,
        outer_radius_m=0.0111 / 2.0,
        inner_radius_m=0.00382 / 2.0,
        young_modulus_pa=50.64e6,
        density_kg_m3=769.96,
        reference="Gao et al. 2024 harvested tomato stalk, hollow circular section",
    ),
}


@dataclass(frozen=True)
class Scenario:
    name: str
    gravity_mps2: float
    tip_force_n: float

    def expected_mm(self, benchmark: Benchmark, n_links: int, force_point: str) -> float:
        if self.name == "self_weight":
            return benchmark.expected_self_weight_mm
        return benchmark.expected_point_force_mm(n_links, self.tip_force_n, force_point)

    def expected_discrete_mm(
        self,
        benchmark: Benchmark,
        n_links: int,
        support: str,
        force_point: str,
        joint_model: str,
    ) -> float:
        if self.name == "self_weight":
            return benchmark.expected_discrete_self_weight_mm(n_links, support, joint_model)
        return benchmark.expected_discrete_tip_force_mm(
            n_links,
            self.tip_force_n,
            support,
            force_point,
            joint_model,
        )


SCENARIOS: dict[str, Scenario] = {
    "self_weight": Scenario(name="self_weight", gravity_mps2=GRAVITY, tip_force_n=0.0),
    "tip_force_0p05N": Scenario(name="tip_force_0p05N", gravity_mps2=0.0, tip_force_n=-0.05),
}


def _split_csv(value: str | None, default: tuple[str, ...]) -> tuple[str, ...]:
    if not value:
        return default
    return tuple(part.strip() for part in value.split(",") if part.strip())


def _split_int_csv(value: str | None, default: tuple[int, ...]) -> tuple[int, ...]:
    if not value:
        return default
    return tuple(int(part.strip()) for part in value.split(",") if part.strip())


def _split_float_csv(value: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if not value:
        return default
    return tuple(float(part.strip()) for part in value.split(",") if part.strip())


def model_uses_legacy(model: str) -> bool:
    if model == "legacy_current":
        return True
    if model == "new_physics":
        return False
    raise ValueError(f"Unknown model: {model}")


def experiment_config(
    benchmark: str,
    model: str,
    n_links: int,
    support: str,
    joint_model: str,
    collisions_enabled: bool,
    backend: str,
) -> dict[str, Any]:
    return {
        "schema_version": 3,
        "benchmark": benchmark,
        "model": model,
        "n_links": n_links,
        "support": support,
        "joint_model": joint_model,
        "collisions_enabled": collisions_enabled,
        "backend": backend,
        "authored_solver_position_iterations": DEFAULT_SOLVER_POSITION_ITERATIONS,
        "authored_solver_velocity_iterations": DEFAULT_SOLVER_VELOCITY_ITERATIONS,
    }


def config_fingerprint(config: dict[str, Any]) -> str:
    encoded = json.dumps(config, sort_keys=True, separators=(",", ":")).encode("ascii")
    return hashlib.sha256(encoded).hexdigest()[:16]


def usd_path(benchmark: str, model: str, n_links: int, support: str, joint_model: str) -> Path:
    return DATA_DIR / f"cantilever_{benchmark}_{model}_{support}_{joint_model}_N{n_links}.usda"


def result_json_path() -> Path:
    return RESULTS_DIR / "cantilever_validation_results.json"


def result_csv_path() -> Path:
    return RESULTS_DIR / "cantilever_validation_measurements.csv"


def report_path() -> Path:
    return RESULTS_DIR / "cantilever_validation_report.md"


def last_run_checkpoint_path() -> Path:
    return RESULTS_DIR / "cantilever_validation_last_run.json"


def branch_defs_for_benchmark(
    benchmark: Benchmark,
    n_links: int,
    support: str,
    joint_model: str,
    collisions_enabled: bool,
) -> list[dict[str, Any]]:
    from exporterV2.core.tree_config import GLOBAL_SCALE

    if support not in VALID_SUPPORTS:
        raise ValueError(f"Unknown support: {support}")
    if joint_model not in VALID_JOINT_MODELS:
        raise ValueError(f"Unknown joint model: {joint_model}")

    link_length = benchmark.world_length_m / n_links
    branch_joint_type = {
        "d6_biaxial": "d6",
        "d6_planar": "d6_planar",
        "revolute_planar": "revolute_planar",
        "fixed_chain": "fixed",
    }[joint_model]
    cantilever = {
        "id": f"cantilever_{benchmark.name}_N{n_links}",
        "parent": "root_anchor",
        "attach_link": 1,
        "n_links": n_links,
        "radius": benchmark.outer_radius_m / GLOBAL_SCALE,
        "inner_radius": benchmark.inner_radius_m / GLOBAL_SCALE,
        "height": benchmark.world_length_m / GLOBAL_SCALE / n_links,
        "density": benchmark.density_kg_m3,
        "young_modulus": benchmark.young_modulus_pa,
        "tilt": 90.0,
        "rot": 0.0,
        "joint_type": branch_joint_type,
        "attachment_joint_type": "fixed" if support == "fixed" else branch_joint_type,
        "collision_enabled": collisions_enabled,
    }
    if support == "half_cell" and joint_model != "fixed_chain":
        cantilever["attachment_stiffness_rad"] = 2.0 * benchmark.flexural_rigidity_nm2 / link_length

    return [
        {
            "id": "root_anchor",
            "parent": None,
            "attach_link": None,
            "n_links": 1,
            "radius": 0.02,
            "height": 0.05,
            "tilt": 0.0,
            "rot": 0.0,
            "joint_type": "fixed",
            "collision_enabled": collisions_enabled,
        },
        cantilever,
    ]


def generate_usd_files(
    benchmarks: tuple[str, ...],
    models: tuple[str, ...],
    n_links_values: tuple[int, ...],
    supports: tuple[str, ...],
    joint_models: tuple[str, ...],
    collisions_enabled: bool,
    backend: str,
) -> list[dict[str, Any]]:
    from exporterV2.core.physics import apply_physx_articulation_settings, apply_physx_scene_settings
    from exporterV2.core.usd.stage import build_stage
    from pxr import Sdf

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    generated = []
    for benchmark_name in benchmarks:
        benchmark = BENCHMARKS[benchmark_name]
        for model in models:
            legacy = model_uses_legacy(model)
            for support in supports:
                for joint_model in joint_models:
                    for n_links in n_links_values:
                        path = usd_path(benchmark_name, model, n_links, support, joint_model)
                        config = experiment_config(
                            benchmark_name, model, n_links, support, joint_model, collisions_enabled, backend
                        )
                        fingerprint = config_fingerprint(config)
                        branches = branch_defs_for_benchmark(
                            benchmark, n_links, support, joint_model, collisions_enabled
                        )
                        stage, stem_path = build_stage(
                            output_path=str(path),
                            branches=branches,
                            skip_limit_check=True,
                            legacy_physics=legacy,
                        )
                        apply_physx_scene_settings(stage, enable_gpu_dynamics=backend == "gpu")
                        apply_physx_articulation_settings(stage, stem_path)
                        stem_prim = stage.GetPrimAtPath(stem_path)
                        stem_prim.CreateAttribute(
                            "cantilever:configFingerprint", Sdf.ValueTypeNames.String, custom=True
                        ).Set(fingerprint)
                        stem_prim.CreateAttribute(
                            "cantilever:configJson", Sdf.ValueTypeNames.String, custom=True
                        ).Set(json.dumps(config, sort_keys=True, separators=(",", ":")))
                        stage.GetRootLayer().Save()
                        generated.append({**config, "fingerprint": fingerprint, "usd_path": str(path)})
                        print(f"[generate] {path} fingerprint={fingerprint}")
    return generated


def _read_float(pattern: str, text: str) -> float | None:
    match = re.search(pattern, text)
    return float(match.group(1)) if match else None


def _read_all_float(pattern: str, text: str) -> list[float]:
    return [float(value) for value in re.findall(pattern, text)]


def _read_bool(pattern: str, text: str) -> bool | None:
    match = re.search(pattern, text)
    if not match:
        return None
    return match.group(1).lower() in {"1", "true"}


def audit_usd_file(
    path: Path,
    benchmark: Benchmark,
    model: str,
    n_links: int,
    support: str,
    joint_model: str,
    collisions_enabled: bool,
    backend: str,
) -> dict[str, Any]:
    text = path.read_text()
    meters_per_unit = _read_float(r"metersPerUnit = ([0-9.eE+-]+)", text)
    kilograms_per_unit = _read_float(r"kilogramsPerUnit = ([0-9.eE+-]+)", text)
    linear_scale_m = meters_per_unit if meters_per_unit is not None else 0.01
    mass_scale_kg = kilograms_per_unit if kilograms_per_unit is not None else 1.0
    prefix = f"cantilever_{benchmark.name}_N{n_links}_Link_"
    link_blocks = re.findall(
        rf'def Xform "({re.escape(prefix)}\d{{2}})".*?\n        \}}',
        text,
        flags=re.DOTALL,
    )

    masses = []
    heights = []
    radii = []
    link_block_texts = []
    for link_name in link_blocks:
        block_match = re.search(
            rf'def Xform "{re.escape(link_name)}".*?\n        \}}',
            text,
            flags=re.DOTALL,
        )
        if not block_match:
            continue
        block = block_match.group(0)
        link_block_texts.append(block)
        mass = _read_float(r"float physics:mass = ([0-9.eE+-]+)", block)
        height = _read_float(r"double height = ([0-9.eE+-]+)", block)
        radius = _read_float(r"double radius = ([0-9.eE+-]+)", block)
        if mass is not None:
            masses.append(mass)
        if height is not None:
            heights.append(height)
        if radius is not None:
            radii.append(radius)

    all_stiffness = _read_all_float(r"float drive:(?:rotX|angular):physics:stiffness = ([0-9.eE+-]+)", text)
    all_damping = _read_all_float(r"float drive:(?:rotX|angular):physics:damping = ([0-9.eE+-]+)", text)
    rot_low = _read_all_float(r"float limit:rotX:physics:low = ([0-9.eE+-]+)", text)
    rot_high = _read_all_float(r"float limit:rotX:physics:high = ([0-9.eE+-]+)", text)

    authored_branch_mass = sum(masses)
    authored_total_length = sum(heights)
    branch_mass = authored_branch_mass * mass_scale_kg
    total_length = authored_total_length * linear_scale_m
    expected_mass = benchmark.density_kg_m3 * benchmark.area_m2 * benchmark.world_length_m
    expected_internal = benchmark.expected_internal_stiffness_usd(n_links)
    has_driven_attachment = support == "half_cell" and joint_model != "fixed_chain"
    internal_stiffness = all_stiffness[1:] if has_driven_attachment else all_stiffness
    config = experiment_config(
        benchmark.name, model, n_links, support, joint_model, collisions_enabled, backend
    )
    expected_fingerprint = config_fingerprint(config)
    fingerprint_match = re.search(r'cantilever:configFingerprint = "([0-9a-f]+)"', text)
    actual_fingerprint = fingerprint_match.group(1) if fingerprint_match else None
    expected_drive_count = 0 if joint_model == "fixed_chain" else n_links - 1 + int(support == "half_cell")
    expected_base_stiffness = benchmark.expected_base_stiffness_usd(n_links) if has_driven_attachment else None
    base_stiffness = all_stiffness[0] if has_driven_attachment and all_stiffness else None
    support_joint_ok = (
        'def PhysicsFixedJoint "AttachJoint"' in text
        if support == "fixed" or joint_model == "fixed_chain"
        else 'def PhysicsFixedJoint "AttachJoint"' not in text and base_stiffness is not None
    )

    return {
        "usd_path": str(path),
        "exists": path.exists(),
        "benchmark": benchmark.name,
        "model": model,
        "n_links": n_links,
        "support": support,
        "joint_model": joint_model,
        "collisions_enabled": collisions_enabled,
        "backend": backend,
        "config_fingerprint": actual_fingerprint,
        "expected_config_fingerprint": expected_fingerprint,
        "fingerprint_ok": actual_fingerprint == expected_fingerprint,
        "meters_per_unit": meters_per_unit,
        "kilograms_per_unit": kilograms_per_unit,
        "units_ok": (
            meters_per_unit == (0.01 if model == "legacy_current" else 1.0)
            and (kilograms_per_unit in (None, 1.0) if model == "legacy_current" else kilograms_per_unit == 1.0)
        ),
        "benchmark_si_units_ok": meters_per_unit == 1.0 and kilograms_per_unit == 1.0,
        "kilograms_per_unit_effective": mass_scale_kg,
        "tip_path": f"/World/Stem/{prefix}{n_links:02d}",
        "link_count": len(link_blocks),
        "total_length_m": total_length,
        "authored_total_length_units": authored_total_length,
        "expected_length_m": benchmark.world_length_m,
        "length_error_m": total_length - benchmark.world_length_m,
        "total_branch_mass_kg": branch_mass,
        "authored_total_branch_mass_units": authored_branch_mass,
        "expected_branch_mass_kg": expected_mass,
        "mass_error_kg": branch_mass - expected_mass,
        "link_height_m": heights[0] * linear_scale_m if heights else None,
        "authored_link_height_units": heights[0] if heights else None,
        "outer_radius_m": radii[0] * linear_scale_m if radii else None,
        "inner_radius_m": benchmark.inner_radius_m,
        "support_type": "half_cell" if has_driven_attachment else "fixed",
        "base_stiffness_usd": base_stiffness,
        "expected_base_stiffness_usd": expected_base_stiffness,
        "base_stiffness_rel_error": (
            (base_stiffness - expected_base_stiffness) / expected_base_stiffness
            if base_stiffness is not None and expected_base_stiffness
            else None
        ),
        "support_joint_ok": support_joint_ok,
        "internal_stiffness_usd_mean": sum(internal_stiffness) / len(internal_stiffness) if internal_stiffness else None,
        "expected_internal_stiffness_usd": expected_internal,
        "internal_stiffness_rel_error": (
            (sum(internal_stiffness) / len(internal_stiffness) - expected_internal) / expected_internal
            if internal_stiffness and expected_internal
            else None
        ),
        "damping_usd_values": all_damping,
        "drive_count": len(all_stiffness),
        "expected_drive_count": expected_drive_count,
        "drive_count_ok": len(all_stiffness) == expected_drive_count,
        "collision_api_count": text.count('prepend apiSchemas = ["PhysicsCollisionAPI"]'),
        "solver_position_iterations": _read_float(r"solverPositionIterationCount = ([0-9]+)", text),
        "solver_velocity_iterations": _read_float(r"solverVelocityIterationCount = ([0-9]+)", text),
        "gpu_dynamics_enabled": _read_bool(
            r"physxScene:enableGPUDynamics = (true|false|1|0)", text
        ),
        "rot_x_limit_low_min": min(rot_low) if rot_low else None,
        "rot_x_limit_high_max": max(rot_high) if rot_high else None,
        "filtered_pair_count": text.count("physics:filteredPairs"),
        "legacy_note": (
            "legacy_current is generated through current code with legacy_physics=True; "
            "it is not a frozen historical implementation."
            if model == "legacy_current"
            else None
        ),
    }


def audit_usd_files(
    benchmarks: tuple[str, ...],
    models: tuple[str, ...],
    n_links_values: tuple[int, ...],
    supports: tuple[str, ...],
    joint_models: tuple[str, ...],
    collisions_enabled: bool,
    backend: str,
) -> list[dict[str, Any]]:
    audits = []
    for benchmark_name in benchmarks:
        benchmark = BENCHMARKS[benchmark_name]
        for model in models:
            for support in supports:
                for joint_model in joint_models:
                    for n_links in n_links_values:
                        path = usd_path(benchmark_name, model, n_links, support, joint_model)
                        if not path.exists():
                            audits.append(
                                {
                                    "usd_path": str(path),
                                    "exists": False,
                                    "benchmark": benchmark_name,
                                    "model": model,
                                    "n_links": n_links,
                                    "support": support,
                                    "joint_model": joint_model,
                                }
                            )
                            continue
                        audit = audit_usd_file(
                            path,
                            benchmark,
                            model,
                            n_links,
                            support,
                            joint_model,
                            collisions_enabled,
                            backend,
                        )
                        audits.append(audit)
                        print(
                            "[audit] "
                            f"{path.name}: L={audit['total_length_m']:.6f} m, "
                            f"m={audit['total_branch_mass_kg']:.8f} kg, "
                            f"Kint={audit['internal_stiffness_usd_mean']} "
                            f"fingerprint_ok={audit['fingerprint_ok']}"
                        )
    return audits


def set_gravity(stage: Any, gravity_mps2: float) -> None:
    from pxr import Gf, UsdPhysics

    scene = UsdPhysics.Scene.Get(stage, "/World/PhysicsScene")
    if not scene:
        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(float(gravity_mps2))


def quat_rotate_wxyz(quat: Any, vec: Any):
    import numpy as np

    q = np.asarray(quat, dtype=np.float64)
    v = np.asarray(vec, dtype=np.float64)
    w = q[0]
    qv = q[1:4]
    t = 2.0 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


def squeeze_pose(value: Any):
    import numpy as np

    return np.squeeze(np.asarray(value, dtype=np.float64))


def tip_world_position(tip_prim: Any, link_height_m: float):
    pos, quat = tip_prim.get_world_poses()
    pos_v = squeeze_pose(pos)
    quat_v = squeeze_pose(quat)
    local_tip = [0.0, 0.0, link_height_m]
    return pos_v + quat_rotate_wxyz(quat_v, local_tip)


def tip_world_z(tip_prim: Any, link_height_m: float) -> float:
    return float(tip_world_position(tip_prim, link_height_m)[2])


def _as_serializable_array(value: Any) -> list[Any] | None:
    import numpy as np

    if value is None:
        return None
    try:
        if hasattr(value, "cpu"):
            value = value.cpu()
        if hasattr(value, "numpy"):
            value = value.numpy()
        return np.asarray(value).tolist()
    except (TypeError, ValueError):
        return None


def articulation_link_world_position(
    articulation: Any,
    link_index: int,
    local_position_m: Any,
):
    """Transform a link-local point using PhysX's articulation tensor state."""
    import numpy as np

    transforms = _as_serializable_array(articulation._physics_view.get_link_transforms())
    if transforms is None:
        raise RuntimeError("PhysX did not return articulation link transforms")
    array = np.asarray(transforms, dtype=np.float64)
    expected_shape = (
        int(articulation._physics_view.count),
        int(articulation._physics_view.max_links),
        7,
    )
    if array.size != math.prod(expected_shape):
        raise RuntimeError(
            f"Unexpected articulation link transform shape {array.shape}; "
            f"expected {expected_shape}"
        )
    transform = array.reshape(expected_shape)[0, link_index]
    # The low-level PhysX tensor API returns (x, y, z, qx, qy, qz, qw).
    quat_xyzw = transform[3:7]
    quat_wxyz = np.asarray(
        [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
        dtype=np.float64,
    )
    return transform[:3] + quat_rotate_wxyz(quat_wxyz, local_position_m)


def articulation_tip_world_position(
    articulation: Any,
    tip_link_index: int,
    link_height_m: float,
):
    return articulation_link_world_position(
        articulation,
        tip_link_index,
        [0.0, 0.0, link_height_m],
    )


def stage_vector_to_meters(value: Any, meters_per_unit: float):
    import numpy as np

    return np.asarray(value, dtype=np.float64) * meters_per_unit


def _max_abs(value: Any) -> float:
    import numpy as np

    serializable = _as_serializable_array(value)
    if serializable is None:
        return float("nan")
    array = np.asarray(serializable, dtype=np.float64)
    return float(np.max(np.abs(array))) if array.size else 0.0


def articulation_residuals(articulation: Any) -> dict[str, float | None]:
    values: dict[str, float | None] = {}
    for name, method_name, report_max in (
        ("position_max", "get_position_residuals", True),
        ("position_rms", "get_position_residuals", False),
        ("velocity_max", "get_velocity_residuals", True),
        ("velocity_rms", "get_velocity_residuals", False),
    ):
        try:
            values[name] = _max_abs(getattr(articulation, method_name)(report_max=report_max))
        except Exception:
            values[name] = None
    return values


def classify_validation_status(
    *,
    settled: bool,
    nan_detected: bool,
    joint_model: str,
    measured_mm: float,
    discrete_error_pct: float,
) -> str:
    if nan_detected:
        return "invalid_nan"
    if not settled:
        return "not_settled"
    if joint_model == "fixed_chain":
        return "passed" if abs(measured_mm) <= 0.02 else "settled_wrong_equilibrium"
    if not math.isnan(discrete_error_pct) and discrete_error_pct <= 10.0:
        return "passed"
    return "settled_wrong_equilibrium"


def synchronize_simulation_manager_scene(stage: Any) -> None:
    """Register the loaded scene so World.step does not use the 60 Hz fallback."""
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import PhysxSchema

    scene_path = "/World/PhysicsScene"
    scene_prim = stage.GetPrimAtPath(scene_path)
    if not scene_prim or scene_prim.GetTypeName() != "PhysicsScene":
        raise RuntimeError(f"Missing PhysicsScene at {scene_path}")
    scene_api = PhysxSchema.PhysxSceneAPI(scene_prim)
    # Isaac's stage-open callback can miss scenes that already exist in a loaded USD.
    SimulationManager._physics_scene_apis.clear()
    SimulationManager._physics_scene_apis[scene_path] = scene_api


def simulate_one(
    simulation_app: Any,
    benchmark_name: str,
    model: str,
    n_links: int,
    support: str,
    joint_model: str,
    scenario_name: str,
    force_point: str,
    collisions_enabled: bool,
    backend: str,
    max_seconds: float,
    physics_hz: float,
    substeps: int,
    settle_window_seconds: float,
    settle_tolerance_m: float,
    solver_position_iterations: int,
    solver_velocity_iterations: int,
    sample_hz: float,
    render: bool,
) -> dict[str, Any]:
    print(
        "[simulate] starting "
        f"{benchmark_name}/{model}/{support}/{joint_model}/N{n_links}/{scenario_name}/{physics_hz:g}Hz",
        flush=True,
    )
    import numpy as np
    import omni
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation, RigidPrim

    benchmark = BENCHMARKS[benchmark_name]
    scenario = SCENARIOS[scenario_name]
    path = usd_path(benchmark_name, model, n_links, support, joint_model)
    if not path.exists():
        raise FileNotFoundError(f"Missing generated USD: {path}. Run the `all` or `generate` command first.")
    audit = audit_usd_file(
        path, benchmark, model, n_links, support, joint_model, collisions_enabled, backend
    )
    if not audit["fingerprint_ok"] or not audit["drive_count_ok"]:
        raise RuntimeError(
            "USD configuration mismatch: "
            f"fingerprint_ok={audit['fingerprint_ok']} drive_count_ok={audit['drive_count_ok']} "
            f"for {path}"
        )
    tip_path = audit["tip_path"]
    link_height_stage_units = audit["authored_link_height_units"]

    if World.instance() is not None:
        World.instance().clear_instance()

    usd_context = omni.usd.get_context()
    usd_context.open_stage(str(path))
    while usd_context.get_stage_loading_status()[2] > 0:
        simulation_app.update()
    simulation_app.update()
    stage = usd_context.get_stage()
    set_gravity(stage, 0.0)

    from pxr import PhysxSchema, UsdGeom

    stage_meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))

    articulation_api = PhysxSchema.PhysxArticulationAPI.Get(stage, "/World/Stem")
    if not articulation_api:
        raise RuntimeError("Missing PhysxArticulationAPI on /World/Stem")
    # Author before reset: PhysX reads these values when the articulation is added.
    articulation_api.GetSolverPositionIterationCountAttr().Set(solver_position_iterations)
    articulation_api.GetSolverVelocityIterationCountAttr().Set(solver_velocity_iterations)

    physics_dt = 1.0 / physics_hz
    world = World(
        physics_dt=physics_dt,
        rendering_dt=1.0 / 60.0,
        stage_units_in_meters=stage_meters_per_unit,
        physics_prim_path="/World/PhysicsScene",
        sim_params={
            "dt": physics_dt,
            "substeps": substeps,
            "gravity": [0.0, 0.0, 0.0],
            "solver_type": 1,
            "enable_solver_residuals": True,
        },
    )
    physics_context = world.get_physics_context()
    runtime_stage_meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    if not math.isclose(
        runtime_stage_meters_per_unit,
        stage_meters_per_unit,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise RuntimeError(
            "World changed stage linear units: "
            f"before={stage_meters_per_unit} after={runtime_stage_meters_per_unit}"
        )
    if physics_context.prim_path != "/World/PhysicsScene":
        raise RuntimeError(
            f"Unexpected PhysicsScene: {physics_context.prim_path}; expected /World/PhysicsScene"
        )
    physics_context.set_physics_dt(physics_dt, substeps=substeps)
    world.set_simulation_dt(physics_dt=physics_dt, rendering_dt=1.0 / 60.0)
    physics_context.set_gravity(0.0)
    synchronize_simulation_manager_scene(stage)
    articulation = Articulation(
        "/World/Stem",
        name=f"stem_{benchmark_name}_{model}_{support}_{joint_model}_{n_links}",
        enable_residual_reports=True,
    )
    world.scene.add(articulation)
    world.reset()
    articulation.initialize()
    # Stage-open/reset events can re-register the scene with default timing.
    physics_context.set_physics_dt(physics_dt, substeps=substeps)
    world.set_simulation_dt(physics_dt=physics_dt, rendering_dt=1.0 / 60.0)
    synchronize_simulation_manager_scene(stage)
    world_reported_physics_dt = float(world.get_physics_dt())
    scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
    scene_hz = int(PhysxSchema.PhysxSceneAPI(scene_prim).GetTimeStepsPerSecondAttr().Get())
    scene_physics_dt = 1.0 / scene_hz if scene_hz else 0.0
    if not (
        math.isclose(world_reported_physics_dt, physics_dt, rel_tol=0.0, abs_tol=1e-12)
        and math.isclose(scene_physics_dt, physics_dt, rel_tol=0.0, abs_tol=1e-12)
    ):
        raise RuntimeError(
            "Physics timestep mismatch: "
            f"requested={physics_dt:.12g}s world={world_reported_physics_dt:.12g}s "
            f"scene={scene_physics_dt:.12g}s ({scene_hz}Hz)"
        )
    runtime_position_iterations = int(
        np.asarray(articulation.get_solver_position_iteration_counts()).reshape(-1)[0]
    )
    runtime_velocity_iterations = int(
        np.asarray(articulation.get_solver_velocity_iteration_counts()).reshape(-1)[0]
    )
    if (
        runtime_position_iterations != solver_position_iterations
        or runtime_velocity_iterations != solver_velocity_iterations
    ):
        raise RuntimeError(
            "Solver iteration mismatch: "
            f"requested={solver_position_iterations}/{solver_velocity_iterations} "
            f"runtime={runtime_position_iterations}/{runtime_velocity_iterations}"
        )
    print(
        "[simulate] solver iterations "
        f"position/velocity={runtime_position_iterations}/{runtime_velocity_iterations}",
        flush=True,
    )

    tip = RigidPrim(tip_path)
    tip.initialize()
    tip_link_name = tip_path.rsplit("/", 1)[-1]
    tip_link_index = articulation.get_link_index(tip_link_name)
    if tip_link_index is None:
        raise RuntimeError(f"Articulation does not contain tip link {tip_link_name}")
    metadata = getattr(articulation, "_metadata", None)
    link_names = list(getattr(metadata, "link_names", [])) if metadata is not None else []
    if not (0 <= tip_link_index < len(link_names)) or link_names[tip_link_index] != tip_link_name:
        raise RuntimeError(
            f"Tip link index mismatch: name={tip_link_name} index={tip_link_index} "
            f"links={link_names}"
        )
    initial_tip_position = articulation_tip_world_position(
        articulation, tip_link_index, link_height_stage_units
    )
    initial_rigid_tip_position = tip_world_position(tip, link_height_stage_units)
    z0_stage_units = float(initial_tip_position[2])
    set_gravity(stage, scenario.gravity_mps2)
    physics_context.set_gravity(-abs(scenario.gravity_mps2) if scenario.gravity_mps2 else 0.0)

    max_steps = int(max_seconds * physics_hz)
    settle_window_steps = max(1, int(round(settle_window_seconds * physics_hz)))
    min_steps = max(60, settle_window_steps)
    deflections = []
    samples = []
    settled = False
    nan_detected = False
    articulation_force_data = np.zeros(
        (articulation._physics_view.count, articulation._physics_view.max_links, 3),
        dtype=np.float32,
    )
    articulation_force_data[0, tip_link_index] = [0.0, 0.0, scenario.tip_force_n]
    articulation_indices = np.asarray([0], dtype=np.uint32)
    max_joint_velocity_rps = float("inf")
    reported_max_joint_velocity_rps = float("inf")
    previous_joint_positions = np.asarray(
        _as_serializable_array(articulation.get_joint_positions()), dtype=np.float64
    )
    sample_interval_steps = max(1, int(round(physics_hz / sample_hz)))

    for step in range(max_steps):
        if scenario.tip_force_n != 0.0:
            if force_point == "geometric_tip":
                application_positions = np.zeros_like(articulation_force_data)
                application_positions[0, tip_link_index] = np.asarray(
                    articulation_tip_world_position(
                        articulation, tip_link_index, link_height_stage_units
                    ),
                    dtype=np.float32,
                )
                articulation._physics_view.apply_forces_and_torques_at_position(
                    articulation_force_data,
                    None,
                    application_positions,
                    articulation_indices,
                    True,
                )
            else:
                articulation._physics_view.apply_forces_and_torques_at_position(
                    articulation_force_data,
                    None,
                    None,
                    articulation_indices,
                    True,
                )
        world.step(render=render)

        current_tip_position = articulation_tip_world_position(
            articulation, tip_link_index, link_height_stage_units
        )
        z_stage_units = float(current_tip_position[2])
        if math.isnan(z_stage_units):
            nan_detected = True
            break
        deflection_m = float(
            stage_vector_to_meters(
                z0_stage_units - z_stage_units,
                runtime_stage_meters_per_unit,
            )
        )
        deflections.append(deflection_m)

        try:
            joint_positions = np.asarray(
                _as_serializable_array(articulation.get_joint_positions()), dtype=np.float64
            )
            max_joint_velocity_rps = _max_abs(
                (joint_positions - previous_joint_positions) / physics_dt
            )
            previous_joint_positions = joint_positions.copy()
            reported_max_joint_velocity_rps = _max_abs(
                articulation.get_joint_velocities()
            )
        except Exception:
            max_joint_velocity_rps = float("nan")
            reported_max_joint_velocity_rps = float("nan")

        if step % sample_interval_steps == 0:
            samples.append(
                {
                    "step": step,
                    "time_s": (step + 1) / physics_hz,
                    "world_time_s": float(world.current_time),
                    "world_step_index": int(world.current_time_step_index),
                    "deflection_mm": deflection_m * 1000.0,
                    "max_joint_velocity_rps": max_joint_velocity_rps,
                    "reported_max_joint_velocity_rps": reported_max_joint_velocity_rps,
                    "residuals": articulation_residuals(articulation),
                }
            )

        if step >= min_steps and len(deflections) >= settle_window_steps:
            recent = deflections[-settle_window_steps:]
            position_stable = max(recent) - min(recent) <= settle_tolerance_m
            # Static convergence is defined on the measured observable. PhysX/TGS
            # joint velocities include solver pseudo-velocity, while finite
            # differences of tiny float32 joint angles become quantized at high N.
            if position_stable:
                settled = True
                break

    final_deflection_m = deflections[-1] if deflections else float("nan")
    expected_mm = scenario.expected_mm(benchmark, n_links, force_point)
    expected_discrete_mm = scenario.expected_discrete_mm(
        benchmark, n_links, support, force_point, joint_model
    )
    measured_mm = final_deflection_m * 1000.0
    reference_comparable = bool(
        audit["benchmark_si_units_ok"]
        and abs(audit["length_error_m"]) <= 1e-9
        and abs(audit["mass_error_kg"])
        <= max(
            1e-8,
            1e-6 * benchmark.density_kg_m3 * benchmark.area_m2 * benchmark.world_length_m,
        )
    )
    error_pct = abs(measured_mm - expected_mm) / expected_mm * 100.0 if expected_mm else float("nan")
    discrete_error_pct = (
        abs(measured_mm - expected_discrete_mm) / expected_discrete_mm * 100.0
        if expected_discrete_mm > 0.0
        else float("nan")
    )
    effective_ei_nm2 = (
        benchmark.flexural_rigidity_nm2 * expected_discrete_mm / measured_mm
        if reference_comparable and measured_mm and not math.isnan(measured_mm)
        else float("nan")
    )
    validation_status = classify_validation_status(
        settled=settled,
        nan_detected=nan_detected,
        joint_model=joint_model,
        measured_mm=measured_mm,
        discrete_error_pct=discrete_error_pct,
    )
    if not reference_comparable:
        validation_status = "baseline_not_physically_equivalent"

    final_residuals = articulation_residuals(articulation)
    final_tip_position = articulation_tip_world_position(
        articulation, tip_link_index, link_height_stage_units
    )
    final_rigid_tip_position = tip_world_position(tip, link_height_stage_units)
    try:
        final_joint_positions = _as_serializable_array(articulation.get_joint_positions())
    except Exception:
        final_joint_positions = None
    try:
        reaction_wrenches = _as_serializable_array(articulation.get_measured_joint_forces())
    except Exception:
        reaction_wrenches = None
    joint_names = list(getattr(metadata, "joint_names", [])) if metadata is not None else []
    world_final_time_s = float(world.current_time)
    world_final_step_index = int(world.current_time_step_index)

    world.stop()
    world.clear_instance()
    omni.usd.get_context().close_stage()

    result = {
        "benchmark": benchmark_name,
        "model": model,
        "n_links": n_links,
        "support": support,
        "joint_model": joint_model,
        "scenario": scenario_name,
        "force_point": force_point,
        "collisions_enabled": collisions_enabled,
        "backend": backend,
        "config_fingerprint": audit["config_fingerprint"],
        "usd_path": str(path),
        "tip_path": tip_path,
        "tip_link_name": tip_link_name,
        "tip_link_index": tip_link_index,
        "measurement_source": "articulation_link_transforms",
        "force_application_source": "articulation_link_tensor",
        "initial_tip_z_m": z0_stage_units * runtime_stage_meters_per_unit,
        "initial_tip_position_m": _as_serializable_array(
            stage_vector_to_meters(initial_tip_position, runtime_stage_meters_per_unit)
        ),
        "final_tip_position_m": _as_serializable_array(
            stage_vector_to_meters(final_tip_position, runtime_stage_meters_per_unit)
        ),
        "initial_rigid_prim_tip_position_m": _as_serializable_array(
            stage_vector_to_meters(initial_rigid_tip_position, runtime_stage_meters_per_unit)
        ),
        "final_rigid_prim_tip_position_m": _as_serializable_array(
            stage_vector_to_meters(final_rigid_tip_position, runtime_stage_meters_per_unit)
        ),
        "final_deflection_m": final_deflection_m,
        "final_deflection_mm": measured_mm,
        "expected_deflection_mm": expected_mm,
        "expected_discrete_deflection_mm": expected_discrete_mm,
        "reference_comparable": reference_comparable,
        "error_pct": error_pct,
        "effective_ei_nm2": effective_ei_nm2,
        "target_ei_nm2": benchmark.flexural_rigidity_nm2,
        "discrete_error_pct": discrete_error_pct,
        "settled": settled,
        "validation_status": validation_status,
        "not_settled": not settled,
        "nan_detected": nan_detected,
        "steps": len(deflections),
        "sim_time_s": len(deflections) / physics_hz,
        "world_final_time_s": world_final_time_s,
        "world_final_step_index": world_final_step_index,
        "physics_hz": physics_hz,
        "physics_dt": physics_dt,
        "requested_sample_hz": sample_hz,
        "effective_sample_hz": physics_hz / sample_interval_steps,
        "world_reported_physics_dt": world_reported_physics_dt,
        "scene_physics_dt": scene_physics_dt,
        "stage_meters_per_unit": runtime_stage_meters_per_unit,
        "step_mode": "world_step_registered_scene",
        "substeps": substeps,
        "settle_window_seconds": settle_window_seconds,
        "settle_tolerance_m": settle_tolerance_m,
        "settle_criterion": "tip_deflection_range_over_window",
        "joint_velocity_used_for_settling": False,
        "solver_position_iterations": runtime_position_iterations,
        "solver_velocity_iterations": runtime_velocity_iterations,
        "max_joint_velocity_rps": max_joint_velocity_rps,
        "reported_max_joint_velocity_rps": reported_max_joint_velocity_rps,
        "solver_residuals": final_residuals,
        "link_names": link_names,
        "joint_names": joint_names,
        "final_joint_positions": final_joint_positions,
        "reaction_wrenches": reaction_wrenches,
        "samples": samples,
    }
    print(
        "[simulate] "
        f"{benchmark_name}/{model}/{support}/{joint_model}/N{n_links}/{scenario_name}/{physics_hz:g}Hz: "
        f"{measured_mm:.4f} mm vs discrete={expected_discrete_mm:.4f} mm "
        f"discrete_err={discrete_error_pct:.1f}% settled={settled} status={validation_status}"
    )
    return result


def simulate_all(
    simulation_app: Any,
    benchmarks: tuple[str, ...],
    models: tuple[str, ...],
    n_links_values: tuple[int, ...],
    supports: tuple[str, ...],
    joint_models: tuple[str, ...],
    scenarios: tuple[str, ...],
    force_point: str,
    collisions_enabled: bool,
    backend: str,
    max_seconds: float,
    physics_hz_values: tuple[float, ...],
    substeps: int,
    settle_window_seconds: float,
    settle_tolerance_m: float,
    solver_position_iterations: int,
    solver_velocity_iterations: int,
    sample_hz: float,
    render: bool,
) -> list[dict[str, Any]]:
    results = []
    for benchmark_name in benchmarks:
        for model in models:
            for support in supports:
                for joint_model in joint_models:
                    for n_links in n_links_values:
                        for scenario_name in scenarios:
                            for physics_hz in physics_hz_values:
                                try:
                                    results.append(
                                        simulate_one(
                                            simulation_app,
                                            benchmark_name,
                                            model,
                                            n_links,
                                            support,
                                            joint_model,
                                            scenario_name,
                                            force_point,
                                            collisions_enabled,
                                            backend,
                                            max_seconds,
                                            physics_hz,
                                            substeps,
                                            settle_window_seconds,
                                            settle_tolerance_m,
                                            solver_position_iterations,
                                            solver_velocity_iterations,
                                            sample_hz,
                                            render,
                                        )
                                    )
                                except KeyboardInterrupt:
                                    raise
                                except BaseException as exc:
                                    print(
                                        "[simulate] error "
                                        f"{benchmark_name}/{model}/{support}/{joint_model}/N{n_links}/"
                                        f"{scenario_name}/{physics_hz:g}Hz: {type(exc).__name__}: {exc}",
                                        flush=True,
                                    )
                                    results.append(
                                        {
                                            "benchmark": benchmark_name,
                                            "model": model,
                                            "n_links": n_links,
                                            "support": support,
                                            "joint_model": joint_model,
                                            "scenario": scenario_name,
                                            "force_point": force_point,
                                            "collisions_enabled": collisions_enabled,
                                            "backend": backend,
                                            "physics_hz": physics_hz,
                                            "error": f"{type(exc).__name__}: {exc}",
                                            "settled": False,
                                            "not_settled": True,
                                        }
                                    )
    return results


def formula_check() -> dict[str, Any]:
    checks = {}
    for name, benchmark in BENCHMARKS.items():
        checks[name] = {
            "area_m2": benchmark.area_m2,
            "I_m4": benchmark.second_moment_m4,
            "EI_Nm2": benchmark.flexural_rigidity_nm2,
            "w_Npm": benchmark.distributed_weight_npm,
            "self_weight_mm": benchmark.expected_self_weight_mm,
            "tip_force_0p05N_mm": benchmark.expected_tip_force_mm(0.05),
            "internal_stiffness_usd_by_n": {
                str(n): benchmark.expected_internal_stiffness_usd(n) for n in DEFAULT_N_LINKS
            },
            "discrete_tip_force_fixed_by_n": {
                str(n): benchmark.expected_discrete_tip_force_mm(n, 0.05)
                for n in DEFAULT_N_LINKS
            },
            "discrete_tip_force_half_cell_by_n": {
                str(n): benchmark.expected_discrete_tip_force_mm(n, 0.05, support="half_cell")
                for n in DEFAULT_N_LINKS
            },
        }

    assert abs(checks["synthetic_solid_40cm"]["self_weight_mm"] - 12.5568) < 0.01
    assert abs(checks["tomato_gao_20cm"]["self_weight_mm"] - 3.4637) < 0.01
    assert abs(checks["tomato_gao_20cm"]["tip_force_0p05N_mm"] - 3.5836) < 0.01
    synthetic = BENCHMARKS["synthetic_solid_40cm"]
    assert abs(synthetic.expected_discrete_tip_force_mm(3, 0.05) - 0.7545123) < 1e-6
    assert abs(
        synthetic.expected_discrete_tip_force_mm(3, 0.05, support="half_cell", force_point="com")
        - 1.094043
    ) < 1e-6
    assert abs(synthetic.expected_discrete_self_weight_mm(20) - 11.332512) < 1e-6
    print(json.dumps(checks, indent=2))
    return checks


def evaluate_acceptance(
    audits: list[dict[str, Any]],
    measurements: list[dict[str, Any]],
    n_links_values: tuple[int, ...],
) -> dict[str, Any]:
    acceptance: dict[str, Any] = {
        "artifact_consistency": {},
        "benchmark_physical_consistency": {},
        "length_mass_invariant": {},
        "physx_vs_matching_discrete_within_10pct": {},
        "single_hinge_within_2pct": {},
        "timestep_independence_within_5pct": {},
        "fixed_chain_below_0p02mm": {},
        "synthetic_validation_passed": False,
        "tomato_realism_claim_allowed": False,
    }

    for row in audits:
        if not row.get("exists"):
            continue
        key = (
            f"{row['benchmark']}/{row['model']}/{row['support']}/"
            f"{row['joint_model']}/N{row['n_links']}"
        )
        collision_ok = (
            row["collision_api_count"] > 0
            if row["collisions_enabled"]
            else row["collision_api_count"] == 0
        )
        stiffness_ok = (
            row["internal_stiffness_rel_error"] is None
            or abs(row["internal_stiffness_rel_error"]) <= 1e-5
        ) and (
            row["base_stiffness_rel_error"] is None
            or abs(row["base_stiffness_rel_error"]) <= 1e-5
        )
        solver_ok = (
            row["solver_position_iterations"] == DEFAULT_SOLVER_POSITION_ITERATIONS
            and row["solver_velocity_iterations"] == DEFAULT_SOLVER_VELOCITY_ITERATIONS
            and row["gpu_dynamics_enabled"] == (row["backend"] == "gpu")
        )
        acceptance["artifact_consistency"][key] = {
            "ok": (
                row["fingerprint_ok"]
                and row["drive_count_ok"]
                and row["units_ok"]
                and row["support_joint_ok"]
                and stiffness_ok
                and solver_ok
                and collision_ok
            ),
            "fingerprint_ok": row["fingerprint_ok"],
            "drive_count_ok": row["drive_count_ok"],
            "units_ok": row["units_ok"],
            "support_joint_ok": row["support_joint_ok"],
            "stiffness_ok": stiffness_ok,
            "solver_ok": solver_ok,
            "collision_state_ok": collision_ok,
        }
        length_ok = abs(row["length_error_m"]) <= 1e-9
        mass_tolerance = max(1e-8, 1e-6 * abs(row["expected_branch_mass_kg"]))
        mass_ok = abs(row["mass_error_kg"]) <= mass_tolerance
        benchmark_si_units_ok = row.get(
            "benchmark_si_units_ok",
            row.get("meters_per_unit") == 1.0 and row.get("kilograms_per_unit") == 1.0,
        )
        acceptance["benchmark_physical_consistency"][key] = {
            "ok": benchmark_si_units_ok and length_ok and mass_ok,
            "si_units_ok": benchmark_si_units_ok,
            "length_ok": length_ok,
            "mass_ok": mass_ok,
            "length_error_m": row["length_error_m"],
            "mass_error_kg": row["mass_error_kg"],
        }

    for benchmark_name in BENCHMARKS:
        for model in DEFAULT_MODELS:
            subset = [
                row
                for row in audits
                if row.get("exists") and row["benchmark"] == benchmark_name and row["model"] == model
            ]
            if not subset:
                continue
            lengths = [row["total_length_m"] for row in subset]
            masses = [row["total_branch_mass_kg"] for row in subset]
            length_ok = max(lengths) - min(lengths) <= 1e-9
            mass_span = max(masses) - min(masses)
            mass_tol = max(1e-8, 1e-6 * max(abs(mass) for mass in masses))
            mass_ok = mass_span <= mass_tol
            acceptance["length_mass_invariant"][f"{benchmark_name}/{model}"] = {
                "ok": length_ok and mass_ok,
                "length_span_m": max(lengths) - min(lengths),
                "mass_span_kg": mass_span,
                "mass_tolerance_kg": mass_tol,
            }

    valid_rows = [row for row in measurements if "final_deflection_mm" in row]
    for row in valid_rows:
        key = (
            f"{row['benchmark']}/{row['model']}/{row['support']}/{row['joint_model']}/"
            f"N{row['n_links']}/{row['scenario']}/{row['physics_hz']:g}Hz"
        )
        if row["joint_model"] == "fixed_chain":
            acceptance["fixed_chain_below_0p02mm"][key] = {
                "ok": abs(row["final_deflection_mm"]) <= 0.02 and row["settled"],
                "deflection_mm": row["final_deflection_mm"],
                "settled": row["settled"],
            }
        elif row.get("reference_comparable", True) and not math.isnan(row["discrete_error_pct"]):
            acceptance["physx_vs_matching_discrete_within_10pct"][key] = {
                "ok": row["discrete_error_pct"] <= 10.0 and row["settled"],
                "discrete_error_pct": row["discrete_error_pct"],
                "settled": row["settled"],
            }
            if row["n_links"] == 2 and row["support"] == "fixed":
                acceptance["single_hinge_within_2pct"][key] = {
                    "ok": row["discrete_error_pct"] <= 2.0 and row["settled"],
                    "discrete_error_pct": row["discrete_error_pct"],
                    "settled": row["settled"],
                }

    timestep_groups: dict[tuple[Any, ...], list[dict[str, Any]]] = {}
    for row in valid_rows:
        group_key = (
            row["benchmark"], row["model"], row["support"], row["joint_model"],
            row["n_links"], row["scenario"], row["force_point"], row["backend"],
        )
        timestep_groups.setdefault(group_key, []).append(row)
    for group_key, rows in timestep_groups.items():
        by_frequency = {row["physics_hz"]: row for row in rows}
        if len(by_frequency) < 2:
            continue
        frequencies = sorted(by_frequency)
        deflections = [by_frequency[frequency]["final_deflection_mm"] for frequency in frequencies]
        discrete_references = [
            abs(row.get("expected_discrete_deflection_mm", 0.0)) for row in rows
        ]
        normalization_mm = max(discrete_references, default=0.0)
        if normalization_mm <= 0.0:
            normalization_mm = max((abs(value) for value in deflections), default=0.0)
        relative_span = (
            (max(deflections) - min(deflections)) / normalization_mm
            if normalization_mm > 0.0
            else float("inf")
        )
        finest_lower_hz, finest_higher_hz = frequencies[-2:]
        finest_pair_difference_mm = abs(
            by_frequency[finest_higher_hz]["final_deflection_mm"]
            - by_frequency[finest_lower_hz]["final_deflection_mm"]
        )
        finest_pair_relative_difference = (
            finest_pair_difference_mm / normalization_mm
            if normalization_mm > 0.0
            else float("inf")
        )
        key = "/".join(map(str, group_key))
        acceptance["timestep_independence_within_5pct"][key] = {
            "ok": (
                finest_pair_relative_difference <= 0.05
                and by_frequency[finest_lower_hz]["settled"]
                and by_frequency[finest_higher_hz]["settled"]
            ),
            "frequencies_hz": frequencies,
            "deflection_span_mm": max(deflections) - min(deflections),
            "normalization_mm": normalization_mm,
            "relative_span": relative_span,
            "finest_pair_hz": [finest_lower_hz, finest_higher_hz],
            "finest_pair_difference_mm": finest_pair_difference_mm,
            "finest_pair_relative_difference": finest_pair_relative_difference,
        }

    required_synthetic = [
        row
        for row in valid_rows
        if row["benchmark"] == "synthetic_solid_40cm"
        and row["model"] == "new_physics"
        and row["support"] == "fixed"
        and row["joint_model"] == "d6_biaxial"
        and row["force_point"] == "geometric_tip"
        and row["n_links"] == 20
    ]
    synthetic_accuracy = any(
        row["settled"]
        and not math.isnan(row["discrete_error_pct"])
        and row["discrete_error_pct"] <= 10.0
        for row in required_synthetic
    )
    synthetic_timestep = any(
        item["ok"]
        for key, item in acceptance["timestep_independence_within_5pct"].items()
        if key.startswith("synthetic_solid_40cm/new_physics/fixed/d6_biaxial/20/")
    )
    acceptance["synthetic_validation_passed"] = synthetic_accuracy and synthetic_timestep
    tomato_n20_rows = [
        row
        for row in valid_rows
        if row["benchmark"] == "tomato_gao_20cm"
        and row["model"] == "new_physics"
        and row["support"] == "fixed"
        and row["joint_model"] == "d6_biaxial"
        and row["force_point"] == "geometric_tip"
        and row["n_links"] == 20
    ]
    tomato_finest_by_scenario: dict[str, dict[str, Any]] = {}
    for row in tomato_n20_rows:
        previous = tomato_finest_by_scenario.get(row["scenario"])
        if previous is None or row["physics_hz"] > previous["physics_hz"]:
            tomato_finest_by_scenario[row["scenario"]] = row

    required_tomato_scenarios = {"tip_force_0p05N", "self_weight"}
    tomato_parameter_validation = (
        set(tomato_finest_by_scenario) == required_tomato_scenarios
        and all(
            row["settled"]
            and row["discrete_error_pct"] <= 10.0
            and row["error_pct"] <= 25.0
            for row in tomato_finest_by_scenario.values()
        )
    )

    def timestep_result_for(benchmark: str, scenario: str) -> bool:
        prefix = f"{benchmark}/new_physics/fixed/d6_biaxial/20/{scenario}/"
        return any(
            item["ok"]
            for key, item in acceptance["timestep_independence_within_5pct"].items()
            if key.startswith(prefix)
        )

    tomato_tip_timestep = timestep_result_for("tomato_gao_20cm", "tip_force_0p05N")
    tomato_self_weight_timestep = timestep_result_for("tomato_gao_20cm", "self_weight")
    acceptance["tomato_parameter_grounded_validation_passed"] = bool(
        acceptance["synthetic_validation_passed"] and tomato_parameter_validation
    )
    acceptance["tomato_tip_force_timestep_validation_passed"] = tomato_tip_timestep
    acceptance["tomato_self_weight_timestep_validation_passed"] = tomato_self_weight_timestep
    acceptance["tomato_empirical_ground_truth_available"] = False
    acceptance["tomato_realism_claim_allowed"] = bool(
        acceptance["tomato_parameter_grounded_validation_passed"]
        and tomato_tip_timestep
        and tomato_self_weight_timestep
        and acceptance["tomato_empirical_ground_truth_available"]
    )

    return acceptance


def sanitize_json_value(value: Any) -> Any:
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: sanitize_json_value(item) for key, item in value.items()}
    if isinstance(value, list):
        return [sanitize_json_value(item) for item in value]
    return value


def write_last_run_checkpoint(
    generated: list[dict[str, Any]],
    audits: list[dict[str, Any]],
    measurements: list[dict[str, Any]],
) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    payload = {"generated": generated, "audits": audits, "measurements": measurements}
    with last_run_checkpoint_path().open("w") as handle:
        json.dump(sanitize_json_value(payload), handle, indent=2, allow_nan=False)
    print(f"[checkpoint] {last_run_checkpoint_path()}", flush=True)


def write_json_results(payload: dict[str, Any]) -> None:

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    with result_json_path().open("w") as handle:
        json.dump(sanitize_json_value(payload), handle, indent=2, allow_nan=False)
    print(f"[results] {result_json_path()}")


def write_measurement_csv(measurements: list[dict[str, Any]]) -> None:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    fields = [
        "benchmark",
        "model",
        "support",
        "joint_model",
        "n_links",
        "scenario",
        "force_point",
        "backend",
        "physics_hz",
        "final_deflection_mm",
        "expected_deflection_mm",
        "error_pct",
        "expected_discrete_deflection_mm",
        "discrete_error_pct",
        "effective_ei_nm2",
        "target_ei_nm2",
        "settled",
        "validation_status",
        "max_joint_velocity_rps",
        "reported_max_joint_velocity_rps",
        "steps",
        "sim_time_s",
        "usd_path",
    ]
    with result_csv_path().open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for row in measurements:
            writer.writerow(
                {
                    field: ""
                    if isinstance(row.get(field), float) and not math.isfinite(row[field])
                    else row.get(field)
                    for field in fields
                }
            )
    print(f"[results] {result_csv_path()}")


def write_report(payload: dict[str, Any]) -> None:
    from scientific_report import render_scientific_report

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    DOCS_DIR.mkdir(parents=True, exist_ok=True)
    report = render_scientific_report(payload)
    report_path().write_text(report)
    (DOCS_DIR / "CantileverValidationReport.md").write_text(report)
    print(f"[results] {report_path()}")
    print(f"[report] {DOCS_DIR / 'CantileverValidationReport.md'}")


def benchmark_payload() -> dict[str, Any]:
    payload = {}
    for name, benchmark in BENCHMARKS.items():
        data = asdict(benchmark)
        data.update(
            {
                "area_m2": benchmark.area_m2,
                "second_moment_m4": benchmark.second_moment_m4,
                "flexural_rigidity_nm2": benchmark.flexural_rigidity_nm2,
                "distributed_weight_npm": benchmark.distributed_weight_npm,
                "expected_self_weight_mm": benchmark.expected_self_weight_mm,
                "expected_tip_force_0p05N_mm": benchmark.expected_tip_force_mm(0.05),
            }
        )
        payload[name] = data
    return payload


def build_payload(
    generated: list[dict[str, Any]],
    audits: list[dict[str, Any]],
    measurements: list[dict[str, Any]],
    n_links_values: tuple[int, ...],
) -> dict[str, Any]:
    return {
        "benchmarks": benchmark_payload(),
        "generated": generated,
        "audits": audits,
        "measurements": measurements,
        "acceptance": evaluate_acceptance(audits, measurements, n_links_values),
        "sources": {
            "gao_2024": "https://www.mdpi.com/2077-0472/14/4/531",
            "openusd_drive_api": "https://openusd.org/release/api/class_usd_physics_drive_a_p_i.html",
            "coutand_2000": "https://pubmed.ncbi.nlm.nih.gov/11113160/",
            "plant_methods_flexural_stiffness": "https://ouci.dntb.gov.ua/en/works/l1pMrXo4/",
        },
    }


def load_existing_payload() -> dict[str, Any]:
    with result_json_path().open() as handle:
        return json.load(handle)


def merge_records(
    existing: list[dict[str, Any]],
    new: list[dict[str, Any]],
    key_fields: tuple[str, ...],
) -> list[dict[str, Any]]:
    merged = {tuple(row.get(field) for field in key_fields): row for row in existing}
    for row in new:
        merged[tuple(row.get(field) for field in key_fields)] = row
    return list(merged.values())


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Cantilever physics validation runner")
    parser.add_argument(
        "command",
        choices=("formula-check", "generate", "audit", "simulate", "report", "all"),
        help="Action to run. `all` generates, audits, simulates, and writes reports.",
    )
    parser.add_argument("--benchmarks", default=",".join(BENCHMARKS.keys()))
    parser.add_argument("--models", default=",".join(DEFAULT_MODELS))
    parser.add_argument("--n-links", default=",".join(str(n) for n in DEFAULT_N_LINKS))
    parser.add_argument("--supports", default=",".join(DEFAULT_SUPPORTS))
    parser.add_argument("--joint-models", default=",".join(DEFAULT_JOINT_MODELS))
    parser.add_argument("--scenarios", default=",".join(SCENARIOS.keys()))
    parser.add_argument("--force-point", choices=sorted(VALID_FORCE_POINTS), default="geometric_tip")
    parser.add_argument("--backend", choices=("cpu", "gpu"), default="cpu")
    parser.add_argument(
        "--collisions",
        action="store_true",
        help="Enable collision shapes. Diagnostic runs disable them by default.",
    )
    parser.add_argument("--max-seconds", type=float, default=20.0)
    parser.add_argument("--physics-hz", default="480", help="One value or comma-separated timestep sweep.")
    parser.add_argument("--substeps", type=int, default=1)
    parser.add_argument("--settle-window-seconds", type=float, default=0.5)
    parser.add_argument("--settle-tolerance-m", type=float, default=2.0e-5)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument(
        "--solver-position-iterations",
        type=int,
        default=DEFAULT_SOLVER_POSITION_ITERATIONS,
    )
    parser.add_argument(
        "--solver-velocity-iterations",
        type=int,
        default=DEFAULT_SOLVER_VELOCITY_ITERATIONS,
        help="Keep at or below 4 for TGS unless deliberately reproducing the legacy diagnostic.",
    )
    parser.add_argument("--gui", action="store_true", help="Render simulation frames.")
    parser.add_argument(
        "--append-results",
        action="store_true",
        help="Merge this run into the existing evidence dataset instead of replacing it.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    print(f"[runner] command={args.command}", flush=True)
    benchmarks = _split_csv(args.benchmarks, tuple(BENCHMARKS.keys()))
    models = _split_csv(args.models, DEFAULT_MODELS)
    n_links_values = _split_int_csv(args.n_links, DEFAULT_N_LINKS)
    supports = _split_csv(args.supports, DEFAULT_SUPPORTS)
    joint_models = _split_csv(args.joint_models, DEFAULT_JOINT_MODELS)
    scenarios = _split_csv(args.scenarios, tuple(SCENARIOS.keys()))
    physics_hz_values = _split_float_csv(args.physics_hz, (480.0,))

    unknown_benchmarks = sorted(set(benchmarks) - set(BENCHMARKS))
    unknown_models = sorted(set(models) - set(DEFAULT_MODELS))
    unknown_supports = sorted(set(supports) - VALID_SUPPORTS)
    unknown_joint_models = sorted(set(joint_models) - VALID_JOINT_MODELS)
    unknown_scenarios = sorted(set(scenarios) - set(SCENARIOS))
    if unknown_benchmarks or unknown_models or unknown_supports or unknown_joint_models or unknown_scenarios:
        raise ValueError(
            f"Unknown choices: benchmarks={unknown_benchmarks}, "
            f"models={unknown_models}, supports={unknown_supports}, "
            f"joint_models={unknown_joint_models}, scenarios={unknown_scenarios}"
        )
    if any(value <= 0 for value in physics_hz_values):
        raise ValueError("All --physics-hz values must be positive")
    if args.sample_hz <= 0:
        raise ValueError("--sample-hz must be positive")
    if not 1 <= args.solver_position_iterations <= 255:
        raise ValueError("--solver-position-iterations must be in [1, 255]")
    if not 1 <= args.solver_velocity_iterations <= 255:
        raise ValueError("--solver-velocity-iterations must be in [1, 255]")

    if args.command == "formula-check":
        formula_check()
        return 0

    # SimulationApp forwards remaining argv to Kit. Some command names, notably
    # "simulate", can be interpreted by Kit before our script continues.
    sys.argv = [sys.argv[0]]

    generated: list[dict[str, Any]] = []
    measurements: list[dict[str, Any]] = []
    simulation_app = None

    needs_isaac = args.command in {"generate", "simulate", "all"}
    if needs_isaac:
        from isaacsim import SimulationApp

        simulation_app = SimulationApp({"headless": not args.gui})
        print("[runner] SimulationApp ready", flush=True)

    try:
        if args.command in {"generate", "all"}:
            print("[runner] generating USD files", flush=True)
            generated = generate_usd_files(
                benchmarks,
                models,
                n_links_values,
                supports,
                joint_models,
                args.collisions,
                args.backend,
            )

        audits: list[dict[str, Any]] = []
        if args.command != "report":
            print("[runner] auditing USD files", flush=True)
            audits = audit_usd_files(
                benchmarks,
                models,
                n_links_values,
                supports,
                joint_models,
                args.collisions,
                args.backend,
            )

        if args.command in {"simulate", "all"}:
            print("[runner] running simulations", flush=True)
            measurements = simulate_all(
                simulation_app,
                benchmarks,
                models,
                n_links_values,
                supports,
                joint_models,
                scenarios,
                force_point=args.force_point,
                collisions_enabled=args.collisions,
                backend=args.backend,
                max_seconds=args.max_seconds,
                physics_hz_values=physics_hz_values,
                substeps=args.substeps,
                settle_window_seconds=args.settle_window_seconds,
                settle_tolerance_m=args.settle_tolerance_m,
                solver_position_iterations=args.solver_position_iterations,
                solver_velocity_iterations=args.solver_velocity_iterations,
                sample_hz=args.sample_hz,
                render=args.gui,
            )
            write_last_run_checkpoint(generated, audits, measurements)

        if args.command == "report":
            payload = load_existing_payload()
            report_n_links = tuple(
                sorted(
                    {
                        int(row["n_links"])
                        for row in [*payload.get("audits", []), *payload.get("measurements", [])]
                        if row.get("n_links") is not None
                    }
                )
            )
            payload["acceptance"] = evaluate_acceptance(
                payload.get("audits", []),
                payload.get("measurements", []),
                report_n_links,
            )
            write_json_results(payload)
            write_measurement_csv(payload.get("measurements", []))
        else:
            if args.append_results and result_json_path().exists():
                existing = load_existing_payload()
                generated = merge_records(
                    existing.get("generated", []),
                    generated,
                    ("benchmark", "model", "support", "joint_model", "n_links"),
                )
                audits = merge_records(
                    existing.get("audits", []),
                    audits,
                    ("benchmark", "model", "support", "joint_model", "n_links"),
                )
                measurements = merge_records(
                    existing.get("measurements", []),
                    measurements,
                    (
                        "benchmark", "model", "support", "joint_model", "n_links",
                        "scenario", "force_point", "backend", "physics_hz",
                        "solver_position_iterations", "solver_velocity_iterations",
                    ),
                )
            payload = build_payload(generated, audits, measurements, n_links_values)
            write_json_results(payload)
            if measurements:
                write_measurement_csv(measurements)

        if args.command in {"report", "all", "simulate"}:
            write_report(payload)
    finally:
        if simulation_app is not None:
            simulation_app.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
