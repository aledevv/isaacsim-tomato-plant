import importlib.util
import math
import symtable
import sys
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).with_name("cantilever_validation.py")
SPEC = importlib.util.spec_from_file_location("cantilever_validation_protocol", MODULE_PATH)
validation = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = validation
SPEC.loader.exec_module(validation)

BENCHMARKS = validation.BENCHMARKS
branch_defs_for_benchmark = validation.branch_defs_for_benchmark
config_fingerprint = validation.config_fingerprint
evaluate_acceptance = validation.evaluate_acceptance
experiment_config = validation.experiment_config


def test_stage_units_are_bound_inside_the_simulation_scope():
    table = symtable.symtable(MODULE_PATH.read_text(), str(MODULE_PATH), "exec")
    simulate_table = next(child for child in table.get_children() if child.get_name() == "simulate_one")

    assert simulate_table.lookup("stage_meters_per_unit").is_local()


class _FakePhysicsView:
    count = 1
    max_links = 1

    def __init__(self, transform):
        self._transform = np.asarray([[transform]], dtype=np.float32)

    def get_link_transforms(self):
        return self._transform


class _FakeArticulation:
    def __init__(self, transform):
        self._physics_view = _FakePhysicsView(transform)


def test_articulation_link_transform_uses_physx_xyzw_quaternion_order():
    # Translation (1, 2, 3), followed by +90 degrees about Y as qx, qy, qz, qw.
    half_sqrt_two = math.sqrt(0.5)
    articulation = _FakeArticulation(
        [1.0, 2.0, 3.0, 0.0, half_sqrt_two, 0.0, half_sqrt_two]
    )

    point = validation.articulation_link_world_position(
        articulation,
        link_index=0,
        local_position_m=[0.0, 0.0, 2.0],
    )

    assert np.allclose(point, [3.0, 2.0, 3.0], atol=1e-6)


def test_stage_vectors_are_explicitly_converted_to_si():
    assert np.allclose(
        validation.stage_vector_to_meters([0.0, 0.42, 0.1], 0.01),
        [0.0, 0.0042, 0.001],
    )


def test_discrete_references_match_the_simulated_topology():
    benchmark = BENCHMARKS["synthetic_solid_40cm"]

    assert math.isclose(
        benchmark.expected_discrete_tip_force_mm(3, 0.05),
        0.7545123228060228,
        abs_tol=1e-12,
    )
    assert math.isclose(
        benchmark.expected_discrete_tip_force_mm(
            3,
            0.05,
            support="half_cell",
            force_point="com",
        ),
        1.0940428680687326,
        abs_tol=1e-12,
    )
    assert math.isclose(
        benchmark.expected_discrete_self_weight_mm(20),
        11.332512,
        abs_tol=1e-9,
    )


def test_half_cell_support_has_exact_two_ei_over_link_length_stiffness():
    benchmark = BENCHMARKS["synthetic_solid_40cm"]
    branches = branch_defs_for_benchmark(
        benchmark,
        n_links=10,
        support="half_cell",
        joint_model="d6_biaxial",
        collisions_enabled=False,
    )
    cantilever = branches[1]
    expected = 2.0 * benchmark.flexural_rigidity_nm2 / (benchmark.world_length_m / 10)

    assert cantilever["attachment_joint_type"] == "d6"
    assert math.isclose(cantilever["attachment_stiffness_rad"], expected)
    assert cantilever["collision_enabled"] is False


def test_fixed_support_and_fixed_chain_are_unambiguous():
    benchmark = BENCHMARKS["synthetic_solid_40cm"]
    branches = branch_defs_for_benchmark(
        benchmark,
        n_links=20,
        support="fixed",
        joint_model="fixed_chain",
        collisions_enabled=False,
    )
    cantilever = branches[1]

    assert cantilever["joint_type"] == "fixed"
    assert cantilever["attachment_joint_type"] == "fixed"
    assert "attachment_stiffness_rad" not in cantilever
    assert benchmark.expected_discrete_tip_force_mm(
        20, 0.05, joint_model="fixed_chain"
    ) == 0.0


def test_d6_planar_is_a_distinct_single_axis_diagnostic_model():
    benchmark = BENCHMARKS["synthetic_solid_40cm"]
    branches = branch_defs_for_benchmark(
        benchmark,
        n_links=2,
        support="fixed",
        joint_model="d6_planar",
        collisions_enabled=False,
    )

    assert branches[1]["joint_type"] == "d6_planar"
    assert branches[1]["attachment_joint_type"] == "fixed"


def test_stable_but_physically_wrong_result_is_not_a_pass():
    status = validation.classify_validation_status(
        settled=True,
        nan_detected=False,
        joint_model="d6_planar",
        measured_mm=0.0,
        discrete_error_pct=100.0,
    )

    assert status == "settled_wrong_equilibrium"


def test_fingerprint_changes_with_mechanical_configuration():
    fixed = experiment_config(
        "synthetic_solid_40cm", "new_physics", 20, "fixed", "d6_biaxial", False, "cpu"
    )
    half_cell = {**fixed, "support": "half_cell"}
    gpu = {**fixed, "backend": "gpu"}

    assert config_fingerprint(fixed) != config_fingerprint(half_cell)
    assert config_fingerprint(fixed) != config_fingerprint(gpu)
    assert config_fingerprint(fixed) == config_fingerprint(dict(reversed(list(fixed.items()))))
    assert fixed["schema_version"] == 3
    assert fixed["authored_solver_position_iterations"] == 32
    assert fixed["authored_solver_velocity_iterations"] == 4


def test_tomato_claim_is_blocked_without_synthetic_solver_independence():
    acceptance = evaluate_acceptance([], [], (3, 5, 10, 20))

    assert acceptance["synthetic_validation_passed"] is False
    assert acceptance["tomato_realism_claim_allowed"] is False


def test_parameter_grounded_validation_is_distinct_from_empirical_realism_claim():
    common = {
        "model": "new_physics",
        "support": "fixed",
        "joint_model": "d6_biaxial",
        "n_links": 20,
        "scenario": "tip_force_0p05N",
        "force_point": "geometric_tip",
        "backend": "cpu",
        "settled": True,
        "discrete_error_pct": 1.0,
        "error_pct": 1.0,
    }
    measurements = [
        {**common, "benchmark": "synthetic_solid_40cm", "physics_hz": 960.0, "final_deflection_mm": 1.25},
        {**common, "benchmark": "synthetic_solid_40cm", "physics_hz": 1920.0, "final_deflection_mm": 1.26},
        {**common, "benchmark": "tomato_gao_20cm", "physics_hz": 960.0, "final_deflection_mm": 3.57},
        {**common, "benchmark": "tomato_gao_20cm", "physics_hz": 1920.0, "final_deflection_mm": 3.58},
        {
            **common,
            "benchmark": "tomato_gao_20cm",
            "scenario": "self_weight",
            "physics_hz": 960.0,
            "final_deflection_mm": 3.44,
        },
        {
            **common,
            "benchmark": "tomato_gao_20cm",
            "scenario": "self_weight",
            "physics_hz": 1920.0,
            "final_deflection_mm": 3.45,
        },
    ]

    acceptance = evaluate_acceptance([], measurements, (20,))

    assert acceptance["synthetic_validation_passed"] is True
    assert acceptance["tomato_parameter_grounded_validation_passed"] is True
    assert acceptance["tomato_tip_force_timestep_validation_passed"] is True
    assert acceptance["tomato_self_weight_timestep_validation_passed"] is True
    assert acceptance["tomato_empirical_ground_truth_available"] is False
    assert acceptance["tomato_realism_claim_allowed"] is False


def test_close_self_weight_values_do_not_pass_when_one_timestep_is_not_settled():
    common = {
        "model": "new_physics",
        "support": "fixed",
        "joint_model": "d6_biaxial",
        "n_links": 20,
        "force_point": "geometric_tip",
        "backend": "cpu",
        "discrete_error_pct": 1.0,
        "error_pct": 10.0,
        "expected_discrete_deflection_mm": 3.126,
    }
    measurements = [
        {
            **common,
            "benchmark": "synthetic_solid_40cm",
            "scenario": "tip_force_0p05N",
            "physics_hz": 960.0,
            "final_deflection_mm": 1.25,
            "settled": True,
        },
        {
            **common,
            "benchmark": "synthetic_solid_40cm",
            "scenario": "tip_force_0p05N",
            "physics_hz": 1920.0,
            "final_deflection_mm": 1.26,
            "settled": True,
        },
        {
            **common,
            "benchmark": "tomato_gao_20cm",
            "scenario": "tip_force_0p05N",
            "physics_hz": 960.0,
            "final_deflection_mm": 3.3465,
            "settled": True,
        },
        {
            **common,
            "benchmark": "tomato_gao_20cm",
            "scenario": "tip_force_0p05N",
            "physics_hz": 1920.0,
            "final_deflection_mm": 3.3330,
            "settled": True,
        },
        {
            **common,
            "benchmark": "tomato_gao_20cm",
            "scenario": "self_weight",
            "physics_hz": 960.0,
            "final_deflection_mm": 3.1725,
            "settled": False,
        },
        {
            **common,
            "benchmark": "tomato_gao_20cm",
            "scenario": "self_weight",
            "physics_hz": 1920.0,
            "final_deflection_mm": 3.1372,
            "settled": True,
        },
    ]

    acceptance = evaluate_acceptance([], measurements, (20,))

    assert acceptance["tomato_parameter_grounded_validation_passed"] is True
    assert acceptance["tomato_tip_force_timestep_validation_passed"] is True
    assert acceptance["tomato_self_weight_timestep_validation_passed"] is False


def test_timestep_span_is_normalized_by_the_discrete_reference_when_a_run_is_zero():
    common = {
        "benchmark": "synthetic_solid_40cm",
        "model": "new_physics",
        "support": "fixed",
        "joint_model": "d6_planar",
        "n_links": 2,
        "scenario": "tip_force_0p05N",
        "force_point": "geometric_tip",
        "backend": "cpu",
        "settled": True,
        "discrete_error_pct": 0.0,
        "error_pct": 0.0,
        "expected_discrete_deflection_mm": 0.5,
    }
    measurements = [
        {**common, "physics_hz": 120.0, "final_deflection_mm": 0.5},
        {**common, "physics_hz": 480.0, "final_deflection_mm": 0.0},
    ]

    acceptance = evaluate_acceptance([], measurements, (2,))
    timestep = next(iter(acceptance["timestep_independence_within_5pct"].values()))

    assert timestep["relative_span"] == 1.0
    assert timestep["finest_pair_relative_difference"] == 1.0
    assert timestep["ok"] is False


def test_result_merge_replaces_only_the_same_experimental_key():
    key_fields = ("n_links", "physics_hz")
    existing = [
        {"n_links": 10, "physics_hz": 240.0, "value": "old"},
        {"n_links": 20, "physics_hz": 240.0, "value": "keep"},
    ]
    new = [
        {"n_links": 10, "physics_hz": 240.0, "value": "new"},
        {"n_links": 10, "physics_hz": 480.0, "value": "add"},
    ]

    merged = validation.merge_records(existing, new, key_fields)
    by_key = {(row["n_links"], row["physics_hz"]): row["value"] for row in merged}

    assert by_key == {(10, 240.0): "new", (20, 240.0): "keep", (10, 480.0): "add"}


def test_acceptance_can_merge_an_audit_from_before_si_consistency_field():
    audit = {
        "exists": True,
        "benchmark": "synthetic_solid_40cm",
        "model": "new_physics",
        "support": "fixed",
        "joint_model": "d6_biaxial",
        "n_links": 3,
        "collisions_enabled": False,
        "collision_api_count": 0,
        "internal_stiffness_rel_error": 0.0,
        "base_stiffness_rel_error": None,
        "solver_position_iterations": 32,
        "solver_velocity_iterations": 4,
        "gpu_dynamics_enabled": False,
        "backend": "cpu",
        "fingerprint_ok": True,
        "drive_count_ok": True,
        "units_ok": True,
        "support_joint_ok": True,
        "meters_per_unit": 1.0,
        "kilograms_per_unit": 1.0,
        "total_length_m": 0.4,
        "length_error_m": 0.0,
        "total_branch_mass_kg": 0.125,
        "expected_branch_mass_kg": 0.125,
        "mass_error_kg": 0.0,
    }

    acceptance = evaluate_acceptance([audit], [], (3,))

    physical = next(iter(acceptance["benchmark_physical_consistency"].values()))
    assert physical["ok"] is True
