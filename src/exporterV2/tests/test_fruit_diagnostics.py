"""Numerical monitor gates and topology-preserving fruit ablations."""
import json
from pathlib import Path

import numpy as np
import pytest
from pxr import Usd, UsdPhysics

from exporterV2.fruit_diagnostics import attachment_errors, evaluate_tail, json_finite, motion_rates, performance_summary, unexpected_breaks
from exporterV2.fruit_experiments import audit, prepare_stage
from exporterV2.config import physics_config_from_dict, visuals_config_from_dict
from exporterV2.exporter import export_plant
from exporterV2.skeleton_branches import build_branch_plan
from plant_skeleton import load_skeleton

ROOT = Path(__file__).resolve().parents[3]


def test_headless_throughput_is_not_rendered_fps():
    result = performance_summary(600, 10, 20, [])
    assert result["physics_steps_per_wall_second"] == 30
    assert result["real_time_factor"] == .5
    assert result["gui_fps"] is None


def test_rendered_fps_includes_physics_and_monitor_time():
    # Rendering alone takes 10 ms, but the whole frame takes 50 ms.
    result = performance_summary(600, 10, 30, [[5., 20., .05, .01], [5.02, 20.05, .05, .01]])
    assert result["gui_steady_fps"] == pytest.approx(20)
    assert result["gui_steady_p05_fps"] == pytest.approx(20)


def test_report_serializes_numpy_break_flags_and_infinite_limits():
    report = json_finite({"continuity_passed": np.bool_(True), "limit": np.float32(np.inf)})
    assert json.loads(json.dumps(report, allow_nan=False)) == {"continuity_passed": True, "limit": "inf"}


def test_common_motion_is_not_detachment():
    pos = np.array([[12., 0, 0], [12., 0, 1]])
    q = np.array([[1., 0, 0, 0], [1., 0, 0, 0]])
    errors = attachment_errors(pos, q, [0], [1], np.array([[0., 0, 1]]), np.zeros((1, 3)), q[:1], q[:1])
    assert errors[0] == pytest.approx([0])
    assert errors[1] == pytest.approx([0])
    pos[1, 0] += .01
    assert attachment_errors(pos, q, [0], [1], np.array([[0., 0, 1]]), np.zeros((1, 3)), q[:1], q[:1])[0] == pytest.approx([.01])


def test_high_frequency_jitter_does_not_alias_into_a_pass():
    t = np.arange(4800) / 480
    positions = np.zeros((len(t), 1, 3))
    positions[:, 0, 0] = .0004 * np.sin(2 * np.pi * 60 * t)
    linear = .0004 * 2 * np.pi * 60 * abs(np.cos(2 * np.pi * 60 * t))
    metrics, _, errors = evaluate_tail(positions, linear, np.zeros(len(t)), np.zeros(len(t)), np.zeros(len(t)), True)
    assert metrics["position_excursion_m"] < .001
    assert any("linear_speed" in e for e in errors)


def test_short_or_incomplete_run_cannot_pass():
    _, _, errors = evaluate_tail(np.zeros((2, 1, 3)), [0], [0], [0], [0], False)
    assert "requested duration not completed" in errors


def test_force_target_break_before_stimulation_is_spontaneous():
    assert unexpected_breaks({"target"}, "target", .1, 30, True) == {"target"}
    assert unexpected_breaks({"target"}, "target", 31, 30, True) == set()
    assert unexpected_breaks({"target", "other"}, "target", 31, 30, True) == {"other"}
    assert unexpected_breaks({"manual"}, None, 31, 30, False) == set()


def test_motion_rates_use_com_and_ignore_quaternion_sign():
    p = np.zeros((1, 3))
    q = np.array([[1., 0, 0, 0]])
    linear, angular = motion_rates(p, q, p, -q, np.array([[0., 0, 1.]]), .01)
    assert linear == pytest.approx([0])
    assert angular == pytest.approx([0])
    angle = .01
    q1 = np.array([[np.cos(angle / 2), np.sin(angle / 2), 0, 0]])
    linear, angular = motion_rates(p, q, p, q1, np.array([[0., 0, 1.]]), .01)
    assert linear == pytest.approx([1], rel=1e-4)
    assert angular == pytest.approx([1])


def test_motion_rates_accept_isaac_singleton_com_tensor_without_cross_body_broadcast():
    before = np.zeros((2, 3))
    after = np.array([[.01, 0, 0], [0, .02, 0]])
    quat = np.array([[1., 0, 0, 0], [1., 0, 0, 0]])
    com = np.array([[[0., 0, 1.]], [[0., 0, 2.]]])
    linear, angular = motion_rates(before, quat, after, quat, com, .01)
    assert linear.shape == angular.shape == (2,)
    assert linear == pytest.approx([1, 2])
    assert angular == pytest.approx([0, 0])


def _skeleton(day):
    return load_skeleton(ROOT / f"input/examples/tomato_day{day}.json")


def test_fruit_config_reaches_terminal_bodies():
    skeleton = _skeleton(25)
    visuals = visuals_config_from_dict()
    physics = physics_config_from_dict({"fruits": {"mode": "physical", "break_force_n": 8.5}})
    bodies = build_branch_plan(skeleton, physics, visuals).terminal_bodies
    assert bodies
    assert all(b["break_force"] == 8.5 and b["exclude_from_articulation"] for b in bodies)
    physics = physics_config_from_dict({"fruits": {"mode": "physical", "detachable": False}})
    bodies = build_branch_plan(skeleton, physics, visuals).terminal_bodies
    assert all(b["break_force"] is None and not b["detachment_enabled"] for b in bodies)


@pytest.fixture(scope="module")
def source(tmp_path_factory):
    path = tmp_path_factory.mktemp("fruit-source") / "source.usda"
    export_plant(
        _skeleton(50),
        path,
        physics_config_from_dict({"fruits": {"mode": "physical"}}),
        visuals_config_from_dict({"leaves": {"generator": "simple"}}),
    )
    return path


@pytest.mark.parametrize("scenario,internal", [("full", False), ("no-fruit", False), ("single", False), ("truss", False), ("trusses", False), ("stem-truss", False), ("single", True)])
def test_ablation_preserves_rest_frames_and_retained_mass(source, scenario, internal):
    original = Usd.Stage.Open(str(source))
    stage = Usd.Stage.Open(original.Flatten())
    before = audit(stage)
    config = dict(scenario=scenario, attachment="internal" if internal else "external", breakable=not internal,
                  stiffness_scale=1, damping_ratio=4, solver="TGS", gpu=True, hz=480,
                  art_position=32, art_velocity=4, fruit_position=32, fruit_velocity=1)
    result = prepare_stage(stage, config)
    assert result["errors"] == []
    if scenario == "stem-truss":
        assert result["reanchored_joints"] == []
        stem_paths = {str(p.GetPath()) for p in original.Traverse()
                      if p.GetAttribute("autotom:branchKind").Get() == "stem"}
        assert stem_paths
        assert all(stage.GetPrimAtPath(path) for path in stem_paths)
    old_masses = {x["body"]: x["mass_kg"] for x in before["masses"]}
    for record in result["masses"]:
        if record["body"] in old_masses:
            assert record["mass_kg"] == old_masses[record["body"]]
    if scenario == "no-fruit":
        assert result["attachments"] == []
    if scenario == "single":
        assert result["body_count"] == 3
        assert len(result["attachments"]) == 1
    if scenario != "full":
        assert result["body_count"] < before["body_count"]
    for record in result["attachments"]:
        joint = UsdPhysics.Joint(stage.GetPrimAtPath(record["joint"]))
        assert joint.GetExcludeFromArticulationAttr().Get() is not internal
    assert audit(original) == before


def test_stiffness_change_preserves_damping_ratio_and_mass(source):
    stage = Usd.Stage.Open(str(source))
    before = audit(stage)
    drives = {str(p.GetPath()): (UsdPhysics.DriveAPI(p, "rotX").GetStiffnessAttr().Get(),
                               UsdPhysics.DriveAPI(p, "rotX").GetDampingAttr().Get())
              for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)
              and UsdPhysics.DriveAPI(p, "rotX")}
    config = dict(scenario="truss", attachment="external", breakable=True,
                  stiffness_scale=.25, damping_ratio=4, solver="PGS", gpu=True, hz=480,
                  art_position=32, art_velocity=0, fruit_position=32, fruit_velocity=0)
    result = prepare_stage(stage, config)
    for path, (stiffness, damping) in drives.items():
        prim = stage.GetPrimAtPath(path)
        if prim:
            drive = UsdPhysics.DriveAPI(prim, "rotX")
            assert drive.GetStiffnessAttr().Get() == pytest.approx(stiffness * .25)
            assert drive.GetDampingAttr().Get() == pytest.approx(damping * .5)
    masses = {r['body']: r['mass_kg'] for r in before['masses']}
    assert all(r['mass_kg'] == masses[r['body']] for r in result['masses'])
    assert stage.GetPrimAtPath('/World/PhysicsScene').GetAttribute('physxScene:solverType').Get() == 'PGS'
