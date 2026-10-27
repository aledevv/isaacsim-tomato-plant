from __future__ import annotations

import json

import pytest
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

from exporterV2.performance_benchmark import (
    SCHEMA_VERSION,
    _labeled_stage,
    _physics_hz_values,
    build_comparison,
    collect_stage_statistics,
    save_report,
)


def test_benchmark_argument_values_are_explicit_and_ordered(tmp_path):
    label, path = _labeled_stage(f"legacy-v2={tmp_path / 'legacy.usda'}")
    assert label == "legacy-v2"
    assert path == (tmp_path / "legacy.usda").resolve()
    assert _physics_hz_values("60,120,240,480") == (60, 120, 240, 480)


@pytest.mark.parametrize("value", ["", "30", "60,60", "60,bad"])
def test_benchmark_rejects_invalid_rate_sets(value):
    with pytest.raises(Exception):
        _physics_hz_values(value)


def test_stage_statistics_are_deterministic(tmp_path):
    path = tmp_path / "stage.usda"
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.Xform.Define(stage, "/World")
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.GetPrim().CreateAttribute(
        "physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Int
    ).Set(480)
    body = UsdGeom.Xform.Define(stage, "/World/Body").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(body)
    mesh = UsdGeom.Mesh.Define(stage, "/World/Body/Mesh")
    mesh.CreatePointsAttr().Set(
        Vt.Vec3fArray(
            [
                Gf.Vec3f(0.0, 0.0, 0.0),
                Gf.Vec3f(1.0, 0.0, 0.0),
                Gf.Vec3f(0.0, 1.0, 0.0),
            ]
        )
    )
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray([3]))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2]))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    stage.DefinePrim("/World/Joint", "PhysicsJoint")
    stage.GetRootLayer().Save()

    first = collect_stage_statistics(path)
    second = collect_stage_statistics(path)
    assert first == second
    assert first["authored_physics_hz"] == 480
    assert first["rigid_bodies"] == 1
    assert first["collision_shapes"] == 1
    assert first["d6_joints"] == 1
    assert first["mesh_points"] == 3
    assert first["mesh_triangles"] == 1


def test_comparison_pairs_rates_and_reports_speedups():
    baseline = [
        {
            "physics_hz": 60,
            "render_updates_per_second": 10.0,
            "physics_steps_per_second": 20.0,
        },
        {
            "physics_hz": 480,
            "render_updates_per_second": 2.0,
            "physics_steps_per_second": 10.0,
        },
    ]
    candidate = [
        {
            "physics_hz": 60,
            "render_updates_per_second": 50.0,
            "physics_steps_per_second": 80.0,
        },
        {
            "physics_hz": 480,
            "render_updates_per_second": 12.0,
            "physics_steps_per_second": 70.0,
        },
    ]
    comparison = build_comparison(baseline, candidate)
    assert comparison[0]["candidate_to_baseline_render_ratio"] == 5.0
    assert comparison[1]["candidate_to_baseline_physics_ratio"] == 7.0
    assert all(item["candidate_render_faster"] for item in comparison)
    assert all(item["candidate_physics_faster"] for item in comparison)


def test_report_serialization_is_stable_and_strict(tmp_path):
    report = {
        "schema_version": SCHEMA_VERSION,
        "configuration": {"physics_hz": [60, 480]},
        "stages": [],
        "comparison": [],
    }
    path = tmp_path / "report.json"
    save_report(report, path)
    first = path.read_bytes()
    save_report(report, path)
    assert path.read_bytes() == first
    assert first.endswith(b"\n")
    assert json.loads(first)["schema_version"] == SCHEMA_VERSION
