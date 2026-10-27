from __future__ import annotations


import pytest

from pathlib import Path
from exporterV2.core.optimizations.techniques.truss_static import PedicelLockTechnique
from exporterV2.skeleton_branches import BranchPlan, SkeletonBranchesError
from exporterV2.joint_budget import (
    optimize_branches,
    budget_config,
)


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def _branch(branch_id, parent, kind, n_links=1, **extra):
    return {
        "id": branch_id,
        "parent": parent,
        "attach_link": 1,
        "n_links": n_links,
        "radius": 0.005,
        "height": 0.02,
        "kind": kind,
        "joint_type": "d6",
        "attachment_joint_type": "d6",
        **extra,
    }


def _result():
    branches = [
        _branch("trunk", None, "stem", n_links=4, joint_type="fixed", attachment_joint_type="fixed"),
        _branch("Leaf_r0_o0_g1_petiole", "trunk", "leaf_petiole"),
        _branch("Leaf_r0_o0_g1_rachis", "Leaf_r0_o0_g1_petiole", "leaf_rachis", n_links=2),
        _branch("LeafVisual_g1_petiolule_left_01_physical", "Leaf_r0_o0_g1_rachis", "petiolule"),
        _branch("LeafVisual_g1_petiolule_right_01_physical", "Leaf_r0_o0_g1_rachis", "petiolule"),
        # Terminal petiolules are only recognisable by their kind.
        _branch("LeafVisual_g1_rachis_terminal_01_physical", "Leaf_r0_o0_g1_rachis", "petiolule"),
        _branch("Truss_r1_o0_g2_rachis", "trunk", "truss_rachis", n_links=3, physics_profile="truss"),
        *[
            _branch(f"Truss_r1_o0_g2_pedicel_0{i}", "Truss_r1_o0_g2_rachis", "pedicel", physics_profile="truss")
            for i in range(1, 3)
        ],
        _branch("Truss_r2_o0_g3_rachis", "trunk", "truss_rachis", n_links=3, physics_profile="truss"),
        *[
            _branch(f"Truss_r2_o0_g3_pedicel_0{i}", "Truss_r2_o0_g3_rachis", "pedicel", physics_profile="truss")
            for i in range(1, 3)
        ],
    ]
    return BranchPlan(
        branches=tuple(branches),
        rigid_leaf_visuals=(),
        terminal_bodies=(),
        attachment_map=(),
        pose="canonical",
        visual_quality="realistic",
        fruit_mode="none",
    )


def _d6(result):
    return sum(b["n_links"] for b in result.branches if b["joint_type"] != "fixed")


def test_within_budget_is_unchanged():
    result = _result()
    assert _d6(result) == 16
    optimized, report = optimize_branches(result, 16)
    assert optimized.branches == result.branches
    assert report["final_d6"] == 16
    assert report["locked_branch_ids"] == []


def test_petiolules_lock_by_kind_and_sync_attachment_joint():
    optimized, report = optimize_branches(_result(), 13)
    by_id = {b["id"]: b for b in optimized.branches}
    for suffix in ("petiolule_left_01", "petiolule_right_01", "rachis_terminal_01"):
        branch = by_id[f"LeafVisual_g1_{suffix}_physical"]
        assert branch["joint_type"] == "fixed"
        assert branch["attachment_joint_type"] == "fixed"
    assert by_id["Leaf_r0_o0_g1_rachis"]["joint_type"] == "d6"
    assert report["final_d6"] == 13
    assert report["techniques"]["petiole_lock"]["joints_saved"] == 3
    assert "pedicel_lock" not in report["techniques"]


def test_pedicels_lock_one_truss_per_pass_without_touching_rachis():
    original = _result()
    optimized, report = optimize_branches(original, 11)
    by_id = {b["id"]: b for b in optimized.branches}
    assert report["final_d6"] == 11
    assert report["techniques"]["pedicel_lock"]["passes"] == 1
    locked = [b["id"] for b in optimized.branches if "_pedicel_" in b["id"] and b["joint_type"] == "fixed"]
    assert locked == ["Truss_r1_o0_g2_pedicel_01", "Truss_r1_o0_g2_pedicel_02"]
    assert by_id["Truss_r1_o0_g2_rachis"] == dict(original.branches[6])
    assert len(optimized.branches) == len(original.branches)


def test_unreachable_budget_reports_locking_minimum():
    with pytest.raises(SkeletonBranchesError, match=r"minimum possible \(9 joints\)"):
        optimize_branches(_result(), 8)


def test_unknown_techniques_are_rejected():
    with pytest.raises(SkeletonBranchesError, match="does not implement"):
        budget_config(100, ("petiole_lock", "magic_reduce"))
    with pytest.raises(SkeletonBranchesError, match="technique set"):
        budget_config(100, "everything")


def test_full_set_follows_yaml_priorities():
    config = budget_config(100, "full")
    assert [entry["id"] for entry in config.techniques] == [
        "thin_link_lock", "petiole_lock", "lateral_reduce",
        "stem_collapse", "truss_static", "leaf_branch_reduce",
    ]


def test_pedicel_lock_never_builds_static_rachis_curves():
    branches = [dict(b, joint_type="fixed") if "_pedicel_" in b["id"] else dict(b) for b in _result().branches]
    assert not PedicelLockTechnique().can_apply(branches)


def _export_day_160(tmp_path, joints):
    from exporterV2.config import physics_config_from_dict, visuals_config_from_dict
    from exporterV2.exporter import export_plant
    from plant_skeleton import load_skeleton

    return export_plant(
        load_skeleton(PROJECT_ROOT / "input" / "skeleton.json"),
        tmp_path / "day160.usda",
        physics_config_from_dict({"joints": joints, "fruits": {"mode": "none"}}),
        visuals_config_from_dict({"leaves": {"generator": "simple"}}),
    )


def test_day_160_lock_export_fits_the_joint_target(tmp_path):
    from pxr import Usd

    result = _export_day_160(tmp_path, {"joint_target": 150, "optimize": "lock"})
    optimization = result.manifest["metadata"]["joint_budget_optimization"]
    assert optimization["original_d6"] == 206
    assert optimization["final_d6"] <= 150
    stage = Usd.Stage.Open(str(result.usd_path))
    assert sum(prim.GetTypeName() == "PhysicsJoint" for prim in stage.Traverse()) == optimization["final_d6"]


def test_day_160_full_export_merges_bodies(tmp_path):
    from pxr import Usd, UsdPhysics

    result = _export_day_160(tmp_path, {"joint_target": 60, "optimize": "full"})
    optimization = result.manifest["metadata"]["joint_budget_optimization"]
    merge = optimization["body_merge"]
    assert optimization["final_d6"] == merge["after"]["d6_joints"] == result.d6_joints <= 60
    assert merge["after"]["meshes"] == merge["before"]["meshes"]
    assert merge["after"]["colliders"] == merge["before"]["colliders"]
    stage = Usd.Stage.Open(str(result.usd_path))
    bodies = sum(prim.HasAPI(UsdPhysics.RigidBodyAPI) for prim in stage.Traverse())
    assert bodies == merge["after"]["rigid_bodies"] < 216
