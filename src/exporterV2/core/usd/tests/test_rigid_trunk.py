"""Verify the RigidTrunk builder flag without requiring Isaac Sim."""

from pathlib import Path
import sys

import pytest
from pxr import Usd

try:
    from pxr import PhysxSchema  # noqa: F401
except ImportError:
    pytest.skip("PhysX schema requires Isaac Sim", allow_module_level=True)


SRC_DIR = Path(__file__).resolve().parents[4]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from exporterV2.core.usd import stage as stage_module
from exporterV2.legacy_csv.parser import parse_csv_to_branches


def _trunk(joint_type=None):
    branch = {
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 3,
        "radius": 0.01,
        "height": 0.10,
        "tilt": 0.0,
        "rot": 0.0,
    }
    if joint_type is not None:
        branch["joint_type"] = joint_type
    return branch


def _joint_types(stage: Usd.Stage) -> list[str]:
    return [
        prim.GetTypeName()
        for prim in stage.Traverse()
        if prim.GetTypeName() in {"PhysicsJoint", "PhysicsFixedJoint"}
    ]


def test_rigid_trunk_flag_builds_fixed_internal_joints(tmp_path, monkeypatch):
    monkeypatch.setattr(stage_module.PhysicsRuntimeConfig, "RIGID_TRUNK", True)
    stage, _ = stage_module.build_stage(str(tmp_path / "rigid.usda"), branches=[_trunk()])

    joint_types = _joint_types(stage)
    assert joint_types.count("PhysicsFixedJoint") == 3
    assert "PhysicsJoint" not in joint_types


def test_disabling_rigid_trunk_restores_d6_joints(tmp_path, monkeypatch):
    monkeypatch.setattr(stage_module.PhysicsRuntimeConfig, "RIGID_TRUNK", False)
    stage, _ = stage_module.build_stage(str(tmp_path / "flexible.usda"), branches=[_trunk()])

    joint_types = _joint_types(stage)
    assert joint_types.count("PhysicsFixedJoint") == 1
    assert joint_types.count("PhysicsJoint") == 2


def test_explicit_branch_joint_type_overrides_global_flag(tmp_path, monkeypatch):
    monkeypatch.setattr(stage_module.PhysicsRuntimeConfig, "RIGID_TRUNK", True)
    stage, _ = stage_module.build_stage(
        str(tmp_path / "override.usda"),
        branches=[_trunk(joint_type="d6")],
    )

    assert _joint_types(stage).count("PhysicsJoint") == 2


def test_real_tomato_plant_keeps_trunk_fixed_and_truss_dynamic(tmp_path):
    branches, terminal_bodies, _ = parse_csv_to_branches(
        day=80,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )
    trunk = next(branch for branch in branches if branch.get("parent") is None)
    assert trunk["joint_type"] == "fixed"
    assert terminal_bodies

    stage, _ = stage_module.build_stage(
        str(tmp_path / "real_tomato_rigid_trunk.usda"),
        branches=branches,
        terminal_bodies=terminal_bodies,
        skip_limit_check=True,
    )
    trunk_joint_types = [
        prim.GetTypeName()
        for prim in stage.Traverse()
        if "/trunk_Link_" in str(prim.GetPath())
        and prim.GetTypeName() in {"PhysicsJoint", "PhysicsFixedJoint"}
    ]
    pedicel_d6 = [
        prim
        for prim in stage.Traverse()
        if "_pedicel_" in str(prim.GetPath()) and prim.GetTypeName() == "PhysicsJoint"
    ]

    assert trunk_joint_types
    assert set(trunk_joint_types) == {"PhysicsFixedJoint"}
    assert pedicel_d6
