from copy import deepcopy
from pathlib import Path
import sys

import pytest

try:
    from pxr import PhysxSchema, UsdPhysics
except ImportError:
    pytest.skip("PhysX schema requires Isaac Sim", allow_module_level=True)


SRC_DIR = Path(__file__).resolve().parents[5]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from exporterV2.tests.legacy_csv.truss_builder import truss_to_complete_config
from exporterV2.core.optimizations.techniques.truss_static import TrussStaticTechnique
from exporterV2.core.tree_config import PhysicsRuntimeConfig, TrussPhysicsConfig
from exporterV2.core.usd.stage import build_stage


def make_truss() -> tuple[list[dict], list[dict]]:
    trunk = {
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 1,
        "radius": 0.01,
        "height": 0.20,
        "tilt": 0.0,
        "rot": 0.0,
    }
    truss = {
        "rachis_length": 0.24,
        "rachis_radius": 0.0025,
        "n_fruits": 7,
        "pedicel_length": 0.035,
        "pedicel_radius": 0.0015,
        "pedicel_angle": 90.0,
        "parent_rank": 0,
        "tilt_deg": 58.0,
        "azimuth_deg": 90.0,
        "tomato_radii": [0.018] * 7,
    }
    truss_branches, tomatoes = truss_to_complete_config(truss, "trunk", rank=0)
    terminal_bodies = [
        {
            "id": tomato["id"],
            "parent_branch_id": tomato["pedicel_id"],
            "shape": "sphere",
            "radius": tomato["radius"],
            "mass": tomato["mass"],
        }
        for tomato in tomatoes
    ]
    return [trunk, *truss_branches], terminal_bodies


def test_usd_uses_official_pedicel_and_static_root_overrides(tmp_path, monkeypatch):
    monkeypatch.setattr(TrussPhysicsConfig, "TOMATO_DETACHMENT_ENABLED", True)
    branches, terminal_bodies = make_truss()
    dynamic_stage, stem_path = build_stage(
        str(tmp_path / "dynamic.usda"),
        branches=branches,
        terminal_bodies=terminal_bodies,
        skip_limit_check=True,
    )
    dynamic_stage.GetRootLayer().Save()

    pedicel_joints = [
        prim
        for prim in dynamic_stage.Traverse()
        if prim.GetTypeName() == "PhysicsJoint" and "_pedicel_" in str(prim.GetPath())
    ]
    terminal_joints = [
        prim
        for prim in dynamic_stage.Traverse()
        if prim.GetTypeName() == "PhysicsFixedJoint"
        and prim.GetName() == "TerminalBodyFixedJoint"
    ]
    assert len(pedicel_joints) == 7
    assert len(terminal_joints) == len(terminal_bodies)

    for prim in terminal_joints:
        assert prim.GetAttribute("physics:breakForce").Get() == pytest.approx(
            TrussPhysicsConfig.TOMATO_DETACHMENT_BREAK_FORCE_N
        )
        assert prim.GetAttribute("physics:excludeFromArticulation").Get() is True
        body1_path = str(prim.GetRelationship("physics:body1").GetTargets()[0])
        assert body1_path.startswith(
            TrussPhysicsConfig.TOMATO_DETACHMENT_BODY_PARENT_PATH
        )
        assert not body1_path.startswith(f"{stem_path}/")

        rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Get(dynamic_stage, body1_path)
        assert (
            rigid_body_api.GetSolverPositionIterationCountAttr().Get()
            == PhysicsRuntimeConfig.TERMINAL_BODY_SOLVER_POSITION_ITERATIONS
        )
        assert (
            rigid_body_api.GetSolverVelocityIterationCountAttr().Get()
            == PhysicsRuntimeConfig.TERMINAL_BODY_SOLVER_VELOCITY_ITERATIONS
        )

        filtered_targets = {
            str(path)
            for path in dynamic_stage.GetPrimAtPath(body1_path)
            .GetRelationship("physics:filteredPairs")
            .GetTargets()
        }
        parent_link_path = str(
            prim.GetRelationship("physics:body0").GetTargets()[0]
        )
        assert parent_link_path in filtered_targets
        assert any("_rachis_Link_" in path for path in filtered_targets)

    for prim in pedicel_joints:
        limit = UsdPhysics.LimitAPI.Get(prim, "rotX")
        assert limit.GetLowAttr().Get() == -TrussPhysicsConfig.PEDICEL_BEND_LIMIT_DEG
        assert limit.GetHighAttr().Get() == TrussPhysicsConfig.PEDICEL_BEND_LIMIT_DEG

    unscaled = deepcopy(branches)
    for branch in unscaled:
        if "_pedicel_" in branch["id"]:
            branch["drive_stiffness_scale"] = 1.0
    unscaled_stage, _ = build_stage(
        str(tmp_path / "unscaled.usda"),
        branches=unscaled,
        terminal_bodies=terminal_bodies,
        skip_limit_check=True,
    )
    unscaled_pedicel = next(
        prim
        for prim in unscaled_stage.Traverse()
        if prim.GetTypeName() == "PhysicsJoint" and "_pedicel_" in str(prim.GetPath())
    )
    soft_stiffness = UsdPhysics.DriveAPI.Get(
        pedicel_joints[0], "rotX"
    ).GetStiffnessAttr().Get()
    full_stiffness = UsdPhysics.DriveAPI.Get(
        unscaled_pedicel, "rotX"
    ).GetStiffnessAttr().Get()
    assert soft_stiffness == pytest.approx(
        full_stiffness * TrussPhysicsConfig.PEDICEL_DRIVE_STIFFNESS_SCALE
    )

    technique = TrussStaticTechnique()
    fixed, _ = technique.apply(branches)
    static, _ = technique.apply(fixed)
    static_stage, _ = build_stage(
        str(tmp_path / "static.usda"),
        branches=static,
        terminal_bodies=terminal_bodies,
        skip_limit_check=True,
    )
    static_stage.GetRootLayer().Save()

    truss_d6 = [
        prim
        for prim in static_stage.Traverse()
        if prim.GetTypeName() == "PhysicsJoint" and "_static_curve_" in str(prim.GetPath())
    ]
    assert len(truss_d6) == 1
    root_limit = UsdPhysics.LimitAPI.Get(truss_d6[0], "rotX")
    assert root_limit.GetLowAttr().Get() == -18.0
    assert root_limit.GetHighAttr().Get() == 18.0


def test_detachment_master_switch_suppresses_break_force_only(tmp_path, monkeypatch):
    monkeypatch.setattr(TrussPhysicsConfig, "TOMATO_DETACHMENT_ENABLED", False)
    branches, terminal_bodies = make_truss()
    terminal_bodies = [
        {
            **body,
            "detachment_enabled": True,
            "exclude_from_articulation": True,
            "parent_path": "/World/TerminalBodies",
            "break_force": 1.0,
        }
        for body in terminal_bodies
    ]

    stage, stem_path = build_stage(
        str(tmp_path / "detachment_disabled.usda"),
        branches=branches,
        terminal_bodies=terminal_bodies,
        skip_limit_check=True,
    )
    terminal_joints = [
        prim
        for prim in stage.Traverse()
        if prim.GetTypeName() == "PhysicsFixedJoint"
        and prim.GetName() == "TerminalBodyFixedJoint"
    ]

    assert len(terminal_joints) == len(terminal_bodies)
    for joint in terminal_joints:
        assert not joint.GetAttribute("physics:breakForce").HasAuthoredValue()
        assert joint.GetAttribute("physics:excludeFromArticulation").Get() is True
        body_path = str(joint.GetRelationship("physics:body1").GetTargets()[0])
        assert body_path.startswith(
            TrussPhysicsConfig.TOMATO_DETACHMENT_BODY_PARENT_PATH
        )
        assert not body_path.startswith(f"{stem_path}/")
