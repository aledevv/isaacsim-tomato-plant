import math
import importlib
from types import SimpleNamespace

import pytest
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from exporterV2.core.skinning import (
    SkinningRuntime,
    branch_system,
    build_skinned_vegetative_structure,
    partition_branches,
    resolve_vegetative_graph,
)
from exporterV2.core.tree_config import (
    TrussPhysicsConfig,
    calculate_physics_params,
    compute_mass,
    scaled,
)

try:
    from pxr import PhysxSchema  # noqa: F401
except ImportError:
    HAS_PHYSX_SCHEMA = False
else:
    HAS_PHYSX_SCHEMA = True
    from exporterV2.core.usd.stage import build_stage


def _load_build_stage():
    if HAS_PHYSX_SCHEMA:
        return build_stage

    # Standard OpenUSD can validate hybrid topology; only Isaac-specific solver
    # APIs are unavailable and are not called by this test fixture.
    import pxr

    class _PhysxRigidBodyAPI:
        def __init__(self, prim):
            self.prim = prim

        @classmethod
        def Apply(cls, prim):
            return cls(prim)

        def CreateSolverPositionIterationCountAttr(self):
            return self.prim.CreateAttribute(
                "physxRigidBody:solverPositionIterationCount",
                Sdf.ValueTypeNames.Int,
            )

        def CreateSolverVelocityIterationCountAttr(self):
            return self.prim.CreateAttribute(
                "physxRigidBody:solverVelocityIterationCount",
                Sdf.ValueTypeNames.Int,
            )

    pxr.PhysxSchema = SimpleNamespace(PhysxRigidBodyAPI=_PhysxRigidBodyAPI)
    return importlib.import_module("exporterV2.core.usd.stage").build_stage


def _root(**overrides):
    branch = {
        "id": "trunk",
        "system": "vegetative",
        "parent": None,
        "attach_link": None,
        "n_links": 2,
        "radius": 0.01,
        "height": 0.10,
        "tilt": 0.0,
        "rot": 0.0,
        "joint_type": "fixed",
    }
    branch.update(overrides)
    return branch


def _child(**overrides):
    branch = {
        "id": "lateral",
        "system": "vegetative",
        "parent": "trunk",
        "attach_link": 2,
        "attach_frac": 0.25,
        "n_links": 2,
        "radius": 0.004,
        "height": 0.04,
        "tilt": 0.0,
        "rot": 0.0,
        "joint_type": "d6",
    }
    branch.update(overrides)
    return branch


def _secondary(**overrides):
    branch = _child(
        id="secondary",
        parent="lateral",
        attach_link=2,
        attach_frac=1.0,
        n_links=1,
        tilt=30.0,
        rot=120.0,
    )
    branch.update(overrides)
    return branch


def _build_skinned_stage(path, branches):
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.Xform.Define(stage, "/World")
    stem = UsdGeom.Xform.Define(stage, "/World/Stem")
    UsdPhysics.ArticulationRootAPI.Apply(stem.GetPrim())
    build_skinned_vegetative_structure(
        stage,
        "/World/Stem",
        branches,
        all_branch_defs={branch["id"]: branch for branch in branches},
    )
    return stage


def test_classification_and_truss_partition():
    legacy_vegetative = _root()
    legacy_vegetative.pop("system")
    legacy_truss = _child(id="truss", physics_profile="truss")
    legacy_truss.pop("system")

    assert branch_system(legacy_vegetative) == "vegetative"
    assert branch_system(legacy_truss) == "truss"
    vegetative, truss = partition_branches([legacy_vegetative, legacy_truss])
    assert vegetative == [legacy_vegetative]
    assert truss == [legacy_truss]


def test_rejects_vegetative_child_of_truss():
    truss = _root(id="truss", system="truss", physics_profile="truss")
    child = _child(parent="truss")
    with pytest.raises(ValueError, match="cannot have truss parent"):
        resolve_vegetative_graph(
            [child],
            all_branch_defs={"truss": truss, "lateral": child},
        )


def test_csv_parser_persists_system_classification():
    from exporterV2.legacy_csv import parse_csv_to_branches

    branches, _, _ = parse_csv_to_branches(
        day=40,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )
    assert all(branch.get("system") in ("vegetative", "truss") for branch in branches)
    assert all(
        branch["system"] == "truss"
        for branch in branches
        if branch.get("physics_profile") == "truss"
    )
    assert all(
        branch["system"] == "vegetative"
        for branch in branches
        if branch.get("physics_profile") != "truss"
    )


@pytest.mark.parametrize("day", (1, 40))
def test_real_csv_vegetative_graph_resolves_without_truss(day):
    from exporterV2.legacy_csv import parse_csv_to_branches

    branches, _, _ = parse_csv_to_branches(
        day=day,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )
    vegetative, truss = partition_branches(branches)
    resolved = resolve_vegetative_graph(
        vegetative,
        all_branch_defs={branch["id"]: branch for branch in branches},
    )
    assert {branch.branch_id for branch in resolved} == {
        branch["id"] for branch in vegetative
    }
    assert all(branch.spec.physics_links == branch.n_links for branch in resolved)
    assert not any(branch.branch_id in {item["id"] for item in truss} for branch in resolved)


def test_adapter_preserves_scale_attachment_mass_and_gains():
    branches = [_root(), _child()]
    resolved = resolve_vegetative_graph(
        branches,
        all_branch_defs={branch["id"]: branch for branch in branches},
    )
    root, child = resolved

    assert root.n_links == branches[0]["n_links"]
    assert child.n_links == branches[1]["n_links"]
    assert child.start == Gf.Vec3d(0.0, 0.0, scaled(0.10) * 1.25)
    assert child.radius == scaled(branches[1]["radius"])
    expected_mass = compute_mass(scaled(0.004), scaled(0.04))
    expected_k, expected_d = calculate_physics_params(
        scaled(0.004),
        scaled(0.04),
        expected_mass,
    )
    assert math.isclose(child.mass, expected_mass)
    assert math.isclose(child.gains.stiffness, expected_k)
    assert math.isclose(child.gains.damping, expected_d)

    tip_child = _child(attach_frac=1.0)
    tip = resolve_vegetative_graph(
        [_root(), tip_child],
        all_branch_defs={"trunk": branches[0], "lateral": tip_child},
    )[1]
    assert tip.start == Gf.Vec3d(0.0, 0.0, scaled(0.10) * 2.0)


@pytest.mark.parametrize(
    ("joint_type", "usd_type"),
    [
        ("fixed", "PhysicsFixedJoint"),
        ("d6", "PhysicsJoint"),
        ("d6_planar", "PhysicsJoint"),
        ("revolute_planar", "PhysicsRevoluteJoint"),
    ],
)
def test_authors_all_supported_joint_types(tmp_path, joint_type, usd_type):
    branches = [_root(), _child(joint_type=joint_type, attachment_joint_type=joint_type)]
    stage = _build_skinned_stage(tmp_path / f"{joint_type}.usda", branches)
    child_path = "/World/Stem/Vegetative/lateral/lateral_Link_01"
    assert stage.GetPrimAtPath(f"{child_path}/AttachJoint").GetTypeName() == usd_type
    if joint_type == "d6_planar":
        joint_prim = stage.GetPrimAtPath(f"{child_path}/AttachJoint")
        assert UsdPhysics.DriveAPI.Get(joint_prim, "rotX").GetStiffnessAttr().HasAuthoredValue()
        assert not UsdPhysics.DriveAPI.Get(joint_prim, "rotY").GetStiffnessAttr().HasAuthoredValue()


def test_smoke_stage_has_capsules_skeleton_relations_and_runtime(tmp_path):
    stage = _build_skinned_stage(
        tmp_path / "smoke.usda",
        [_root(), _child(tilt=40.0, rot=90.0), _secondary()],
    )
    link_path = "/World/Stem/Vegetative/lateral/lateral_Link_01"
    capsule = stage.GetPrimAtPath(f"{link_path}/Collider_01")
    assert capsule.IsA(UsdGeom.Capsule)
    assert UsdGeom.Imageable(capsule).ComputeVisibility() == UsdGeom.Tokens.invisible
    assert not stage.GetPrimAtPath(f"{link_path}/Cylinder").IsValid()
    assert stage.GetPrimAtPath("/World/PlantVisual/lateral/SkelRoot").IsValid()

    stage.GetRootLayer().Save()
    reopened = Usd.Stage.Open(str(tmp_path / "smoke.usda"))
    runtime = SkinningRuntime.discover(reopened)
    assert runtime.branch_count == 3
    runtime.sync()
    for branch in runtime.branches:
        values = branch.translations_attr.Get()
        assert len(values) == len(branch.link_prims)
        assert all(math.isfinite(component) for value in values for component in value)


def test_hybrid_keeps_truss_and_tomato_on_legacy_backend(tmp_path):
    truss = {
        "id": "Truss_01_rachis",
        "system": "truss",
        "physics_profile": "truss",
        "parent": "trunk",
        "attach_link": 2,
        "n_links": 1,
        "radius": 0.002,
        "height": 0.04,
        "tilt": 45.0,
        "rot": 20.0,
    }
    pedicel = {
        "id": "Truss_01_pedicel_terminal",
        "system": "truss",
        "physics_profile": "truss",
        "parent": truss["id"],
        "attach_link": 1,
        "n_links": 1,
        "radius": 0.001,
        "height": 0.01,
        "tilt": 20.0,
        "rot": 0.0,
    }
    tomato = {
        "id": "Tomato_01",
        "shape": "sphere",
        "parent_branch_id": pedicel["id"],
        "radius": 0.01,
        "mass": 0.02,
    }
    hybrid_build_stage = _load_build_stage()
    stage, _ = hybrid_build_stage(
        str(tmp_path / "hybrid.usda"),
        branches=[_root(), truss, pedicel],
        terminal_bodies=[tomato],
        branch_backend="skinned",
        skinning_visual_mode="segmented",
    )

    assert stage.GetPrimAtPath(
        "/World/Stem/Vegetative/trunk/trunk_Link_01/OrganicVisual_01"
    ).IsValid()
    truss_link = f"/World/Stem/{truss['id']}_Link_01"
    pedicel_link = f"/World/Stem/{pedicel['id']}_Link_01"
    assert stage.GetPrimAtPath(f"{truss_link}/Cylinder").IsValid()
    assert stage.GetPrimAtPath(f"{pedicel_link}/Cylinder").IsValid()
    assert not stage.GetPrimAtPath("/World/PlantVisual/Truss_01_rachis/SkelRoot").IsValid()
    tomato_prim = stage.GetPrimAtPath("/World/TerminalBodies/Tomato_01")
    assert tomato_prim.IsValid()
    assert UsdPhysics.MassAPI(tomato_prim).GetMassAttr().Get() == pytest.approx(0.02)
    joint = stage.GetPrimAtPath("/World/TerminalBodies/Tomato_01/TerminalBodyFixedJoint")
    assert joint.IsValid()
    assert UsdPhysics.Joint(joint).GetBody0Rel().GetTargets()[0].pathString == pedicel_link
    assert joint.GetAttribute("physics:breakForce").Get() == pytest.approx(
        TrussPhysicsConfig.TOMATO_DETACHMENT_BREAK_FORCE_N
    )
    assert joint.GetAttribute("physics:excludeFromArticulation").Get() is True
