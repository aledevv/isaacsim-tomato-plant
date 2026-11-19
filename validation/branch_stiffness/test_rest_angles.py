from __future__ import annotations

import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from rest_angles import organ_of, rest_angles


def _body(stage, path, mass, centre):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr().Set(mass)
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*centre))
    return prim


def test_horizontal_link_bends_by_moment_over_stiffness(tmp_path):
    usd = tmp_path / "beam.usda"
    stage = Usd.Stage.CreateNew(str(usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = "/World/Stem/Vegetative/Branch_g1"
    _body(stage, f"{root}/Base", 1.0, (0.0, 0.0, 0.0))
    _body(stage, f"{root}/Tip", 0.1, (0.2, 0.0, 0.0))  # 0.1 kg, 0.2 m from the joint
    joint = UsdPhysics.Joint.Define(stage, f"{root}/Tip/Joint")
    joint.CreateBody0Rel().SetTargets([f"{root}/Base"])
    joint.CreateBody1Rel().SetTargets([f"{root}/Tip"])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "rotX")
    drive.CreateStiffnessAttr().Set(0.01)  # N m per degree
    UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotX").CreateHighAttr().Set(30.0)
    stage.GetRootLayer().Save()

    [row] = rest_angles(usd)
    assert row["organ"] == "branch"
    assert row["angle_deg"] == pytest.approx(0.1 * 9.81 * 0.2 / 0.01)
    assert row["limit_deg"] == 30.0


def test_organ_of_pedicel_before_truss():
    assert organ_of("/World/Stem/Vegetative/Truss_g7_truss01_pedicel_01/x") == "pedicel"
    assert organ_of("/World/Stem/Vegetative/Truss_g7_truss01_rachis/x") == "truss rachis"
