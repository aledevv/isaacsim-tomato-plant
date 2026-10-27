from __future__ import annotations

import pytest
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from exporterV2.core.usd.body_merge import BodyMergeError, merge_rigid_links


def _link(stage, path, z, mass, tilt_deg=0.0):
    xform = UsdGeom.Xform.Define(stage, path)
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z))
    xform.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), tilt_deg).GetQuat()))
    prim = xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr().Set(mass)
    mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.05))
    capsule = UsdGeom.Capsule.Define(stage, f"{path}/Collider_01")
    capsule.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.05))
    UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
    UsdGeom.Mesh.Define(stage, f"{path}/OrganicVisual_01")
    return prim


def _d6(stage, path, body0, body1, pos0, stiffness):
    joint = UsdPhysics.Joint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*pos0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    for axis in ("rotX", "rotY"):
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateStiffnessAttr().Set(stiffness)
        drive.CreateDampingAttr().Set(1.0)
    return joint.GetPrim()


@pytest.fixture
def stage():
    stage = Usd.Stage.CreateInMemory()
    root = "/World/Stem"
    UsdPhysics.ArticulationRootAPI.Apply(UsdGeom.Xform.Define(stage, root).GetPrim())
    _link(stage, f"{root}/A", 0.0, 1.0)
    _link(stage, f"{root}/B", 0.1, 1.0, tilt_deg=30.0)
    _link(stage, f"{root}/C", 0.2, 1.0, tilt_deg=60.0)
    anchor = UsdPhysics.FixedJoint.Define(stage, f"{root}/A/Root")
    anchor.CreateBody1Rel().SetTargets([f"{root}/A"])
    _d6(stage, f"{root}/B/Joint", f"{root}/A", f"{root}/B", (0, 0, 0.1), 4.0)
    _d6(stage, f"{root}/C/Joint", f"{root}/B", f"{root}/C", (0, 0, 0.1), 4.0)
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"{root}/C")).CreateFilteredPairsRel().SetTargets([f"{root}/B"])
    return stage


def _joint_mismatch(stage, joint):
    def world(side):
        body = joint.GetRelationship(f"physics:body{side}").GetTargets()[0]
        matrix = UsdGeom.Xformable(stage.GetPrimAtPath(body)).ComputeLocalToWorldTransform(0)
        return matrix.Transform(Gf.Vec3d(joint.GetAttribute(f"physics:localPos{side}").Get()))

    return (world(0) - world(1)).GetLength()


def test_merge_keeps_geometry_mass_and_retargets_downstream_joint(stage):
    joint_c = stage.GetPrimAtPath("/World/Stem/C/Joint")
    before = _joint_mismatch(stage, joint_c)
    report = merge_rigid_links(stage, ["/World/Stem/B"], stiffness_policy="series")
    assert not stage.GetPrimAtPath("/World/Stem/B")
    assert report["after"]["rigid_bodies"] == 2
    assert report["after"]["d6_joints"] == 1
    assert report["after"]["meshes"] == 3 and report["after"]["colliders"] == 3
    assert stage.GetPrimAtPath("/World/Stem/A/Merged_B/OrganicVisual_01")
    joint_c = stage.GetPrimAtPath("/World/Stem/C/Joint")
    assert joint_c.GetRelationship("physics:body0").GetTargets()[0] == "/World/Stem/A"
    # The re-expressed frame still meets body1 at the same world point.
    assert _joint_mismatch(stage, joint_c) == pytest.approx(before, abs=1e-6)
    mass = UsdPhysics.MassAPI(stage.GetPrimAtPath("/World/Stem/A"))
    assert mass.GetMassAttr().Get() == pytest.approx(2.0)
    filters = UsdPhysics.FilteredPairsAPI(stage.GetPrimAtPath("/World/Stem/C")).GetFilteredPairsRel().GetTargets()
    assert [str(t) for t in filters] == ["/World/Stem/A"]


def test_series_and_keep_policies(stage):
    # C into B: B's joint takes the removed compliance (1/K = 1/4 + 1/4).
    report = merge_rigid_links(stage, ["/World/Stem/C"], stiffness_policy="series")
    drive = UsdPhysics.DriveAPI(stage.GetPrimAtPath("/World/Stem/B/Joint"), "rotX")
    assert drive.GetStiffnessAttr().Get() == pytest.approx(2.0)
    assert drive.GetDampingAttr().Get() == pytest.approx(2.0 ** -0.5)
    assert report["merges"][0]["compliance"]["status"] == "series"


def test_keep_policy_leaves_upstream_drive(stage):
    merge_rigid_links(stage, ["/World/Stem/C"], stiffness_policy="keep")
    drive = UsdPhysics.DriveAPI(stage.GetPrimAtPath("/World/Stem/B/Joint"), "rotX")
    assert drive.GetStiffnessAttr().Get() == pytest.approx(4.0)


def test_load_policy_weights_compliance_between_keep_and_series(stage):
    report = merge_rigid_links(stage, ["/World/Stem/C"], stiffness_policy="load")
    weight = report["merges"][0]["compliance"]["weight"]
    assert 0.0 < weight < 1.0
    drive = UsdPhysics.DriveAPI(stage.GetPrimAtPath("/World/Stem/B/Joint"), "rotX")
    assert drive.GetStiffnessAttr().Get() == pytest.approx(1.0 / (0.25 + weight / 4.0))


def test_world_anchored_root_cannot_be_absorbed(stage):
    with pytest.raises(BodyMergeError, match="anchored"):
        merge_rigid_links(stage, ["/World/Stem/A"])


def test_chain_merge_in_any_order(stage):
    report = merge_rigid_links(stage, ["/World/Stem/C", "/World/Stem/B"], stiffness_policy="keep")
    assert report["after"]["rigid_bodies"] == 1
    assert report["after"]["d6_joints"] == 0
    assert stage.GetPrimAtPath("/World/Stem/A/Merged_B/Merged_C/Collider_01")


def test_tip_policy_weights_by_squared_distance_to_the_tip(stage):
    for path, length in (("/World/Stem/A", 0.1), ("/World/Stem/B", 0.1), ("/World/Stem/C", 0.1)):
        stage.GetPrimAtPath(path).CreateAttribute("autotom:sourceLength", Sdf.ValueTypeNames.Double).Set(length)
    report = merge_rigid_links(stage, ["/World/Stem/C"], stiffness_policy="tip")
    weight = report["merges"][0]["compliance"]["weight"]
    # C's joint is closer to the tip than B's joint, so 0 < w < 1.
    assert 0.0 < weight < 1.0
    drive = UsdPhysics.DriveAPI(stage.GetPrimAtPath("/World/Stem/B/Joint"), "rotX")
    assert drive.GetStiffnessAttr().Get() == pytest.approx(1.0 / (0.25 + weight / 4.0))
