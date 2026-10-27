from __future__ import annotations

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from complexity_profile import (
    complexity_profile,
    plant_complexity,
    stage_complexity,
)


def _body(stage, path, collider=True):
    prim = UsdGeom.Xform.Define(stage, path).GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    if collider:
        capsule = UsdGeom.Capsule.Define(stage, f"{path}/Collider")
        UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
    return prim


def _d6(stage, path, body0, body1, free_axes):
    joint = UsdPhysics.Joint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    for axis in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        if axis in free_axes:
            limit.CreateLowAttr().Set(-30.0)
            limit.CreateHighAttr().Set(30.0)
            UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis).CreateStiffnessAttr().Set(1.0)
        else:
            limit.CreateLowAttr().Set(1.0)
            limit.CreateHighAttr().Set(-1.0)
    return joint


def _stage(tmp_path, self_collisions=False):
    path = tmp_path / "plant.usda"
    stage = Usd.Stage.CreateNew(str(path))
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene").GetPrim()
    scene.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Int).Set(60)
    scene.CreateAttribute("physxScene:solverType", Sdf.ValueTypeNames.Token).Set("PGS")
    root = UsdGeom.Xform.Define(stage, "/World/Stem").GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(root)
    root.CreateAttribute(
        "physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int
    ).Set(32)
    root.CreateAttribute(
        "physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool
    ).Set(self_collisions)
    _body(stage, "/World/Stem/A")
    _body(stage, "/World/Stem/B")
    _body(stage, "/World/Stem/C")
    _body(stage, "/World/Stem/D")
    fixed = UsdPhysics.FixedJoint.Define(stage, "/World/Stem/A/Root")
    fixed.CreateBody1Rel().SetTargets(["/World/Stem/A"])
    _d6(stage, "/World/Stem/B/J", "/World/Stem/A", "/World/Stem/B", ("rotX", "rotY"))
    _d6(stage, "/World/Stem/C/J", "/World/Stem/B", "/World/Stem/C", ("rotX",))
    revolute = UsdPhysics.RevoluteJoint.Define(stage, "/World/Stem/D/J")
    revolute.CreateBody0Rel().SetTargets(["/World/Stem/C"])
    revolute.CreateBody1Rel().SetTargets(["/World/Stem/D"])
    # A detachable fruit outside the articulation.
    _body(stage, "/World/TerminalBodies/Fruit")
    attach = UsdPhysics.FixedJoint.Define(stage, "/World/TerminalBodies/Fruit/Attach")
    attach.CreateBody0Rel().SetTargets(["/World/Stem/D"])
    attach.CreateBody1Rel().SetTargets(["/World/TerminalBodies/Fruit"])
    attach.CreateExcludeFromArticulationAttr().Set(True)
    attach.CreateBreakForceAttr().Set(6.0)
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath("/World/Stem/A")).CreateFilteredPairsRel().AddTarget("/World/TerminalBodies/Fruit")
    mesh = UsdGeom.Mesh.Define(stage, "/World/Visual")
    mesh.CreateFaceVertexCountsAttr().Set([3, 4])
    hidden = UsdGeom.Mesh.Define(stage, "/World/Hidden")
    hidden.CreateFaceVertexCountsAttr().Set([3])
    hidden.MakeInvisible()
    stage.GetRootLayer().Save()
    return path


def test_dofs_constraint_rows_and_joint_roles(tmp_path):
    model = stage_complexity(_stage(tmp_path))["model"]
    assert model["joints"] == {"d6": 2, "fixed": 2, "revolute": 1}
    assert model["dofs"] == 2 + 1 + 1
    assert model["limited_dofs"] == 3
    assert model["drives"] == 3
    assert model["maximal_joints"] == 1
    assert model["breakable_joints"] == 1
    # 3 limits + 3 drives in reduced coordinates, 6 locked rows for the fruit.
    assert model["constraint_rows"] == 12
    assert model["articulations"] == 1
    assert model["max_links_per_articulation"] == 4


def test_collision_pairs_respect_self_collisions_filters_and_joints(tmp_path):
    pairs = stage_complexity(_stage(tmp_path))["model"]["collision_pairs"]
    assert pairs["raw"] == 10
    assert pairs["articulation_self_excluded"] == 6
    assert pairs["filtered_pairs_excluded"] == 1  # A-Fruit
    assert pairs["jointed_excluded"] == 1  # D-Fruit
    assert pairs["candidate"] == 2  # B-Fruit, C-Fruit

    pairs = stage_complexity(_stage(tmp_path, self_collisions=True))["model"]["collision_pairs"]
    assert pairs["articulation_self_excluded"] == 0
    assert pairs["jointed_excluded"] == 4
    assert pairs["candidate"] == 2 + 3  # A-C, A-D, B-D


def test_visible_geometry_and_solver_work_proxy(tmp_path):
    profile = stage_complexity(_stage(tmp_path))
    assert profile["model"]["visible_mesh_prims"] == 1
    assert profile["model"]["visible_triangles"] == 3
    assert profile["model"]["mesh_triangles"] == 4
    assert profile["solver"]["solverType"] == "PGS"
    assert profile["solver"]["work_proxy_dof_iters_per_s"] == 4 * 32 * 60


def test_plant_complexity_uses_mtg_branching_order():
    from plant_skeleton import PlantSkeleton, SkeletonFruit, SkeletonMetadata, SkeletonNode

    nodes = (
        SkeletonNode("a", None, "stem", (0, 0, 0), (0, 0, 0.1), 0.006),
        SkeletonNode("b", "a", "stem", (0, 0, 0.1), (0, 0, 0.2), 0.005),
        SkeletonNode("c", "a", "branch", (0, 0, 0.1), (0.1, 0, 0.15), 0.004),
        SkeletonNode("d", "c", "petiole", (0.1, 0, 0.15), (0.15, 0, 0.2), 0.002),
        SkeletonNode("t", "b", "truss_rachis", (0, 0, 0.2), (0.03, 0, 0.18), 0.0015),
        SkeletonNode("p", "t", "pedicel", (0.03, 0, 0.18), (0.03, 0, 0.165), 0.0008),
    )
    skeleton = PlantSkeleton(
        nodes,
        fruits=(SkeletonFruit("f", "p", 0.01),),
        metadata=SkeletonMetadata(time=10.0),
    )
    plant = plant_complexity(skeleton)
    assert plant["time"] == 10
    assert plant["metamers"] == 3
    assert plant["fruits"] == 1
    assert plant["max_branching_order"] == 2
    assert plant["nodes_by_branching_order"] == {"0": 2, "1": 2, "2": 2}
    assert plant["max_topological_depth"] == 4


def test_profile_reads_neighbouring_manifest(tmp_path):
    path = _stage(tmp_path)
    path.with_suffix(".manifest.json").write_text('{"metadata": {"day": 7}}')
    profile = complexity_profile(path)
    assert profile["variant"] == {"day": 7}
