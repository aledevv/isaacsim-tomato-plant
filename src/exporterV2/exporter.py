"""Skeleton + config -> articulated USDA for Isaac Sim, plus a JSON manifest.

    skeleton (input/skeleton.json)
      -> skeleton_branches.build_branch_plan   group nodes into branches
      -> joint_budget (optional)               lock/merge joints to fit PhysX
      -> core.usd.build_stage                  rigid links, joints, branch meshes
      -> visuals.leaves / visuals.trusses      leaflets, truss dressing, stem colour
      -> collisions                            filter overlaps at rest, branch contacts
      -> audit                                 counts and poses -> manifest
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from plant_skeleton import SKELETON_SCHEMA_VERSION, PlantSkeleton, skeleton_warnings

from .collisions import auto_filter_initial_overlaps, collider_overlaps, enable_branch_collisions
from .config import PhysicsConfig, VisualsConfig
from .core import tree_config
from .core.physics import apply_physx_articulation_settings, apply_physx_scene_settings
from .core.usd import build_stage
from .joint_budget import optimize_branches
from .joint_budget_merges import MAX_ARTICULATION_LINKS, count_bodies, count_d6, merged_links
from .skeleton_branches import BranchPlan, build_branch_plan
from .visuals.leaves import author_leaf_visuals
from .visuals.fruits import author_calyces
from .visuals.stems import apply_stem_color
from .visuals.trusses import author_truss_visuals


MANIFEST_SCHEMA = "autotom_export_manifest/1.0"
# global_scale: auto keeps the thinnest segment at least this thick (PhysX).
AUTO_SCALE_MIN_RADIUS_M = tree_config.MIN_LINK_RADIUS_WORLD
AUTO_SCALE_RANGE = (1.0, 10.0)


class ExportError(ValueError):
    pass


@dataclass(frozen=True)
class ExportResult:
    plan: BranchPlan
    usd_path: Path
    manifest_path: Path
    manifest: dict[str, Any]

    @property
    def d6_joints(self) -> int:
        authored = self.manifest["authored"]
        return int(authored.get("post_merge", authored)["d6_joints"])


def manifest_path_for(path: Path) -> Path:
    return Path(path).with_suffix(".manifest.json")


def resolve_global_scale(physics: PhysicsConfig, skeleton: PlantSkeleton) -> float:
    if physics.global_scale != "auto":
        return float(physics.global_scale)
    thinnest = min(node.radius for node in skeleton.nodes)
    low, high = AUTO_SCALE_RANGE
    return max(low, min(high, AUTO_SCALE_MIN_RADIUS_M / thinnest))


def apply_physics_config(physics: PhysicsConfig, scale: float) -> None:
    """Copy the physics settings into the core's global configuration."""

    simulation = physics.simulation
    tree_config.GLOBAL_SCALE = float(scale)
    runtime = tree_config.PhysicsRuntimeConfig
    runtime.PHYSICS_HZ = int(simulation["physics_hz"])
    runtime.SOLVER_POSITION_ITERATIONS = int(simulation["solver_position_iterations"])
    runtime.SOLVER_VELOCITY_ITERATIONS = int(simulation["solver_velocity_iterations"])
    runtime.TERMINAL_BODY_SOLVER_POSITION_ITERATIONS = int(simulation["fruit_solver_position_iterations"])
    runtime.TERMINAL_BODY_SOLVER_VELOCITY_ITERATIONS = int(simulation["fruit_solver_velocity_iterations"])
    runtime.ENABLE_GPU_DYNAMICS = bool(simulation["gpu_dynamics"])
    # The core welds the root branch when RIGID_TRUNK is set; follow the config.
    runtime.RIGID_TRUNK = physics.segment("stem")["joint"] == "fixed"


def _custom(prim, name: str, value_type, value) -> None:
    prim.CreateAttribute(name, value_type, custom=True).Set(value)


def _author_metadata(stage, skeleton: PlantSkeleton, plan: BranchPlan, scale: float) -> None:
    stem = stage.GetPrimAtPath("/World/Stem")
    _custom(stem, "autotom:skeletonSchema", Sdf.ValueTypeNames.String, skeleton.schema_version)
    _custom(stem, "autotom:skeletonSource", Sdf.ValueTypeNames.String, skeleton.metadata.source)
    _custom(stem, "autotom:pose", Sdf.ValueTypeNames.String, plan.pose)
    _custom(stem, "autotom:visualQuality", Sdf.ValueTypeNames.String, plan.visual_quality)
    _custom(stem, "autotom:fruitMode", Sdf.ValueTypeNames.String, plan.fruit_mode)
    _custom(stem, "autotom:globalScale", Sdf.ValueTypeNames.Double, float(scale))
    _custom(stem, "autotom:collidersEnabled", Sdf.ValueTypeNames.Bool, True)
    _custom(stem, "autotom:drivesEnabled", Sdf.ValueTypeNames.Bool, True)
    _custom(stem, "autotom:articulationEnabled", Sdf.ValueTypeNames.Bool, True)
    _custom(stem, "autotom:terminalBodiesPhysical", Sdf.ValueTypeNames.Bool, plan.fruit_mode == "physical")
    for branch in plan.branches:
        prim = stage.GetPrimAtPath(f"/World/Stem/Vegetative/{branch['id']}")
        if not prim or not prim.IsValid():
            continue
        _custom(prim, "autotom:branchId", Sdf.ValueTypeNames.String, branch["id"])
        _custom(prim, "autotom:branchKind", Sdf.ValueTypeNames.String, branch["kind"])
        _custom(prim, "autotom:jointType", Sdf.ValueTypeNames.String, branch["joint_type"])
        if branch.get("source_parent_node_id") is not None:
            _custom(prim, "autotom:sourceParentNodeId", Sdf.ValueTypeNames.String, branch["source_parent_node_id"])


def _check_budget(plan: BranchPlan, physics: PhysicsConfig) -> tuple[BranchPlan, dict | None]:
    joints = physics.joints
    optimization = None
    if joints["optimize"] != "none":
        plan_branches, optimization = optimize_branches(plan, int(joints["joint_target"]), techniques=joints["optimize"])
        plan = plan_branches
    d6 = count_d6(plan.branches)
    if d6 > joints["joint_max"]:
        raise ExportError(
            f"{d6} D6 joints exceed joints.joint_max={joints['joint_max']}; "
            "set joints.optimize: lock (or full) in config/physics.yaml"
        )
    if d6 > joints["joint_target"]:
        print(f"[WARNING] {d6} D6 joints are above joints.joint_target={joints['joint_target']}")
    links = count_bodies(plan.branches)
    if links > MAX_ARTICULATION_LINKS:
        raise ExportError(
            f"{links} articulation links exceed the PhysX limit of {MAX_ARTICULATION_LINKS}; "
            "set joints.optimize: full in config/physics.yaml to merge links"
        )
    return plan, optimization


def export_plant(
    skeleton: PlantSkeleton,
    output_path: str | Path,
    physics: PhysicsConfig,
    visuals: VisualsConfig,
) -> ExportResult:
    """Build, check and save one plant. Raises ExportError when the audit fails."""

    from .leaf_shapes.baked import prepare_baked_leaves
    from .leaf_shapes.seeds import author_shape_manifest, prepare_leaf_shapes

    destination = Path(output_path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    for warning in skeleton_warnings(skeleton):
        print(f"[WARNING] {warning}")

    scale = resolve_global_scale(physics, skeleton)
    apply_physics_config(physics, scale)
    plan = build_branch_plan(skeleton, physics, visuals)
    plan, optimization = _check_budget(plan, physics)

    leaflets = [(record["leaflet_id"], record["leaflet_role"]) for record in plan.rigid_leaf_visuals]
    baked = None
    if visuals.leaves["generator"] == "baked":
        baked = prepare_baked_leaves(
            leaflets,
            plant_id=skeleton.metadata.plant_id,
            seed=int(visuals.leaves["seed"]),
            one_per_leaflet=bool(visuals.leaves["one_per_leaflet"]),
            folder=visuals.leaves["baked_set"],
        )
    leaf_shapes = prepare_leaf_shapes(
        leaflets,
        plant_id=skeleton.metadata.plant_id,
        seed=int(visuals.leaves["seed"]),
    )
    stage, stem_path = build_stage(
        str(destination),
        branches=list(plan.branches),
        terminal_bodies=list(plan.terminal_bodies),
        locked_joints=False,
        branch_backend="skinned",
        skinning_visual_mode="segmented",
        leaf_shapes=leaf_shapes,
    )
    _author_metadata(stage, skeleton, plan, scale)
    leaf_visuals = author_leaf_visuals(
        stage,
        plan.rigid_leaf_visuals,
        plan.branches[0]["visual_profile"],
        leaf_shapes,
        baked=baked,
        leaves=visuals.leaves,
        plant_id=skeleton.metadata.plant_id,
    )
    truss_visuals = author_truss_visuals(stage, plan.branches)
    calyces = author_calyces(stage) if visuals.fruits["calyx"] else []
    if visuals.stems["color"] is not None:
        apply_stem_color(stage, visuals.stems["color"])
    filters = []
    if physics.collisions["initial_overlaps"] == "filter":
        filters = auto_filter_initial_overlaps(stage)
    apply_physx_articulation_settings(stage, stem_path)
    branch_contacts = enable_branch_collisions(stage) if physics.collisions["branches"] else None
    apply_physx_scene_settings(stage, physics_hz=int(physics.simulation["physics_hz"]))
    author_shape_manifest(stage, leaf_shapes)
    stage.GetRootLayer().Save()

    manifest = audit_stage(stage, skeleton, plan, scale, leaf_visuals, truss_visuals, filters)
    manifest["authored"]["calyces"] = len(calyces)
    manifest["collisions"]["branch_contacts"] = branch_contacts
    manifest["metadata"].update(
        usd=str(destination),
        physics_config=physics.source,
        visuals_config=visuals.source,
        physics=physics.values,
        visuals=visuals.values,
        leaf_shapes=leaf_shapes.manifest(),
        baked_leaves=baked.manifest() if baked is not None else None,
        joint_budget_optimization=optimization,
    )
    _merge_links(stage, plan, manifest, physics, optimization)
    manifest_path = manifest_path_for(destination)
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    if manifest["errors"]:
        raise ExportError("; ".join(manifest["errors"]))
    return ExportResult(plan, destination, manifest_path, manifest)


def _merge_links(stage, plan: BranchPlan, manifest, physics: PhysicsConfig, optimization) -> None:
    """Absorb links that joints.optimize: full marked as merged."""

    targets = {branch["id"]: merged_links(branch) for branch in plan.branches if merged_links(branch)}
    if not targets or manifest["errors"]:
        return
    from .core.usd.body_merge import merge_rigid_links

    absorbed = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.GetAttribute("autotom:entityKind").Get() == "physical_link"
        and prim.GetAttribute("autotom:branchLinkIndex").Get()
        in targets.get(prim.GetAttribute("autotom:branchId").Get(), ())
    ]
    expected = sum(len(links) for links in targets.values())
    if len(absorbed) != expected:
        raise ExportError(f"found {len(absorbed)} links to merge, expected {expected}")
    report = merge_rigid_links(stage, absorbed, stiffness_policy=physics.joints["merge_stiffness"])
    stage.GetRootLayer().Save()
    optimization["body_merge"] = report
    manifest["authored"]["post_merge"] = report["after"]
    if report["after"]["d6_joints"] != count_d6(plan.branches):
        manifest["errors"].append(
            f"merged D6 joints {report['after']['d6_joints']} != predicted {count_d6(plan.branches)}"
        )


def _expected_organic_meshes(plan: BranchPlan) -> int:
    """One mesh per link, except a visual axis welded by fixed joints gets one
    continuous mesh (see core/skinning/visual_segmented.py)."""

    axes: dict[str, list[dict]] = {}
    for branch in plan.branches:
        axes.setdefault(branch.get("visual_axis_id") or branch["id"], []).append(branch)
    total = 0
    for members in axes.values():
        links = sum(branch["n_links"] for branch in members)
        rigid = all(branch["joint_type"] == "fixed" for branch in members)
        total += 1 if rigid else links
    return total


def audit_stage(stage, skeleton, plan: BranchPlan, scale: float, leaf_visuals, truss_visuals, filters) -> dict[str, Any]:
    """Count what was authored and check every rigid link sits where the skeleton says."""

    specs = [spec for branch in plan.branches for spec in branch["link_specs"]]
    spec_by_node = {spec["canonical_axis_id"]: spec for spec in specs}
    traversal = list(stage.Traverse())
    links = [prim for prim in traversal if prim.GetAttribute("autotom:entityKind").Get() == "physical_link"]
    bodies = [prim for prim in traversal if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
    fixed_joints = [prim for prim in traversal if prim.GetTypeName() == "PhysicsFixedJoint"]
    d6_joints = [prim for prim in traversal if prim.GetTypeName() == "PhysicsJoint"]
    meshes = [prim for prim in traversal if prim.IsA(UsdGeom.Mesh)]
    organic = [prim for prim in meshes if prim.GetName().startswith("OrganicVisual_")]
    branch_meshes = [prim for prim in organic if prim.GetParent().GetAttribute("autotom:entityKind").Get() == "physical_link"]
    petiolule_meshes = [prim for prim in organic if prim.GetParent().GetAttribute("autotom:entityKind").Get() == "rigid_leaf_visual"]
    # A baked blade is an Xform referencing its asset, a generated one a Mesh.
    blades = [prim for prim in traversal if prim.GetName() == "LeafBlade"]
    fork_shoots = [prim for prim in meshes if prim.GetName() == "TerminalForkYoungShoot"]
    capsules = [prim for prim in traversal if prim.IsA(UsdGeom.Capsule)]
    spheres = [prim for prim in traversal if prim.IsA(UsdGeom.Sphere)]
    errors: list[str] = []

    physical_fruits = sum(bool(body["physical"]) for body in plan.terminal_bodies)
    expected = {
        "rigid_bodies": len(specs) + physical_fruits,
        # Before link merging (joints.optimize: full merges after this audit).
        # The root link is anchored to the world, so it never gets a D6.
        "d6_joints": sum(
            branch["n_links"] - (branch.get("parent") is None)
            for branch in plan.branches
            if branch["joint_type"] != "fixed"
        ),
        "organic_meshes": _expected_organic_meshes(plan),
        "capsule_colliders": 2 * len(specs),
        "leaflets": len(plan.rigid_leaf_visuals),
        "fruit_spheres": len(plan.terminal_bodies),
    }
    authored = {
        "rigid_bodies": len(bodies),
        "physical_links": len(links),
        "fixed_joints": len(fixed_joints),
        "d6_joints": len(d6_joints),
        "organic_meshes": len(branch_meshes),
        "capsule_colliders": len(capsules),
        "leaflets": len(leaf_visuals),
        "petiolule_meshes": len(petiolule_meshes),
        "leaf_blades": len(blades),
        "terminal_forks": len(fork_shoots),
        "truss_visuals": len(truss_visuals),
        "fruit_spheres": len(spheres),
        "total_meshes": len(meshes),
    }
    for key, value in expected.items():
        if authored[key] != value:
            errors.append(f"{key}: authored {authored[key]}, expected {value}")
    if len(blades) != len(plan.rigid_leaf_visuals):
        errors.append(f"leaf_blades: authored {len(blades)}, expected {len(plan.rigid_leaf_visuals)}")

    poses = []
    for prim in links:
        node_id = prim.GetAttribute("autotom:canonicalPrimitiveId").Get()
        spec = spec_by_node.get(node_id)
        if spec is None:
            errors.append(f"body {prim.GetPath()} has unknown node {node_id!r}")
            continue
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        position = matrix.ExtractTranslation()
        direction = Gf.Vec3d(matrix.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))).GetNormalized()
        frame = spec["rest_frame"]
        expected_position = Gf.Vec3d(*(float(frame[row][3]) * scale for row in range(3)))
        expected_direction = Gf.Vec3d(*(float(frame[row][2]) for row in range(3))).GetNormalized()
        if (position - expected_position).GetLength() > 1e-6:
            errors.append(f"rest-position mismatch for {prim.GetPath()}")
        if (direction - expected_direction).GetLength() > 1e-6:
            errors.append(f"rest-direction mismatch for {prim.GetPath()}")
        if not math.isclose(float(prim.GetAttribute("autotom:visualRadius").Get()), spec["radius"] * scale, abs_tol=1e-6):
            errors.append(f"visual-radius mismatch for {prim.GetPath()}")
        poses.append(
            {
                "node_id": node_id,
                "body_path": str(prim.GetPath()),
                "start": [float(value) for value in position],
                "direction": [float(value) for value in direction],
                "length": float(spec["length"]) * scale,
            }
        )

    _filtered, active = collider_overlaps(stage)
    if active:
        errors.append(f"{len(active)} collider overlaps at rest are not filtered")

    masses = [
        {
            "body_path": str(prim.GetPath()),
            "mass_kg": float(UsdPhysics.MassAPI(prim).GetMassAttr().Get()),
        }
        for prim in bodies
    ]
    return {
        "schema_version": MANIFEST_SCHEMA,
        "metadata": {
            "skeleton_schema": SKELETON_SCHEMA_VERSION,
            "skeleton_source": skeleton.metadata.source,
            "skeleton_description": skeleton.metadata.description,
            "plant_id": skeleton.metadata.plant_id,
            "time": skeleton.metadata.time,
            "global_scale": scale,
            "pose": plan.pose,
            "visual_quality": plan.visual_quality,
            "fruit_mode": plan.fruit_mode,
        },
        "skeleton": {
            "nodes": len(skeleton.nodes),
            "leaflets": len(skeleton.leaflets),
            "fruits": len(skeleton.fruits),
        },
        "expected": expected,
        "authored": authored,
        "topology": {
            "branches": [
                {
                    "id": branch["id"],
                    "kind": branch["kind"],
                    "parent": branch["parent"],
                    "attach_link": branch["attach_link"],
                    "joint_type": branch["joint_type"],
                    "node_ids": [spec["canonical_node_id"] for spec in branch["link_specs"]],
                }
                for branch in plan.branches
            ],
            "attachment_map": list(plan.attachment_map),
            "leaflets": [
                {
                    key: record[key]
                    for key in ("id", "leaflet_id", "leaflet_role", "host_body_path", "root_path", "rest_frame", "length")
                }
                for record in leaf_visuals
            ],
        },
        "poses": poses,
        "physics": {"body_masses": masses},
        "collisions": {"filtered_at_rest": filters, "active_at_rest": active},
        "errors": errors,
    }
