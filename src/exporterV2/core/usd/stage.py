"""
stage.py - USD Stage Setup and Orchestration

Top-level functions for building tree USD stages with articulated physics.
"""

import os
import sys
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf

# Support both direct execution and module import
if __name__ == "__main__" or "exporterV2" not in sys.modules:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    SRC_DIR = os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))
    if SRC_DIR not in sys.path:
        sys.path.insert(0, SRC_DIR)
    from exporterV2.core.tree_config import (
        BRANCHES, GAP, OutputConfig, PhysicsRuntimeConfig, TrussPhysicsConfig,
        scaled, validate_branches
    )
else:
    from ..tree_config import (
        BRANCHES, GAP, OutputConfig, PhysicsRuntimeConfig, TrussPhysicsConfig,
        scaled, validate_branches
    )

from .branch_chains import build_chain, is_truss_branch as _is_truss_branch
from .collision import (
    add_sibling_collision_filtering,
)
from .terminal_bodies import (
    build_terminal_bodies,
    filter_external_terminal_body_collisions,
    validate_terminal_body_clearance,
)


def get_output_usd_path() -> str:
    """Get the default output path for generated USD file."""
    # Navigate from usd → exporterV2 → src → project_root
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    output_dir = os.path.join(project_root, "data", "usd_models")
    os.makedirs(output_dir, exist_ok=True)
    return os.path.join(output_dir, "tree_v2.usda")


def setup_base_stage(path: str, legacy_physics: bool = False):
    """Create or clear USD stage with World and Stem prims."""
    existing_layer = Sdf.Layer.Find(path)
    if existing_layer:
        existing_layer.Clear()
        stage = Usd.Stage.Open(existing_layer)
    else:
        if os.path.exists(path):
            try:
                os.remove(path)
            except Exception:
                pass
        stage = Usd.Stage.CreateNew(path)

    world_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_prim.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    if not legacy_physics:
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

    stem_path = "/World/Stem"
    stem_prim = UsdGeom.Xform.Define(stage, stem_path)
    UsdPhysics.ArticulationRootAPI.Apply(stem_prim.GetPrim())

    return stage, stem_path


def build_stage(
    output_path: str,
    branches=None,
    locked_joints: bool = False,
    skip_limit_check: bool = False,
    terminal_bodies=None,
    legacy_physics: bool = False,
    branch_backend: str = "legacy",
    skinning_visual_mode: str = None,
    leaf_shapes=None,
):
    """
    Build the full tree USD stage from BRANCHES configuration.

    Args:
        output_path: Path where to save the USD file
        branches: List of branch definitions (uses BRANCHES from tree_config if None)
        locked_joints: If True, use FixedJoint instead of flexible D6 joints
                      (for integration tests to verify rigid geometry)
        skip_limit_check: If True, skip the link count limit check
        terminal_bodies: Optional rigid bodies attached to branch tips. This is a
                         generic hook used by adapter-generated tomatoes.
        branch_backend: ``legacy`` keeps cylinder branches; ``skinned`` uses
                        smooth UsdSkel visuals and capsule proxies for vegetation.
        skinning_visual_mode: Optional visual implementation for the skinned
                              backend. The environment variable remains a fallback.

    Returns:
        Tuple (stage, stem_path)
    """
    if branches is None:
        branches = BRANCHES
    if terminal_bodies is None:
        terminal_bodies = []
    if branch_backend not in ("legacy", "skinned"):
        raise ValueError(
            f"Unsupported branch_backend={branch_backend!r}; expected 'legacy' or 'skinned'"
        )

    validate_branches(branches, skip_limit_check=skip_limit_check)

    stage, stem_path = setup_base_stage(output_path, legacy_physics=legacy_physics)

    if legacy_physics:
        # Revert to legacy non-physics units to simulate original behavior
        UsdGeom.SetStageMetersPerUnit(stage, 0.01) # fallback to default cm

    # Registry: branch_id → (link_paths, base_positions, axis_vector, orientation_quat)
    branch_registry = {}
    branch_defs = {branch["id"]: branch for branch in branches}

    branches_to_build = branches
    if branch_backend == "skinned":
        try:
            from ..skinning import (
                build_skinned_vegetative_structure,
                partition_branches,
            )
        except ImportError:
            from exporterV2.core.skinning import (
                build_skinned_vegetative_structure,
                partition_branches,
            )

        vegetative_branches, branches_to_build = partition_branches(branches)
        if not vegetative_branches:
            raise ValueError("The skinned backend requires at least one vegetative branch")
        branch_registry.update(build_skinned_vegetative_structure(
            stage,
            stem_path,
            vegetative_branches,
            all_branch_defs=branch_defs,
            locked_joints=locked_joints,
            legacy_physics=legacy_physics,
            visual_mode=skinning_visual_mode,
            leaf_shapes=leaf_shapes,
        ))

    for b in branches_to_build:
        bid     = b["id"]
        is_root = b.get("parent") is None
        h_world = scaled(b["height"])
        r_world = scaled(b["radius"])
        gap     = scaled(GAP)

        if is_root:
            # Root trunk (vertical)
            chain_axis = Gf.Vec3d(0.0, 0.0, 1.0)
            start_pos  = Gf.Vec3d(0.0, 0.0, 0.0)
            if OutputConfig.STEP_1_VERBOSE:
                print(f"[INFO] '{bid}' (root): {b['n_links']} links, "
                    f"r={r_world:.3f}m, h={h_world:.3f}m")

            link_paths, link_bases = build_chain(
                stage, stem_path, b,
                start_pos, chain_axis,
                is_root=True,
                chain_orientation=None,
                locked_joints=locked_joints,
            )

            branch_registry[bid] = (link_paths, link_bases, chain_axis, Gf.Quatf(1, 0, 0, 0))

        else:
            # Branch attached to parent
            parent_id  = b["parent"]
            attach_idx = b["attach_link"] - 1
            tilt_deg   = b["tilt"]
            rot_deg    = b["rot"]
            roll_deg   = b.get("roll", 0.0)

            parent_paths, parent_bases, parent_axis, parent_orientation = branch_registry[parent_id]
            parent_def = branch_defs[parent_id]
            p_h_world  = scaled(parent_def["height"])
            p_r_world  = scaled(parent_def["radius"])

            # Compute branch orientation: rot_z → tilt → roll
            rot_z    = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_deg)
            rot_tilt = Gf.Rotation(Gf.Vec3d(1, 0, 0), -tilt_deg)
            rot_roll = Gf.Rotation(Gf.Vec3d(0, 0, 1), roll_deg)
            branch_rot_in_parent_frame = rot_roll * rot_tilt * rot_z

            parent_rot = Gf.Rotation(Gf.Quatd(parent_orientation))
            combined = branch_rot_in_parent_frame * parent_rot
            chain_axis_raw = combined.TransformDir(Gf.Vec3d(0, 0, 1))
            chain_axis     = Gf.Vec3d(*chain_axis_raw).GetNormalized()
            chain_orientation = Gf.Quatf(combined.GetQuat())

            if tilt_deg == 0.0 and rot_deg == 0.0:
                radial_distance = 0.0
            else:
                radial_distance = p_r_world / 2.0

            # attach_frac: fractional position within the parent link [0.0, 1.0].
            #   1.0 (default) = top of link + small gap (original behaviour, coaxial seams)
            #   <1.0          = mid-link attachment (used by remapping; no gap needed,
            #                   branch emerges from the side of the cylinder)
            # This field is set by the remapping code after stem reduction.
            attach_frac = b.get("attach_frac", 1.0)
            if attach_frac >= 1.0:
                z_local = p_h_world + gap        # top of link + gap (default)
            else:
                z_local = attach_frac * p_h_world  # sub-link: exact fraction, no gap
            base_offset_local = Gf.Vec3d(0.0, radial_distance, z_local)

            rot_z_local = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_deg)
            offset_in_parent_frame = rot_z_local.TransformDir(base_offset_local)
            offset_in_world = parent_rot.TransformDir(offset_in_parent_frame)

            attach_base  = parent_bases[attach_idx]
            start_pos    = attach_base + offset_in_world

            # Joint frame in parent-link local frame
            local_pos0 = Gf.Vec3f(
                offset_in_parent_frame[0],
                offset_in_parent_frame[1],
                offset_in_parent_frame[2]
            )
            local_rot0 = Gf.Quatf(branch_rot_in_parent_frame.GetQuat())

            if OutputConfig.STEP_1_VERBOSE:
                print(f"[INFO] '{bid}': {b['n_links']} links, "
                      f"r={r_world:.3f}m, h={h_world:.3f}m, "
                      f"parent='{parent_id}' link {b['attach_link']}, "
                      f"tilt={tilt_deg}deg, rot={rot_deg}deg, roll={roll_deg}deg")

            link_paths, link_bases = build_chain(
                stage, stem_path, b,
                start_pos, chain_axis,
                is_root=False,
                parent_link_path=parent_paths[attach_idx],
                attachment_local_pos0=local_pos0,
                attachment_local_rot0=local_rot0,
                chain_orientation=chain_orientation,
                locked_joints=locked_joints,
                use_truss_physics=_is_truss_branch(b),
                parent_def=parent_def,
                legacy_physics=legacy_physics,
            )

            branch_registry[bid] = (link_paths, link_bases, chain_axis, chain_orientation)

    terminal_body_records = build_terminal_bodies(
        stage,
        stem_path,
        terminal_bodies,
        branch_registry,
        branch_defs,
    )


    validate_terminal_body_clearance(
        terminal_body_records,
        branch_registry,
        branches,
        stage=stage,
        apply_filters=True,
        filter_terminal_body_pairs=TrussPhysicsConfig.FILTER_TERMINAL_BODY_PAIR_OVERLAPS,
        branch_defs=branch_defs,
    )

    filter_external_terminal_body_collisions(
        stage, terminal_body_records, branch_registry, branch_defs
    )

    # Add sibling collision filtering
    add_sibling_collision_filtering(stage, branches, branch_registry)

    return stage, stem_path


def build_stage_locked(
    output_path: str,
    branches=None,
    branch_backend: str = "legacy",
    skinning_visual_mode: str = None,
):
    """
    Convenience wrapper for build_stage() with locked_joints=True.

    Creates a USD stage where all joints are FixedJoint (completely rigid).
    Used for Isaac Sim integration tests to verify geometry doesn't change
    during simulation when joints have no flexibility.

    Args:
        output_path: Path where to save the USD file
        branches: List of branch definitions (uses BRANCHES if None)

    Returns:
        Tuple (stage, stem_path)

    Example:
        stage, stem_path = build_stage_locked("test_locked.usda")
    """
    return build_stage(
        output_path,
        branches,
        locked_joints=True,
        branch_backend=branch_backend,
        skinning_visual_mode=skinning_visual_mode,
    )
