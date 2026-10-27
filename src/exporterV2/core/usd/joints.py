"""
joints.py - USD Joint Creation

Creates flexible (D6) and locked (Fixed) joints for articulated structures.
"""

from pxr import Usd, UsdPhysics, Gf, Sdf
from .collision import add_collision_filter, add_attachment_collision_filters


def _set_optional_joint_attr(joint, create_method_name: str, attr_name: str, sdf_type, value) -> None:
    create_method = getattr(joint, create_method_name, None)
    if create_method is not None:
        create_method().Set(value)
    else:
        joint.GetPrim().CreateAttribute(attr_name, sdf_type).Set(value)


def configure_detachable_joint(
    joint,
    break_force: float = None,
    break_torque: float = None,
    exclude_from_articulation: bool = False,
) -> None:
    """Author USD physics attributes needed for a breakable terminal joint."""
    if break_force is not None:
        if break_force <= 0.0:
            raise ValueError(f"break_force must be positive, got {break_force}")
        _set_optional_joint_attr(
            joint,
            "CreateBreakForceAttr",
            "physics:breakForce",
            Sdf.ValueTypeNames.Float,
            float(break_force),
        )

    if break_torque is not None:
        if break_torque <= 0.0:
            raise ValueError(f"break_torque must be positive, got {break_torque}")
        _set_optional_joint_attr(
            joint,
            "CreateBreakTorqueAttr",
            "physics:breakTorque",
            Sdf.ValueTypeNames.Float,
            float(break_torque),
        )

    if exclude_from_articulation:
        _set_optional_joint_attr(
            joint,
            "CreateExcludeFromArticulationAttr",
            "physics:excludeFromArticulation",
            Sdf.ValueTypeNames.Bool,
            True,
        )


# BEND_LIMIT_DEG imported from tree_config at runtime
def _get_bend_limit():
    """Get BEND_LIMIT_DEG from tree_config."""
    import sys
    import os
    # Dynamically import tree_config
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(script_dir)
    sys.path.insert(0, parent_dir)
    from tree_config import BEND_LIMIT_DEG
    return BEND_LIMIT_DEG


def configure_joint_drives(
    joint,
    stiff: float,
    damp: float,
    bend_axes=("rotX", "rotY"),
    bend_limit_deg: float = None,
) -> None:
    """Configure D6 drives on selected bend axes and lock all remaining axes."""
    if bend_limit_deg is None:
        bend_limit_deg = _get_bend_limit()
    
    # Lock all translations
    for axis in ["transX", "transY", "transZ"]:
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        lim.CreateLowAttr().Set(1.0)
        lim.CreateHighAttr().Set(-1.0)

    for axis in ("rotX", "rotY", "rotZ"):
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        if axis in bend_axes:
            lim.CreateLowAttr().Set(-bend_limit_deg)
            lim.CreateHighAttr().Set(bend_limit_deg)
            drv = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
            drv.CreateTypeAttr().Set("force")
            drv.CreateStiffnessAttr().Set(stiff)
            drv.CreateDampingAttr().Set(damp)
            drv.CreateTargetPositionAttr().Set(0.0)
        else:
            lim.CreateLowAttr().Set(1.0)
            lim.CreateHighAttr().Set(-1.0)


def configure_revolute_drive(joint, stiff: float, damp: float) -> None:
    """Configure a planar revolute spring using the same degree-based gains as D6."""
    joint.CreateAxisAttr().Set("X")
    joint.CreateLowerLimitAttr().Set(-_get_bend_limit())
    joint.CreateUpperLimitAttr().Set(_get_bend_limit())
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr().Set("force")
    drive.CreateStiffnessAttr().Set(stiff)
    drive.CreateDampingAttr().Set(damp)
    drive.CreateTargetPositionAttr().Set(0.0)


def anchor_link_to_world(
    stage,
    link_path: str,
    world_position: Gf.Vec3f = None,
    world_orientation: Gf.Quatf = None,
) -> None:
    """Anchor root link to world with a FixedJoint."""
    joint = UsdPhysics.FixedJoint.Define(stage, f"{link_path}/RootFixedJoint")
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])
    if world_position is not None:
        joint.CreateLocalPos0Attr().Set(world_position)
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    if world_orientation is not None:
        joint.CreateLocalRot0Attr().Set(world_orientation)
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def create_internal_joint(
    stage,
    parent_path: str,
    child_path: str,
    joint_name: str,
    parent_height: float,
    gap: float,
    stiff: float,
    damp: float,
    bend_axes=("rotX", "rotY"),
    bend_limit_deg: float = None,
) -> None:
    """
    D6 bending joint between consecutive links in the same chain.

    LocalPos0 = top of parent link = (0, 0, parent_height + gap)
    LocalPos1 = base of child link = (0, 0, 0)
    Both LocalRot are identity (chain direction encoded in world position)
    """
    joint = UsdPhysics.Joint.Define(stage, f"{child_path}/{joint_name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_height + gap))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    configure_joint_drives(
        joint,
        stiff,
        damp,
        bend_axes=bend_axes,
        bend_limit_deg=bend_limit_deg,
    )
    
    # Filter collision between parent and child
    add_collision_filter(stage, child_path, parent_path)


def create_internal_joint_at_rest(
    stage,
    parent_path: str,
    child_path: str,
    joint_name: str,
    parent_local_pos: Gf.Vec3f,
    parent_local_rot: Gf.Quatf,
    stiff: float,
    damp: float,
    bend_axes=("rotX", "rotY"),
    bend_limit_deg: float = None,
) -> None:
    """D6 internal joint preserving an explicit parent-to-child rest frame."""

    joint = UsdPhysics.Joint.Define(stage, f"{child_path}/{joint_name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    joint.CreateLocalPos0Attr().Set(parent_local_pos)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(parent_local_rot)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    configure_joint_drives(
        joint,
        stiff,
        damp,
        bend_axes=bend_axes,
        bend_limit_deg=bend_limit_deg,
    )
    add_collision_filter(stage, child_path, parent_path)


def create_internal_revolute_joint(
    stage,
    parent_path: str,
    child_path: str,
    joint_name: str,
    parent_height: float,
    gap: float,
    stiff: float,
    damp: float,
) -> None:
    """Planar counterpart of ``create_internal_joint`` for solver diagnosis."""
    joint = UsdPhysics.RevoluteJoint.Define(stage, f"{child_path}/{joint_name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_height + gap))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    configure_revolute_drive(joint, stiff, damp)
    add_collision_filter(stage, child_path, parent_path)


def create_attachment_joint(
    stage,
    parent_link_path: str,
    child_link_path: str,
    local_pos0: Gf.Vec3f,
    local_rot0: Gf.Quatf,
    stiff: float,
    damp: float,
    bend_axes=("rotX", "rotY"),
    bend_limit_deg: float = None,
) -> None:
    """
    D6 joint attaching first link of a branch to a parent chain link.

    Args:
        local_pos0: Attachment point in parent-link local frame
        local_rot0: Branch direction in parent-link local frame
        LocalPos1: Always (0,0,0) - base of child link
        LocalRot1: Always identity
    """
    joint = UsdPhysics.Joint.Define(stage, f"{child_link_path}/AttachJoint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link_path)])
    joint.CreateLocalPos0Attr().Set(local_pos0)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(local_rot0)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    configure_joint_drives(
        joint,
        stiff,
        damp,
        bend_axes=bend_axes,
        bend_limit_deg=bend_limit_deg,
    )
    
    # Filter collisions with parent and its neighbor
    add_attachment_collision_filters(stage, child_link_path, parent_link_path)


def create_attachment_revolute_joint(
    stage,
    parent_link_path: str,
    child_link_path: str,
    local_pos0: Gf.Vec3f,
    local_rot0: Gf.Quatf,
    stiff: float,
    damp: float,
) -> None:
    """Attach a branch through one planar rotational spring."""
    joint = UsdPhysics.RevoluteJoint.Define(stage, f"{child_link_path}/AttachJoint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link_path)])
    joint.CreateLocalPos0Attr().Set(local_pos0)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(local_rot0)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    configure_revolute_drive(joint, stiff, damp)
    add_attachment_collision_filters(stage, child_link_path, parent_link_path)


# ==============================================================================
# LOCKED JOINT HELPERS (for testing)
# ==============================================================================

def create_internal_joint_locked(
    stage,
    parent_path: str,
    child_path: str,
    joint_name: str,
    parent_height: float,
    gap: float,
) -> None:
    """
    FixedJoint between consecutive links (for rigid testing).
    
    Used for Isaac Sim integration tests to verify geometry doesn't change
    when joints are completely rigid (no flexibility).
    """
    joint = UsdPhysics.FixedJoint.Define(stage, f"{child_path}/{joint_name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_height + gap))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    
    add_collision_filter(stage, child_path, parent_path)


def create_internal_joint_locked_at_rest(
    stage,
    parent_path: str,
    child_path: str,
    joint_name: str,
    parent_local_pos: Gf.Vec3f,
    parent_local_rot: Gf.Quatf,
) -> None:
    """Fixed internal joint whose rest frame is explicit in the parent body."""

    joint = UsdPhysics.FixedJoint.Define(stage, f"{child_path}/{joint_name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    joint.CreateLocalPos0Attr().Set(parent_local_pos)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(parent_local_rot)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    add_collision_filter(stage, child_path, parent_path)


def create_attachment_joint_locked(
    stage,
    parent_link_path: str,
    child_link_path: str,
    local_pos0: Gf.Vec3f,
    local_rot0: Gf.Quatf,
) -> None:
    """
    FixedJoint attaching branch to parent (for rigid testing).
    
    Creates completely rigid attachment with no flexibility.
    """
    joint = UsdPhysics.FixedJoint.Define(stage, f"{child_link_path}/AttachJoint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link_path)])
    joint.CreateLocalPos0Attr().Set(local_pos0)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(local_rot0)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    
    add_attachment_collision_filters(stage, child_link_path, parent_link_path)


# ==============================================================================
# LEAF NODE ATTACHMENT (for spheres and other terminal bodies)
# ==============================================================================

def create_fixed_joint_to_tip(
    stage,
    parent_link_path: str,
    child_body_path: str,
    parent_height: float,
    child_offset: float = 0.0,
    joint_name: str = "FixedJoint",
    break_force: float = None,
    break_torque: float = None,
    exclude_from_articulation: bool = False,
    local_pos0: Gf.Vec3f = None,
    local_pos1: Gf.Vec3f = None,
    local_rot0: Gf.Quatf = None,
    local_rot1: Gf.Quatf = None,
) -> UsdPhysics.FixedJoint:
    """
    Create FixedJoint attaching a rigid body (leaf node) to the tip of a parent link.
    
    Used for tomatoes attached to pedicel tips, or other terminal bodies that are
    rigidly attached to a branch tip. By default the joint remains part of the
    articulation. When the experimental native detachment path is enabled,
    ``break_force`` and ``exclude_from_articulation`` author the USD attributes
    needed for PhysX breakable fixed joints.
    
    Args:
        stage: USD stage
        parent_link_path: Path to parent link (e.g., pedicel)
        child_body_path: Path to child rigid body (e.g., tomato sphere)
        parent_height: Height of parent cylinder [m]
        child_offset: Additional offset from tip [m] (default: 0.0)
            Positive offset moves child further away from parent tip.
            Useful for positioning sphere center at desired location.
        joint_name: USD prim name for the terminal body's fixed attachment.
        break_force: Optional USD joint break force threshold [N].
        break_torque: Optional USD joint break torque threshold [N*m].
        exclude_from_articulation: Whether PhysX should keep this terminal fixed
            joint out of the articulation chain when native detachment is enabled.
    
    Example:
        # Attach tomato sphere (radius=0.03m) to pedicel (height=0.01m)
        # Sphere center should be 0.03m beyond pedicel tip
        create_fixed_joint_to_tip(
            stage, pedicel_path, tomato_path,
            parent_height=0.01,
            child_offset=0.03  # = sphere radius
        )
    """
    joint = UsdPhysics.FixedJoint.Define(stage, f"{child_body_path}/{joint_name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_body_path)])
    
    # Parent anchor: tip of parent link
    if local_pos0 is not None:
        joint.CreateLocalPos0Attr().Set(local_pos0)
    else:
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_height))
    
    # Child anchor: at child origin, but offset if needed
    # Negative offset because we're measuring from child's perspective
    if local_pos1 is not None:
        joint.CreateLocalPos1Attr().Set(local_pos1)
    else:
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, -child_offset))
    
    # Legacy callers inherit the parent orientation.  Explicit canonical poses
    # may instead provide the relative frame needed to make both world-space
    # joint frames coincide without a reset-time angular projection.
    joint.CreateLocalRot0Attr().Set(
        local_rot0 or Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    )
    joint.CreateLocalRot1Attr().Set(
        local_rot1 or Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    )
    configure_detachable_joint(
        joint,
        break_force=break_force,
        break_torque=break_torque,
        exclude_from_articulation=exclude_from_articulation,
    )
    
    # Filter collision between child and parent
    add_collision_filter(stage, child_body_path, parent_link_path)
    return joint


def create_fixed_joint_attachment(
    stage,
    parent_link_path: str,
    child_body_path: str,
    local_pos0: Gf.Vec3f,
    local_rot0: Gf.Quatf,
) -> None:
    """
    Create FixedJoint attaching a rigid body (leaf node) at arbitrary position on parent.
    
    More general version of create_fixed_joint_to_tip that allows custom attachment
    point and orientation. Used for complex attachment scenarios.
    
    Args:
        stage: USD stage
        parent_link_path: Path to parent link
        child_body_path: Path to child rigid body
        local_pos0: Attachment point in parent-link local frame
        local_rot0: Child orientation in parent-link local frame
    """
    joint = UsdPhysics.FixedJoint.Define(stage, f"{child_body_path}/FixedJoint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_body_path)])
    joint.CreateLocalPos0Attr().Set(local_pos0)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(local_rot0)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    
    # Filter collision between child and parent
    add_collision_filter(stage, child_body_path, parent_link_path)
