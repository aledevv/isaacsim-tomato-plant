"""Author articulated rigid branch chains shared by both V2 backends."""

import math

from pxr import Gf

from ..tree_config import (
    BioConfig,
    GAP,
    PhysicsRuntimeConfig,
    PlantColors,
    TrussPhysicsConfig,
    calculate_physics_params,
    calculate_truss_physics_params,
    compute_flexural_rigidity,
    compute_hinge_stiffness_rad,
    compute_mass,
    scaled,
)
from .geometry import create_rigid_segment
from .joints import (
    anchor_link_to_world,
    create_attachment_joint,
    create_attachment_joint_locked,
    create_attachment_revolute_joint,
    create_internal_joint,
    create_internal_joint_locked,
    create_internal_revolute_joint,
)


def _branch_inner_radius_world(branch_def: dict) -> float:
    return scaled(branch_def.get("inner_radius", 0.0))


def _branch_density(branch_def: dict, use_truss_physics: bool = False) -> float:
    if "density" in branch_def:
        return branch_def["density"]
    if use_truss_physics:
        return TrussPhysicsConfig.PLANT_DENSITY
    return BioConfig.PLANT_DENSITY


def _branch_young_modulus(
    branch_def: dict, use_truss_physics: bool = False
) -> float:
    if "young_modulus" in branch_def:
        return branch_def["young_modulus"]
    if use_truss_physics:
        return TrussPhysicsConfig.YOUNG_MODULUS
    return BioConfig.YOUNG_MODULUS


def _branch_damping_ratio(branch_def: dict, use_truss_physics: bool = False):
    if "damping_ratio" in branch_def:
        return branch_def["damping_ratio"]
    if use_truss_physics:
        return TrussPhysicsConfig.DAMPING_RATIO
    return None


def is_truss_branch(branch_def: dict) -> bool:
    """Honor explicit classification while retaining old profile-only configs."""
    return (
        branch_def.get("system") == "truss"
        or branch_def.get("physics_profile") == "truss"
    )


def build_chain(
    stage,
    stem_path: str,
    branch_def: dict,
    start_world_pos: Gf.Vec3d,
    chain_axis: Gf.Vec3d,
    is_root: bool = False,
    parent_link_path: str = None,
    attachment_local_pos0: Gf.Vec3f = None,
    attachment_local_rot0: Gf.Quatf = None,
    chain_orientation: Gf.Quatf = None,
    locked_joints: bool = False,
    use_truss_physics: bool = False,
    parent_def: dict = None,
    legacy_physics: bool = False,
):
    """Build one articulated chain and return its link paths and world bases."""
    branch_joint_type = branch_def.get("joint_type")
    if branch_joint_type == "fixed":
        locked_joints = True
    elif branch_joint_type in {"d6", "d6_planar", "revolute_planar"}:
        locked_joints = False
    elif is_root and PhysicsRuntimeConfig.RIGID_TRUNK:
        locked_joints = True

    radius = scaled(branch_def["radius"])
    height = scaled(branch_def["height"])
    inner_radius = _branch_inner_radius_world(branch_def)
    gap = scaled(GAP)
    link_count = branch_def["n_links"]
    branch_id = branch_def["id"]
    density = _branch_density(branch_def, use_truss_physics=use_truss_physics)
    mass = compute_mass(
        radius, height, density=density, inner_radius=inner_radius
    )

    if use_truss_physics:
        young_modulus = _branch_young_modulus(
            branch_def, use_truss_physics=True
        )
        damping_ratio = _branch_damping_ratio(
            branch_def, use_truss_physics=True
        )
        if (
            "young_modulus" in branch_def
            or "inner_radius" in branch_def
            or damping_ratio is not None
        ):
            stiffness, damping = calculate_physics_params(
                radius,
                height,
                mass,
                legacy_physics=legacy_physics,
                young_modulus=young_modulus,
                damping_ratio=damping_ratio,
                inner_radius=inner_radius,
            )
        else:
            stiffness, damping = calculate_truss_physics_params(
                radius, height, mass
            )
    else:
        young_modulus = _branch_young_modulus(branch_def)
        stiffness, damping = calculate_physics_params(
            radius,
            height,
            mass,
            legacy_physics=legacy_physics,
            young_modulus=young_modulus,
            damping_ratio=_branch_damping_ratio(branch_def),
            inner_radius=inner_radius,
        )

    if not legacy_physics and not is_root and parent_def is not None:
        parent_radius = scaled(parent_def["radius"])
        parent_height = scaled(parent_def["height"])
        parent_inner_radius = _branch_inner_radius_world(parent_def)
        parent_uses_truss_physics = is_truss_branch(parent_def)
        parent_young_modulus = _branch_young_modulus(
            parent_def, use_truss_physics=parent_uses_truss_physics
        )
        branch_ei = compute_flexural_rigidity(
            radius, young_modulus, inner_radius
        )
        parent_ei = compute_flexural_rigidity(
            parent_radius, parent_young_modulus, parent_inner_radius
        )
        attachment_stiffness_rad = compute_hinge_stiffness_rad(
            parent_height, parent_ei, height, branch_ei
        )
        attachment_stiffness = attachment_stiffness_rad * (math.pi / 180.0)
        stiffness_ratio = (
            attachment_stiffness / stiffness if stiffness > 0 else 1.0
        )
        attachment_damping = damping * math.sqrt(stiffness_ratio)
    else:
        attachment_stiffness = stiffness * 5.0
        attachment_damping = damping * 2.236

    if branch_def.get("attachment_stiffness_rad") is not None:
        attachment_stiffness_override = float(
            branch_def["attachment_stiffness_rad"]
        ) * (3.141592653589793 / 180.0)
        stiffness_ratio = (
            attachment_stiffness_override / stiffness
            if stiffness > 0
            else 1.0
        )
        attachment_stiffness = attachment_stiffness_override
        attachment_damping = damping * math.sqrt(stiffness_ratio)

    drive_stiffness_scale = float(
        branch_def.get("drive_stiffness_scale", 1.0)
    )
    if drive_stiffness_scale <= 0.0:
        raise ValueError(
            f"Branch '{branch_id}' drive_stiffness_scale must be positive, "
            f"got {drive_stiffness_scale}"
        )
    damping_scale = math.sqrt(drive_stiffness_scale)
    stiffness *= drive_stiffness_scale
    damping *= damping_scale
    attachment_stiffness *= drive_stiffness_scale
    attachment_damping *= damping_scale
    bend_limit_deg = branch_def.get("bend_limit_deg")

    step = chain_axis * (height + gap)
    link_paths = []
    link_world_bases = []
    previous_link = None
    current_position = start_world_pos

    for index in range(link_count):
        link_name = f"{branch_id}_Link_{index + 1:02d}"
        branch_kind = branch_def.get("kind", "")
        branch_id_lower = branch_id.lower()
        if branch_kind == "pedicel" or "pedicel" in branch_id_lower:
            link_color = PlantColors.PEDICEL
        elif (
            branch_kind in ("truss_rachis", "rachis")
            or "rachis" in branch_id_lower
        ):
            link_color = PlantColors.TRUSS_RACHIS
        elif branch_kind == "petiolule" or "petiolule" in branch_id_lower:
            link_color = PlantColors.PETIOLULE
        elif branch_kind == "petiole" or "petiole" in branch_id_lower:
            link_color = PlantColors.PETIOLE
        else:
            link_color = PlantColors.STEM

        link_path = create_rigid_segment(
            stage,
            stem_path,
            link_name,
            radius,
            height,
            current_position,
            mass,
            orientation=chain_orientation,
            collision_enabled=branch_def.get("collision_enabled", True),
            color=link_color,
        )

        if previous_link is None:
            if is_root:
                anchor_link_to_world(stage, link_path)
            elif locked_joints or branch_def.get("attachment_joint_type") == "fixed":
                create_attachment_joint_locked(
                    stage,
                    parent_link_path,
                    link_path,
                    attachment_local_pos0,
                    attachment_local_rot0,
                )
            elif branch_joint_type == "revolute_planar":
                create_attachment_revolute_joint(
                    stage,
                    parent_link_path,
                    link_path,
                    attachment_local_pos0,
                    attachment_local_rot0,
                    attachment_stiffness,
                    attachment_damping,
                )
            else:
                create_attachment_joint(
                    stage,
                    parent_link_path,
                    link_path,
                    attachment_local_pos0,
                    attachment_local_rot0,
                    attachment_stiffness,
                    attachment_damping,
                    bend_axes=("rotX",)
                    if branch_joint_type == "d6_planar"
                    else ("rotX", "rotY"),
                    bend_limit_deg=bend_limit_deg,
                )
        elif locked_joints:
            create_internal_joint_locked(
                stage,
                previous_link,
                link_path,
                f"Joint_{index:02d}_{index + 1:02d}",
                height,
                gap,
            )
        elif branch_joint_type == "revolute_planar":
            create_internal_revolute_joint(
                stage,
                previous_link,
                link_path,
                f"Joint_{index:02d}_{index + 1:02d}",
                height,
                gap,
                stiffness,
                damping,
            )
        else:
            create_internal_joint(
                stage,
                previous_link,
                link_path,
                f"Joint_{index:02d}_{index + 1:02d}",
                height,
                gap,
                stiffness,
                damping,
                bend_axes=("rotX",)
                if branch_joint_type == "d6_planar"
                else ("rotX", "rotY"),
                bend_limit_deg=bend_limit_deg,
            )

        link_paths.append(link_path)
        link_world_bases.append(current_position)
        previous_link = link_path
        current_position = current_position + step

    return link_paths, link_world_bases
