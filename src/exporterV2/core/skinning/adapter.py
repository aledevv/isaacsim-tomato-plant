"""Resolve ExporterV2 branch dictionaries for the skinned backend."""

import math
import re
from typing import Dict, Iterable, List, Tuple

from pxr import Gf

from ..tree_config import (
    BioConfig,
    GAP,
    MIN_LINK_RADIUS_WORLD,
    PhysicsRuntimeConfig,
    TrussPhysicsConfig,
    calculate_physics_params,
    calculate_truss_physics_params,
    compute_flexural_rigidity,
    compute_hinge_stiffness_rad,
    compute_mass,
    scaled,
)
from .model import BranchData, BranchSpec, PhysicsGains, VisualProfile


VALID_SYSTEMS = frozenset(("vegetative", "truss"))
VALID_JOINT_TYPES = frozenset(("fixed", "d6", "d6_planar", "revolute_planar"))


def _visual_profile(branch: dict) -> VisualProfile:
    """Resolve an optional per-branch visual LOD without changing legacy defaults."""

    raw = branch.get("visual_profile")
    if raw is None:
        return VisualProfile()
    if not isinstance(raw, dict):
        raise ValueError(
            f"Branch '{branch.get('id', '<unknown>')}' visual_profile must be a mapping"
        )

    allowed = {
        "radial_segments",
        "axial_spacing_m",
        "radius_transition_samples",
        "junction_bulge_amplitude",
    }
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise ValueError(
            f"Branch '{branch.get('id', '<unknown>')}' visual_profile has "
            f"unsupported fields {unknown}"
        )

    profile = VisualProfile(
        radial_segments=int(raw.get("radial_segments", 20)),
        axial_spacing_m=float(raw.get("axial_spacing_m", 0.005)),
        radius_transition_samples=int(raw.get("radius_transition_samples", 9)),
        junction_bulge_amplitude=float(raw.get("junction_bulge_amplitude", 0.18)),
    )
    if profile.radial_segments < 6:
        raise ValueError(
            f"Branch '{branch.get('id', '<unknown>')}' radial_segments must be >= 6"
        )
    if profile.axial_spacing_m <= 0.0:
        raise ValueError(
            f"Branch '{branch.get('id', '<unknown>')}' axial_spacing_m must be positive"
        )
    if profile.radius_transition_samples < 3:
        raise ValueError(
            f"Branch '{branch.get('id', '<unknown>')}' "
            "radius_transition_samples must be >= 3"
        )
    return profile


def branch_system(branch: dict) -> str:
    """Return explicit classification, with a compatibility fallback."""
    system = branch.get("system")
    if system is None:
        system = "truss" if branch.get("physics_profile") == "truss" else "vegetative"
    if system not in VALID_SYSTEMS:
        raise ValueError(
            f"Branch '{branch.get('id', '<unknown>')}' has invalid system={system!r}; "
            f"expected one of {sorted(VALID_SYSTEMS)}"
        )
    return system


def partition_branches(branches: Iterable[dict]) -> Tuple[List[dict], List[dict]]:
    """Partition definitions while preserving their original order."""
    vegetative = []
    truss = []
    for branch in branches:
        (truss if branch_system(branch) == "truss" else vegetative).append(branch)
    return vegetative, truss


def is_structural_terminal_host(branch_def: dict) -> bool:
    """Return whether a branch definition is a main structural line."""
    branch_id = str(branch_def.get("id", "")).lower()
    kind = str(branch_def.get("kind", "")).lower()
    return (
        branch_def.get("parent") is None
        or branch_id.startswith("branch_r")
        or kind in {"stem", "trunk", "branch", "lateral_branch"}
    )


def is_centered_terminal_leaf(child_def: dict, parent_def: dict, all_branch_defs: Dict[str, dict]) -> bool:
    """Check if the child is the primary terminal continuation of its host."""
    if child_def.get("disable_centered_terminal", False):
        return False
    if branch_system(child_def) != "vegetative":
        return False
    if "petiole" not in str(child_def.get("id", "")).lower():
        return False
    if not is_structural_terminal_host(parent_def):
        return False

    parent_links = int(parent_def.get("n_links", 0))
    if parent_links <= 0:
        return False

    try:
        attach_link = int(child_def.get("attach_link", -1))
        attach_frac = float(child_def.get("attach_frac", 1.0))
    except (TypeError, ValueError):
        return False

    if attach_link != parent_links or attach_frac < 0.95:
        return False

    parent_id = parent_def.get("id")
    candidates = []
    for sibling in all_branch_defs.values():
        if sibling.get("parent") != parent_id:
            continue
        if branch_system(sibling) != "vegetative":
            continue
        if "petiole" not in str(sibling.get("id", "")).lower():
            continue
        try:
            s_attach_link = int(sibling.get("attach_link", -1))
            s_attach_frac = float(sibling.get("attach_frac", 1.0))
            if s_attach_link == parent_links and s_attach_frac >= 0.95:
                candidates.append(sibling)
        except (TypeError, ValueError):
            pass

    if not candidates:
        return False

    chosen = sorted(candidates, key=lambda item: str(item.get("id", "")))[0]
    return child_def.get("id") == chosen.get("id")


def _path_component(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_]", "_", value)
    if not result or result[0].isdigit():
        result = f"Branch_{result}"
    return result


def _quat_from_column_rotation(rows) -> Gf.Quatf:
    """Convert a PlantState column-vector rotation matrix to a Gf quaternion."""

    m00, m01, m02 = (float(value) for value in rows[0][:3])
    m10, m11, m12 = (float(value) for value in rows[1][:3])
    m20, m21, m22 = (float(value) for value in rows[2][:3])
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (m21 - m12) / scale
        y = (m02 - m20) / scale
        z = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / scale
        x = 0.25 * scale
        y = (m01 + m10) / scale
        z = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / scale
        x = (m01 + m10) / scale
        y = 0.25 * scale
        z = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / scale
        x = (m02 + m20) / scale
        y = (m12 + m21) / scale
        z = 0.25 * scale
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    return Gf.Quatf(w / norm, x / norm, y / norm, z / norm)


def _explicit_link_data(branch: dict):
    """Resolve optional per-link dimensions and canonical rest frames."""

    raw_specs = branch.get("link_specs")
    if raw_specs is None:
        return None

    result = []
    for raw in raw_specs:
        length = scaled(float(raw["length"]))
        radius = scaled(float(raw["radius"]))
        frame = raw.get("rest_frame")
        if frame is None:
            start = None
            orientation = None
        else:
            start = Gf.Vec3d(
                scaled(float(frame[0][3])),
                scaled(float(frame[1][3])),
                scaled(float(frame[2][3])),
            )
            orientation = _quat_from_column_rotation(frame)
        result.append((raw, length, radius, start, orientation))
    return result


def _inner_radius(branch: dict) -> float:
    return scaled(branch.get("inner_radius", 0.0))


def _density(branch: dict) -> float:
    default = (
        TrussPhysicsConfig.PLANT_DENSITY
        if branch.get("physics_profile") == "truss"
        else BioConfig.PLANT_DENSITY
    )
    return float(branch.get("density", default))


def _young_modulus(branch: dict) -> float:
    default = (
        TrussPhysicsConfig.YOUNG_MODULUS
        if branch.get("physics_profile") == "truss"
        else BioConfig.YOUNG_MODULUS
    )
    return float(branch.get("young_modulus", default))


def _resolve_gains(
    branch: dict,
    parent: dict | None,
    radius: float,
    inner_radius: float,
    height: float,
    mass: float,
    legacy_physics: bool,
) -> PhysicsGains:
    young_modulus = _young_modulus(branch)
    is_truss = branch.get("physics_profile") == "truss" and inner_radius <= 0.0
    if is_truss:
        stiffness, damping = calculate_truss_physics_params(
            radius,
            height,
            mass,
            young_modulus=young_modulus,
            damping_ratio=float(
                branch.get("damping_ratio", TrussPhysicsConfig.DAMPING_RATIO)
            ),
        )
    else:
        damping_ratio = branch.get("damping_ratio")
        if damping_ratio is None and branch.get("physics_profile") == "truss":
            damping_ratio = TrussPhysicsConfig.DAMPING_RATIO
        stiffness, damping = calculate_physics_params(
            radius,
            height,
            mass,
            legacy_physics=legacy_physics,
            young_modulus=young_modulus,
            damping_ratio=damping_ratio,
            inner_radius=inner_radius,
        )

    if not legacy_physics and parent is not None:
        parent_radius = scaled(parent["radius"])
        parent_inner_radius = _inner_radius(parent)
        parent_height = scaled(parent["height"])
        branch_ei = compute_flexural_rigidity(radius, young_modulus, inner_radius)
        parent_ei = compute_flexural_rigidity(
            parent_radius,
            _young_modulus(parent),
            parent_inner_radius,
        )
        attachment_rad = compute_hinge_stiffness_rad(
            parent_height,
            parent_ei,
            height,
            branch_ei,
        )
        attachment_stiffness = attachment_rad * (math.pi / 180.0)
        ratio = attachment_stiffness / stiffness if stiffness > 0.0 else 1.0
        attachment_damping = damping * math.sqrt(ratio)
    else:
        attachment_stiffness = stiffness * 5.0
        attachment_damping = damping * math.sqrt(5.0)

    if branch.get("attachment_stiffness_rad") is not None:
        attachment_stiffness = (
            float(branch["attachment_stiffness_rad"]) * (math.pi / 180.0)
        )
        ratio = attachment_stiffness / stiffness if stiffness > 0.0 else 1.0
        attachment_damping = damping * math.sqrt(ratio)

    scale = float(branch.get("drive_stiffness_scale", 1.0))
    if scale <= 0.0:
        raise ValueError(
            f"Branch '{branch['id']}' drive_stiffness_scale must be positive, got {scale}"
        )
    damping_scale = math.sqrt(scale)
    return PhysicsGains(
        stiffness=stiffness * scale,
        damping=damping * damping_scale,
        attachment_stiffness=attachment_stiffness * scale,
        attachment_damping=attachment_damping * damping_scale,
    )


def _joint_type(branch: dict, *, is_root: bool, locked_joints: bool) -> Tuple[str, str, bool]:
    requested = branch.get("joint_type", "d6")
    if requested not in VALID_JOINT_TYPES:
        raise ValueError(f"Branch '{branch['id']}' has unsupported joint_type={requested!r}")

    locked = locked_joints or requested == "fixed"
    if is_root and PhysicsRuntimeConfig.RIGID_TRUNK:
        locked = True

    attachment = branch.get("attachment_joint_type", requested)
    if attachment not in VALID_JOINT_TYPES:
        raise ValueError(
            f"Branch '{branch['id']}' has unsupported attachment_joint_type={attachment!r}"
        )
    if locked_joints:
        attachment = "fixed"
    return requested, attachment, locked


def resolve_vegetative_graph(
    branches: Iterable[dict],
    *,
    all_branch_defs: Dict[str, dict] | None = None,
    physics_parent_path: str = "/World/Stem/Vegetative",
    visual_parent_path: str = "/World/PlantVisual",
    locked_joints: bool = False,
    legacy_physics: bool = False,
) -> List[BranchData]:
    """Resolve vegetative branches using the exact legacy V2 rest-pose rules."""
    definitions = list(branches)
    if any(branch_system(branch) != "vegetative" for branch in definitions):
        raise ValueError("resolve_vegetative_graph accepts only vegetative branches")

    if all_branch_defs is None:
        all_branch_defs = {branch["id"]: branch for branch in definitions}
    for branch in definitions:
        parent_id = branch.get("parent")
        if parent_id is not None:
            parent = all_branch_defs.get(parent_id)
            if parent is None:
                raise ValueError(f"Branch '{branch['id']}' references missing parent '{parent_id}'")
            if branch_system(parent) == "truss":
                raise ValueError(
                    f"Vegetative branch '{branch['id']}' cannot have truss parent '{parent_id}'"
                )

    unresolved = list(definitions)
    resolved: List[BranchData] = []
    by_id: Dict[str, BranchData] = {}
    gap = scaled(GAP)

    while unresolved:
        progress = False
        for branch in list(unresolved):
            branch_id = branch["id"]
            parent_id = branch.get("parent")
            if parent_id is not None and parent_id not in by_id:
                continue

            is_root = parent_id is None
            radius = scaled(branch["radius"])
            inner_radius = _inner_radius(branch)
            height = scaled(branch["height"])
            n_links = int(branch["n_links"])
            explicit_links = _explicit_link_data(branch)
            explicit_link_poses = bool(
                explicit_links and explicit_links[0][3] is not None
            )
            if explicit_links is None:
                link_lengths = [height] * n_links
                link_radii = [radius] * n_links
                link_metadata = [{} for _ in range(n_links)]
            else:
                link_lengths = [item[1] for item in explicit_links]
                link_radii = [item[2] for item in explicit_links]
                link_metadata = [dict(item[0]) for item in explicit_links]
            link_collider_radii = [
                max(value * 0.90, MIN_LINK_RADIUS_WORLD)
                for value in link_radii
            ]
            link_masses = [
                compute_mass(
                    link_radius,
                    link_length,
                    density=_density(branch),
                    inner_radius=min(inner_radius, link_radius * 0.99),
                )
                for link_length, link_radius in zip(link_lengths, link_radii)
            ]
            mass = link_masses[0]

            parent_definition = all_branch_defs.get(parent_id) if parent_id else None
            gains = _resolve_gains(
                branch,
                parent_definition,
                radius,
                inner_radius,
                height,
                mass,
                legacy_physics,
            )
            joint_type, attachment_joint_type, locked = _joint_type(
                branch,
                is_root=is_root,
                locked_joints=locked_joints,
            )

            parent_link_index = None
            local_pos0 = None
            local_rot0 = None
            centered_terminal = False

            if not is_root:
                parent_link_index = int(branch["attach_link"]) - 1
                parent_data = by_id[parent_id]
                if not 0 <= parent_link_index < parent_data.n_links:
                    raise ValueError(
                        f"Branch '{branch_id}' attach_link={branch['attach_link']} is outside "
                        f"parent '{parent_id}' link range"
                    )

            if explicit_link_poses:
                link_bases = [item[3] for item in explicit_links]
                link_orientations = [item[4] for item in explicit_links]
                start = link_bases[0]
                orientation = link_orientations[0]
                first_rotation = Gf.Rotation(Gf.Quatd(orientation))
                axis = Gf.Vec3d(
                    first_rotation.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
                ).GetNormalized()
            elif is_root:
                start = Gf.Vec3d(0.0, 0.0, 0.0)
                axis = Gf.Vec3d(0.0, 0.0, 1.0)
                orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
            else:
                parent_data = by_id[parent_id]
                parent_definition = all_branch_defs[parent_id]

                tilt = float(branch.get("tilt", 0.0))
                rot = float(branch.get("rot", 0.0))
                roll = float(branch.get("roll", 0.0))
                rot_z = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), rot)
                rot_tilt = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -tilt)
                rot_roll = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), roll)
                branch_in_parent = rot_roll * rot_tilt * rot_z
                parent_rotation = Gf.Rotation(Gf.Quatd(parent_data.orientation))
                combined = branch_in_parent * parent_rotation
                axis = Gf.Vec3d(combined.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)))
                axis.Normalize()
                orientation = Gf.Quatf(combined.GetQuat())

                # A terminal petiole on a structural branch gets a true centerline
                # attachment. This is preferable to drawing a separate visual bridge:
                # the real leaf branch itself becomes one arm of the Y, its joint
                # remains physical, and there is no hollow/off-axis sleeve at the
                # structural tip.
                centered_terminal = is_centered_terminal_leaf(branch, parent_definition, all_branch_defs)
                radial_distance = (
                    0.0
                    if centered_terminal or (tilt == 0.0 and rot == 0.0)
                    else parent_data.radius / 2.0
                )
                attach_frac = float(branch.get("attach_frac", 1.0))
                z_local = (
                    parent_data.link_height
                    if centered_terminal and attach_frac >= 1.0
                    else (
                        parent_data.link_height + gap
                        if attach_frac >= 1.0
                        else attach_frac * parent_data.link_height
                    )
                )
                base_offset = Gf.Vec3d(0.0, radial_distance, z_local)
                offset_parent = rot_z.TransformDir(base_offset)
                offset_world = parent_rotation.TransformDir(offset_parent)
                start = parent_data.link_bases[parent_link_index] + offset_world
                local_pos0 = Gf.Vec3f(*offset_parent)
                local_rot0 = Gf.Quatf(branch_in_parent.GetQuat())

            if not explicit_link_poses:
                link_orientations = [orientation] * n_links
                cursor = Gf.Vec3d(start)
                link_bases = []
                for link_length, link_orientation in zip(
                    link_lengths, link_orientations
                ):
                    link_bases.append(Gf.Vec3d(cursor))
                    rotation = Gf.Rotation(Gf.Quatd(link_orientation))
                    direction = Gf.Vec3d(
                        rotation.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
                    ).GetNormalized()
                    cursor += direction * (link_length + gap)

            if explicit_link_poses and not is_root:
                parent_data = by_id[parent_id]
                centered_terminal = is_centered_terminal_leaf(
                    branch, parent_definition, all_branch_defs
                )
                parent_rotation = Gf.Rotation(
                    Gf.Quatd(parent_data.link_orientations[parent_link_index])
                )
                offset_world = start - parent_data.link_bases[parent_link_index]
                local_offset = parent_rotation.GetInverse().TransformDir(offset_world)
                child_rotation = Gf.Rotation(Gf.Quatd(orientation))
                relative = child_rotation * parent_rotation.GetInverse()
                local_pos0 = Gf.Vec3f(*local_offset)
                local_rot0 = Gf.Quatf(relative.GetQuat())

            safe_id = _path_component(branch_id)
            physics_root = f"{physics_parent_path}/{safe_id}"
            link_paths = []
            for index in range(n_links):
                suffix = ""
                if explicit_links is not None:
                    suffix = f"_{_path_component(str(link_metadata[index]['id']))}"
                link_paths.append(
                    f"{physics_root}/{safe_id}_Link_{index + 1:02d}{suffix}"
                )
            visual_root = f"{visual_parent_path}/{safe_id}"
            skel_root = f"{visual_root}/SkelRoot"

            data = BranchData(
                definition=branch,
                spec=BranchSpec(
                    physics_links=n_links,
                    radius=radius,
                    inner_radius=inner_radius,
                    link_height=height,
                    density=_density(branch),
                    young_modulus=_young_modulus(branch),
                    visual=_visual_profile(branch),
                ),
                branch_id=branch_id,
                parent_id=parent_id,
                n_links=n_links,
                radius=radius,
                inner_radius=inner_radius,
                link_height=height,
                start=start,
                axis=axis,
                orientation=orientation,
                link_bases=link_bases,
                link_orientations=link_orientations,
                link_lengths=link_lengths,
                link_radii=link_radii,
                link_collider_radii=link_collider_radii,
                link_masses=link_masses,
                link_metadata=link_metadata,
                link_paths=link_paths,
                physics_root_path=physics_root,
                visual_root_path=visual_root,
                skel_root_path=skel_root,
                skeleton_path=f"{skel_root}/Skeleton",
                animation_path=f"{skel_root}/SkelAnim",
                mesh_path=f"{skel_root}/BranchMesh",
                mass=mass,
                gains=gains,
                joint_type=joint_type,
                attachment_joint_type=attachment_joint_type,
                bend_limit_deg=branch.get("bend_limit_deg"),
                locked_joints=locked,
                parent_link_index=parent_link_index,
                attachment_local_pos0=local_pos0,
                attachment_local_rot0=local_rot0,
                centered_terminal=centered_terminal,
                explicit_link_poses=explicit_link_poses,
            )
            resolved.append(data)
            by_id[branch_id] = data

            if centered_terminal and parent_id is not None:
                by_id[parent_id].centered_terminal_host = True

            unresolved.remove(branch)
            progress = True

        if not progress:
            ids = [branch["id"] for branch in unresolved]
            raise ValueError(f"Cannot resolve vegetative graph order/cycle for branches: {ids}")

    return resolved
