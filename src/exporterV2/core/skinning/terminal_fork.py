"""Visual-only terminal leaf forks for the segmented backend."""

import hashlib
import math
from typing import Dict, Iterable, Optional

from pxr import Gf

from ..mesh_geometry import build_open_tube_topology
from ..tree_config import PlantColors
from .adapter import branch_system, is_structural_terminal_host
from .mesh import (
    _axis_color,
    _smoothstep,
    _visual_radius,
    author_plain_mesh,
    centerline_point,
    centerline_tangent,
    link_rest_world,
)
from .model import BranchData, VisualAxisData


RADIAL_SEGMENTS_MIN = 10
CURVE_SAMPLES = 11

ROOT_OVERLAP_MAX_M = 0.005
ROOT_OVERLAP_LINK_FRACTION = 0.10
CONTROL_FORWARD_MAX_M = 0.008
CONTROL_FORWARD_LENGTH_FRACTION = 0.25

SHOOT_LENGTH_MIN_M = 0.014
SHOOT_LENGTH_MAX_M = 0.026
SHOOT_LENGTH_RADIUS_SCALE = 3.0

ROOT_RADIUS_SCALE = 0.32
SHOULDER_RADIUS_SCALE = 0.23
TIP_RADIUS_SCALE = 0.11
ROOT_ZONE_FRACTION = 0.20

LEAF_LENGTH_FRACTION = 0.42
LEAF_LENGTH_MIN_M = 0.008
LEAF_LENGTH_MAX_M = 0.014
LEAF_HALF_WIDTH_FRACTION = 0.27

AZIMUTH_JITTER_DEG = 75.0


def _normalized(vector: Gf.Vec3d) -> Gf.Vec3d:
    vector = Gf.Vec3d(vector)
    if vector.GetLength() <= 1e-10:
        raise ValueError("Cannot normalize a zero-length terminal-fork vector")
    vector.Normalize()
    return vector


def _dot(a: Gf.Vec3d, b: Gf.Vec3d) -> float:
    return float(Gf.Dot(a, b))


def _quadratic_point(p0, p1, p2, t: float) -> Gf.Vec3d:
    u = 1.0 - t
    return p0 * (u * u) + p1 * (2.0 * u * t) + p2 * (t * t)


def _quadratic_tangent(p0, p1, p2, t: float) -> Gf.Vec3d:
    tangent = (p1 - p0) * (2.0 * (1.0 - t)) + (p2 - p1) * (2.0 * t)
    return _normalized(tangent)


def _transport_frames(tangents):
    first = tangents[0]
    reference = Gf.Vec3d(0.0, 0.0, 1.0)
    if abs(_dot(first, reference)) > 0.92:
        reference = Gf.Vec3d(1.0, 0.0, 0.0)

    normal = _normalized(Gf.Cross(reference, first))
    binormal = _normalized(Gf.Cross(first, normal))

    normals = [normal]
    binormals = [binormal]
    previous_tangent = first
    previous_normal = normal

    for tangent in tangents[1:]:
        axis = Gf.Cross(previous_tangent, tangent)
        if axis.GetLength() > 1e-10:
            axis = _normalized(axis)
            cosine = max(-1.0, min(1.0, _dot(previous_tangent, tangent)))
            rotation = Gf.Rotation(axis, math.degrees(math.acos(cosine)))
            normal = _normalized(rotation.TransformDir(previous_normal))
        else:
            normal = previous_normal

        binormal = _normalized(Gf.Cross(tangent, normal))
        normal = _normalized(Gf.Cross(binormal, tangent))
        normals.append(normal)
        binormals.append(binormal)
        previous_tangent = tangent
        previous_normal = normal

    return normals, binormals




def _is_supported_existing_organ(branch_def: dict) -> bool:
    """Fork dressing is intentionally leaf-only; never modify truss/tomato tips."""
    if branch_system(branch_def) != "vegetative":
        return False
    branch_id = str(branch_def.get("id", "")).lower()
    return "petiole" in branch_id


def _find_terminal_existing_child(
    parent: BranchData,
    all_branch_defs: Dict[str, dict],
) -> Optional[dict]:
    candidates = []
    for child in all_branch_defs.values():
        if child.get("parent") != parent.branch_id:
            continue
        if child.get("disable_centered_terminal", False):
            continue
        if child.get("disable_terminal_fork", False):
            continue
        if not _is_supported_existing_organ(child):
            continue

        try:
            attach_link = int(child.get("attach_link", -1))
            attach_frac = float(child.get("attach_frac", 1.0))
        except (TypeError, ValueError):
            continue

        if attach_link != parent.n_links or attach_frac < 0.95:
            continue
        candidates.append(child)

    if not candidates:
        return None
    return sorted(candidates, key=lambda item: str(item.get("id", "")))[0]


def _child_axis_from_definition(parent: BranchData, child_def: dict) -> Gf.Vec3d:
    link_specs = child_def.get("link_specs")
    if link_specs and link_specs[0].get("rest_frame") is not None:
        frame = link_specs[0]["rest_frame"]
        return _normalized(
            Gf.Vec3d(*(float(frame[row][2]) for row in range(3)))
        )

    tilt = float(child_def.get("tilt", 0.0))
    rot = float(child_def.get("rot", 0.0))
    roll = float(child_def.get("roll", 0.0))

    rot_z = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), rot)
    rot_tilt = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -tilt)
    rot_roll = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), roll)
    branch_in_parent = rot_roll * rot_tilt * rot_z
    parent_rotation = Gf.Rotation(Gf.Quatd(parent.orientation))
    combined = branch_in_parent * parent_rotation
    return _normalized(
        Gf.Vec3d(combined.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)))
    )


def _stable_azimuth_jitter(parent_id: str, child_id: str) -> float:
    """Return a deterministic angle in [-AZIMUTH_JITTER_DEG, +...]."""
    key = f"{parent_id}|{child_id}|terminal-fork-azimuth".encode("utf-8")
    digest = hashlib.blake2b(key, digest_size=8).digest()
    integer = int.from_bytes(digest, byteorder="big", signed=False)
    unit = integer / float((1 << 64) - 1)
    return (unit * 2.0 - 1.0) * AZIMUTH_JITTER_DEG


def _complementary_shoot_direction(
    parent_axis: Gf.Vec3d,
    existing_axis: Gf.Vec3d,
    azimuth_jitter_deg: float,
) -> Gf.Vec3d:
    """Place the twig opposite the petiole with stable azimuth variation."""
    parent_axis = _normalized(parent_axis)
    existing_axis = _normalized(existing_axis)
    world_up = Gf.Vec3d(0.0, 0.0, 1.0)

    lateral = existing_axis - parent_axis * _dot(existing_axis, parent_axis)
    if lateral.GetLength() <= 1e-8:
        lateral = Gf.Cross(parent_axis, world_up)
        if lateral.GetLength() <= 1e-8:
            lateral = Gf.Cross(parent_axis, Gf.Vec3d(1.0, 0.0, 0.0))
    lateral = _normalized(lateral)

    opposite = -lateral
    rotation = Gf.Rotation(parent_axis, azimuth_jitter_deg)
    varied_radial = _normalized(rotation.TransformDir(opposite))

    upward = world_up - parent_axis * _dot(world_up, parent_axis)
    if upward.GetLength() > 1e-8:
        upward = _normalized(upward)
    else:
        upward = Gf.Vec3d(0.0, 0.0, 0.0)

    return _normalized(
        parent_axis * 0.36
        + varied_radial * 0.82
        + upward * 0.18
    )


def _shoot_radii(parent_radius: float, count: int):
    root_radius = parent_radius * ROOT_RADIUS_SCALE
    shoulder_radius = parent_radius * SHOULDER_RADIUS_SCALE
    tip_radius = parent_radius * TIP_RADIUS_SCALE
    radii = []

    for index in range(count):
        t = index / float(max(count - 1, 1))
        if t <= ROOT_ZONE_FRACTION:
            q = _smoothstep(t / max(ROOT_ZONE_FRACTION, 1e-8))
            radius = root_radius + (shoulder_radius - root_radius) * q
        else:
            q = _smoothstep(
                (t - ROOT_ZONE_FRACTION) / max(1.0 - ROOT_ZONE_FRACTION, 1e-8)
            )
            radius = shoulder_radius + (tip_radius - shoulder_radius) * q
        radii.append(radius)

    return radii


def _build_shoot_mesh(
    axis: VisualAxisData,
    junction: Gf.Vec3d,
    shoot_direction: Gf.Vec3d,
    parent_radius: float,
):
    parent_axis = _normalized(centerline_tangent(axis, axis.total_length))
    last_length = axis.bone_lengths[-1]
    overlap = min(
        ROOT_OVERLAP_MAX_M,
        last_length * ROOT_OVERLAP_LINK_FRACTION,
    )

    shoot_length = max(
        SHOOT_LENGTH_MIN_M,
        min(SHOOT_LENGTH_MAX_M, parent_radius * SHOOT_LENGTH_RADIUS_SCALE),
    )
    control_forward = min(
        CONTROL_FORWARD_MAX_M,
        shoot_length * CONTROL_FORWARD_LENGTH_FRACTION,
    )

    p0 = junction - parent_axis * overlap
    p1 = junction + parent_axis * control_forward
    p2 = junction + shoot_direction * shoot_length

    centers = []
    tangents = []
    for index in range(CURVE_SAMPLES):
        t = index / float(CURVE_SAMPLES - 1)
        centers.append(_quadratic_point(p0, p1, p2, t))
        tangents.append(_quadratic_tangent(p0, p1, p2, t))

    radii = _shoot_radii(parent_radius, len(centers))
    normals, binormals = _transport_frames(tangents)
    radial_segments = max(RADIAL_SEGMENTS_MIN, axis.profile.radial_segments)
    world_to_link = link_rest_world(axis, -1).GetInverse()

    points = []
    for center, normal, binormal, radius in zip(
        centers,
        normals,
        binormals,
        radii,
    ):
        for radial in range(radial_segments):
            theta = 2.0 * math.pi * radial / radial_segments
            world_point = center + radius * (
                normal * math.cos(theta) + binormal * math.sin(theta)
            )
            points.append(Gf.Vec3f(*world_to_link.Transform(world_point)))

    face_counts, face_indices = build_open_tube_topology(
        len(centers), radial_segments
    )

    tip_center = len(points)
    points.append(Gf.Vec3f(*world_to_link.Transform(centers[-1])))
    last_row = (len(centers) - 1) * radial_segments
    for radial in range(radial_segments):
        next_radial = (radial + 1) % radial_segments
        face_counts.append(3)
        face_indices.extend((
            tip_center,
            last_row + radial,
            last_row + next_radial,
        ))

    return (
        points,
        face_counts,
        face_indices,
        centers[-1],
        tangents[-1],
        world_to_link,
        shoot_length,
    )


def _author_small_leaf(
    stage,
    path: str,
    root_world: Gf.Vec3d,
    shoot_tangent: Gf.Vec3d,
    world_to_link: Gf.Matrix4d,
    shoot_length: float,
):
    length = max(
        LEAF_LENGTH_MIN_M,
        min(LEAF_LENGTH_MAX_M, shoot_length * LEAF_LENGTH_FRACTION),
    )
    half_width = length * LEAF_HALF_WIDTH_FRACTION

    forward = _normalized(shoot_tangent + Gf.Vec3d(0.08, 0.03, 0.10))
    side = Gf.Cross(Gf.Vec3d(0.0, 0.0, 1.0), forward)
    if side.GetLength() <= 1e-8:
        side = Gf.Cross(Gf.Vec3d(0.0, 1.0, 0.0), forward)
    side = _normalized(side)
    bend = _normalized(Gf.Cross(forward, side))

    world_points = [
        root_world,
        root_world + forward * (length * 0.28) + side * (half_width * 0.92),
        root_world + forward * (length * 0.62) + side * (half_width * 0.72) + bend * 0.0010,
        root_world + forward * length,
        root_world + forward * (length * 0.62) - side * (half_width * 0.72) + bend * 0.0010,
        root_world + forward * (length * 0.28) - side * (half_width * 0.92),
    ]
    points = [Gf.Vec3f(*world_to_link.Transform(point)) for point in world_points]
    author_plain_mesh(
        stage,
        path,
        points,
        [3, 3, 3, 3],
        [0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5],
        PlantColors.LEAF_BLADE,
    )


def author_terminal_visual_fork(
    stage,
    axis: VisualAxisData,
    existing_child: dict,
) -> dict:
    parent = axis.members[-1]
    child_axis = _child_axis_from_definition(parent, existing_child)
    azimuth_jitter_deg = _stable_azimuth_jitter(
        parent.branch_id,
        str(existing_child.get("id", "")),
    )
    shoot_direction = _complementary_shoot_direction(
        axis.axis,
        child_axis,
        azimuth_jitter_deg,
    )
    junction = centerline_point(axis, axis.total_length)
    parent_radius = _visual_radius(axis, axis.total_length)

    (
        points,
        face_counts,
        face_indices,
        shoot_tip,
        shoot_tangent,
        world_to_link,
        shoot_length,
    ) = _build_shoot_mesh(axis, junction, shoot_direction, parent_radius)

    root_path = axis.link_paths[-1]
    author_plain_mesh(
        stage,
        f"{root_path}/TerminalForkYoungShoot",
        points,
        face_counts,
        face_indices,
        _axis_color(axis),
    )
    _author_small_leaf(
        stage,
        f"{root_path}/TerminalForkYoungLeaf",
        shoot_tip,
        shoot_tangent,
        world_to_link,
        shoot_length,
    )

    return {
        "parent": parent.branch_id,
        "existing_child": existing_child["id"],
        "existing_system": branch_system(existing_child),
        "azimuth_jitter_deg": azimuth_jitter_deg,
    }


def author_terminal_visual_forks(
    stage,
    visual_axes: Iterable[VisualAxisData],
    all_branch_defs: Dict[str, dict],
) -> list:
    authored = []

    for axis in visual_axes:
        parent = axis.members[-1]
        if not is_structural_terminal_host(parent.definition):
            continue

        existing_child = _find_terminal_existing_child(parent, all_branch_defs)
        if existing_child is None:
            continue

        authored.append(author_terminal_visual_fork(stage, axis, existing_child))

    return authored
