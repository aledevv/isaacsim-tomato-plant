"""Per-link rigid organic visual mode for vegetative axes."""

import math

from pxr import Gf

from ..mesh_geometry import build_open_tube_topology
from ..usd.materials import get_or_create_tomato_stem_material
from .mesh import (
    _axis_color,
    _smoothstep,
    _visual_radius,
    author_plain_mesh,
    build_axis_sample_arcs,
    build_parallel_transport_frames,
    centerline_point,
    centerline_tangent,
    link_rest_world,
)
from .model import VisualAxisData


_SEGMENT_TONGUE_MAX_M = 0.006
_SEGMENT_TONGUE_FRACTION = 0.18
_SEGMENT_TONGUE_START_SCALE = 0.90
_SEGMENT_TONGUE_END_SCALE = 0.75
_TERMINAL_TAPER_MAX_M = 0.022
_TERMINAL_TAPER_LINK_FRACTION = 0.42
_CENTERED_FORK_ROOT_OVERLAP_MAX_M = 0.006
_CENTERED_FORK_ROOT_OVERLAP_LINK_FRACTION = 0.16
_CENTERED_FORK_ROOT_START_SCALE = 0.72
_CENTERED_FORK_HOST_DOME_MAX_M = 0.0035
_CENTERED_FORK_HOST_DOME_RADIUS_SCALE = 0.45
# Rounded caps close both ends of every link tube, so a bent joint or a stem
# tip never shows the hollow inside: depth as a fraction of the end radius.
_END_CAP_DEPTH_RADIUS_SCALE = 0.6
_END_CAP_RING_DEPTH = 0.6  # the intermediate ring, as a fraction of the depth
_END_CAP_RING_RADIUS = 0.8  # its radius, as a fraction of the end radius


def _segment_overlap(axis: VisualAxisData, link_index: int) -> float:
    if link_index >= len(axis.link_paths) - 1:
        return 0.0
    current_length = axis.bone_lengths[link_index]
    next_length = axis.bone_lengths[link_index + 1]
    return min(
        _SEGMENT_TONGUE_MAX_M,
        current_length * _SEGMENT_TONGUE_FRACTION,
        next_length * _SEGMENT_TONGUE_FRACTION,
    )


def _centered_fork_root_overlap(axis: VisualAxisData, link_index: int) -> float:
    if link_index != 0 or not axis.members[0].centered_terminal:
        return 0.0
    return min(
        _CENTERED_FORK_ROOT_OVERLAP_MAX_M,
        axis.bone_lengths[0] * _CENTERED_FORK_ROOT_OVERLAP_LINK_FRACTION,
    )


def _is_centered_fork_host(axis: VisualAxisData, link_index: int) -> bool:
    return (
        link_index == len(axis.link_paths) - 1
        and axis.members[-1].centered_terminal_host
    )


def _terminal_taper_start(
    axis: VisualAxisData, link_index: int, terminal_tip_scale: float
):
    if terminal_tip_scale >= 0.999999 or link_index != len(axis.link_paths) - 1:
        return None

    taper_length = min(
        _TERMINAL_TAPER_MAX_M,
        axis.bone_lengths[link_index] * _TERMINAL_TAPER_LINK_FRACTION,
    )
    if taper_length <= 1e-8:
        return None
    return max(axis.bone_starts[link_index], axis.total_length - taper_length)


def _segment_sample_arcs(
    axis: VisualAxisData,
    visual_start: float,
    core_start: float,
    core_end: float,
    tongue_end: float,
    *,
    terminal_taper_start=None,
):
    eps = 1e-10
    arcs = {
        arc
        for arc in build_axis_sample_arcs(axis)
        if visual_start - eps <= arc <= tongue_end + eps
    }
    arcs.update((visual_start, core_start, core_end, tongue_end))

    if visual_start < core_start - eps:
        root_overlap = core_start - visual_start
        arcs.update(
            (
                visual_start + root_overlap * 0.25,
                visual_start + root_overlap * 0.50,
                visual_start + root_overlap * 0.75,
            )
        )

    if tongue_end > core_end + eps:
        overlap = tongue_end - core_end
        arcs.add(core_end + overlap / 3.0)
        arcs.add(core_end + 2.0 * overlap / 3.0)

    if terminal_taper_start is not None:
        taper_length = max(core_end - terminal_taper_start, 0.0)
        arcs.update(
            (
                terminal_taper_start,
                terminal_taper_start + taper_length * 0.25,
                terminal_taper_start + taper_length * 0.50,
                terminal_taper_start + taper_length * 0.75,
                core_end,
            )
        )

    return sorted(arcs)


def _root_overlap_radius_scale(
    arc: float, visual_start: float, core_start: float
) -> float:
    if visual_start >= core_start - 1e-10 or arc >= core_start:
        return 1.0
    q = _smoothstep((arc - visual_start) / max(core_start - visual_start, 1e-8))
    return _CENTERED_FORK_ROOT_START_SCALE + (
        1.0 - _CENTERED_FORK_ROOT_START_SCALE
    ) * q


def _tongue_radius_scale(arc: float, core_end: float, tongue_end: float) -> float:
    if tongue_end <= core_end + 1e-10 or arc <= core_end + 1e-10:
        return 1.0
    q = _smoothstep((arc - core_end) / max(tongue_end - core_end, 1e-8))
    return _SEGMENT_TONGUE_START_SCALE + (
        _SEGMENT_TONGUE_END_SCALE - _SEGMENT_TONGUE_START_SCALE
    ) * q


def _terminal_radius_scale(
    arc: float, core_end: float, taper_start, terminal_tip_scale: float
) -> float:
    if taper_start is None or arc <= taper_start:
        return 1.0
    q = _smoothstep((arc - taper_start) / max(core_end - taper_start, 1e-8))
    return 1.0 + (terminal_tip_scale - 1.0) * q


def _append_centered_host_dome(
    axis: VisualAxisData,
    link_index: int,
    core_end: float,
    radial_segments: int,
    world_to_link: Gf.Matrix4d,
    points,
    face_counts,
    face_indices,
):
    if not _is_centered_fork_host(axis, link_index):
        return

    tip_radius = _visual_radius(axis, core_end)
    dome_depth = min(
        _CENTERED_FORK_HOST_DOME_MAX_M,
        max(tip_radius * _CENTERED_FORK_HOST_DOME_RADIUS_SCALE, 0.0010),
    )
    apex_world = centerline_point(axis, core_end) + centerline_tangent(
        axis, core_end
    ) * dome_depth
    apex_local = world_to_link.Transform(apex_world)
    apex_index = len(points)
    points.append(Gf.Vec3f(*apex_local))

    last_row = len(points) - 1 - radial_segments
    for radial in range(radial_segments):
        next_radial = (radial + 1) % radial_segments
        face_counts.append(3)
        face_indices.extend(
            (last_row + radial, apex_index, last_row + next_radial)
        )


def _append_end_cap(
    center,
    tangent,
    radius: float,
    ring_start: int,
    radial_segments: int,
    normal,
    binormal,
    world_to_link: Gf.Matrix4d,
    points,
    face_counts,
    face_indices,
    *,
    outward: float,
):
    """Close the ring starting at ``ring_start`` with a low rounded dome.

    ``outward`` is +1 for the far end of the tube and -1 for its start, so
    the dome bulges away from the tube and the faces keep facing out.
    """

    depth = radius * _END_CAP_DEPTH_RADIUS_SCALE
    ring_index = len(points)
    ring_center = center + tangent * (outward * depth * _END_CAP_RING_DEPTH)
    for radial in range(radial_segments):
        theta = 2.0 * math.pi * radial / radial_segments
        world_point = ring_center + radius * _END_CAP_RING_RADIUS * (
            normal * math.cos(theta) + binormal * math.sin(theta)
        )
        points.append(Gf.Vec3f(*world_to_link.Transform(world_point)))
    apex_index = len(points)
    points.append(Gf.Vec3f(*world_to_link.Transform(center + tangent * (outward * depth))))
    for radial in range(radial_segments):
        next_radial = (radial + 1) % radial_segments
        a, b = ring_start + radial, ring_start + next_radial
        c, d = ring_index + radial, ring_index + next_radial
        if outward > 0:
            face_counts.extend((3, 3, 3))
            face_indices.extend((a, c, d, a, d, b, c, apex_index, d))
        else:
            face_counts.extend((3, 3, 3))
            face_indices.extend((c, a, b, c, b, d, d, apex_index, c))


def _build_segmented_link_mesh(
    axis: VisualAxisData,
    link_index: int,
    *,
    terminal_tip_scale: float = 1.0,
    last_link_index: int = None,
):
    """Mesh for one link, or for links ``link_index..last_link_index`` as one
    continuous tube expressed in the frame of ``link_index``."""
    if last_link_index is None:
        last_link_index = link_index
    radial_segments = axis.profile.radial_segments
    if radial_segments < 3:
        raise ValueError(
            f"Visual axis '{axis.axis_id}' radial_segments must be at least 3"
        )
    if not 0.1 <= terminal_tip_scale <= 1.0:
        raise ValueError(
            f"terminal_tip_scale must be in [0.1, 1.0], got {terminal_tip_scale}"
        )

    core_start = axis.bone_starts[link_index]
    core_end = min(
        axis.total_length,
        axis.bone_starts[last_link_index] + axis.bone_lengths[last_link_index],
    )
    root_overlap = _centered_fork_root_overlap(axis, link_index)
    visual_start = core_start - root_overlap
    overlap = _segment_overlap(axis, last_link_index)
    tongue_end = min(axis.total_length, core_end + overlap)
    taper_start = _terminal_taper_start(axis, last_link_index, terminal_tip_scale)
    arcs = _segment_sample_arcs(
        axis,
        visual_start,
        core_start,
        core_end,
        tongue_end,
        terminal_taper_start=taper_start,
    )

    normals, binormals = build_parallel_transport_frames(axis, arcs)
    world_to_link = link_rest_world(axis, link_index).GetInverse()
    points = []

    for arc, normal, binormal in zip(arcs, normals, binormals):
        center = centerline_point(axis, arc)
        radius = _visual_radius(axis, arc)
        radius *= _root_overlap_radius_scale(arc, visual_start, core_start)
        radius *= _tongue_radius_scale(arc, core_end, tongue_end)
        radius *= _terminal_radius_scale(
            arc, core_end, taper_start, terminal_tip_scale
        )

        for radial in range(radial_segments):
            theta = 2.0 * math.pi * radial / radial_segments
            world_point = center + radius * (
                normal * math.cos(theta) + binormal * math.sin(theta)
            )
            local_point = world_to_link.Transform(world_point)
            points.append(Gf.Vec3f(*local_point))

    face_counts, face_indices = build_open_tube_topology(
        len(arcs), radial_segments
    )

    # Before the caps: the dome closes the last ring of points.
    _append_centered_host_dome(
        axis,
        last_link_index,
        core_end,
        radial_segments,
        world_to_link,
        points,
        face_counts,
        face_indices,
    )

    end_caps = [(0, -1.0)]
    if not _is_centered_fork_host(axis, last_link_index):
        end_caps.append((len(arcs) - 1, 1.0))
    for index, outward in end_caps:
        arc = arcs[index]
        ring_points = points[index * radial_segments:(index + 1) * radial_segments]
        center_local = sum((Gf.Vec3d(q) for q in ring_points), Gf.Vec3d(0.0)) / radial_segments
        ring_radius = max((Gf.Vec3d(q) - center_local).GetLength() for q in ring_points)
        _append_end_cap(
            centerline_point(axis, arc),
            centerline_tangent(axis, arc),
            ring_radius,
            index * radial_segments,
            radial_segments,
            normals[index],
            binormals[index],
            world_to_link,
            points,
            face_counts,
            face_indices,
            outward=outward,
        )

    return points, face_counts, face_indices, overlap


def is_rigid_axis(axis: VisualAxisData) -> bool:
    """True when every link of the axis is welded to the next (fixed joints)."""
    return all(member.locked_joints for member in axis.members)


def author_segmented_visual_axis(
    stage, axis: VisualAxisData, *, terminal_tip_scale: float = 1.0
) -> dict:
    """Author one organic rigid mesh per PhysX link, with no UsdSkel.

    An axis whose links are all joined by fixed joints never bends, so it gets
    a single continuous mesh on its first link instead (no seams at nodes).
    """
    segment_count = 0
    tongue_count = 0
    material = get_or_create_tomato_stem_material(stage)

    if len(axis.link_paths) > 1 and is_rigid_axis(axis):
        points, face_counts, face_indices, _ = _build_segmented_link_mesh(
            axis,
            0,
            terminal_tip_scale=terminal_tip_scale,
            last_link_index=len(axis.link_paths) - 1,
        )
        author_plain_mesh(
            stage,
            f"{axis.link_paths[0]}/OrganicVisual_01",
            points,
            face_counts,
            face_indices,
            _axis_color(axis),
            material=material,
        )
        return {"segments": 1, "tongues": 0}

    for link_index, link_path in enumerate(axis.link_paths):
        points, face_counts, face_indices, overlap = _build_segmented_link_mesh(
            axis, link_index, terminal_tip_scale=terminal_tip_scale
        )
        author_plain_mesh(
            stage,
            f"{link_path}/OrganicVisual_{link_index + 1:02d}",
            points,
            face_counts,
            face_indices,
            _axis_color(axis),
            material=material,
        )
        segment_count += 1
        if overlap > 0.0:
            tongue_count += 1

    return {"segments": segment_count, "tongues": tongue_count}
