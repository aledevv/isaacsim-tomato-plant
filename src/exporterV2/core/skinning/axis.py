"""Grouping and validation for continuous vegetative visual axes."""

import math
import re
from collections import defaultdict
from typing import Dict, Iterable, List

from pxr import Gf

from ..tree_config import scaled
from .model import BranchData, VisualAxisData, VisualSegment


_CHAIN_TOLERANCE = 1e-6


def _path_component(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_]", "_", value)
    if not result or result[0].isdigit():
        result = f"Axis_{result}"
    return result


def _ordered_members(axis_id: str, members: List[BranchData]) -> List[BranchData]:
    if len(members) == 1:
        return members

    by_id = {member.branch_id: member for member in members}
    roots = [member for member in members if member.parent_id not in by_id]
    if len(roots) != 1:
        raise ValueError(
            f"Visual axis '{axis_id}' must be one linear chain; found {len(roots)} roots"
        )

    children = defaultdict(list)
    for member in members:
        if member.parent_id in by_id:
            children[member.parent_id].append(member)

    ordered = []
    current = roots[0]
    while current is not None:
        ordered.append(current)
        next_members = children.get(current.branch_id, [])
        if len(next_members) > 1:
            raise ValueError(
                f"Visual axis '{axis_id}' branches at '{current.branch_id}'"
            )
        current = next_members[0] if next_members else None

    if len(ordered) != len(members):
        raise ValueError(f"Visual axis '{axis_id}' is disconnected or cyclic")
    return ordered


def _validate_continuation(
    axis_id: str,
    previous: BranchData,
    current: BranchData,
) -> None:
    attach_frac = float(current.definition.get("attach_frac", 1.0))
    tilt = float(current.definition.get("tilt", 0.0))
    rot = float(current.definition.get("rot", 0.0))
    previous_rotation = Gf.Rotation(
        Gf.Quatd(previous.link_orientations[-1])
    )
    previous_direction = Gf.Vec3d(
        previous_rotation.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
    ).GetNormalized()
    expected_tip = (
        previous.link_bases[-1]
        + previous_direction * previous.link_lengths[-1]
    )
    position_error = (current.start - expected_tip).GetLength()
    axis_alignment = Gf.Dot(previous.axis, current.axis)
    explicit = previous.explicit_link_poses or current.explicit_link_poses

    problems = []
    if current.parent_id != previous.branch_id:
        problems.append(f"parent is '{current.parent_id}', expected '{previous.branch_id}'")
    if current.parent_link_index != previous.n_links - 1:
        problems.append("attachment is not on the parent's final link")
    if not math.isclose(attach_frac, 1.0, abs_tol=_CHAIN_TOLERANCE):
        problems.append(f"attach_frac is {attach_frac}, expected 1.0")
    if not explicit and not math.isclose(tilt, 0.0, abs_tol=_CHAIN_TOLERANCE):
        problems.append(f"tilt is {tilt}, expected 0.0")
    if not explicit and not math.isclose(rot, 0.0, abs_tol=_CHAIN_TOLERANCE):
        problems.append(f"rot is {rot}, expected 0.0")
    if position_error > _CHAIN_TOLERANCE:
        problems.append(f"rest-pose tip gap is {position_error:.6g}m")
    if not explicit and axis_alignment < 1.0 - _CHAIN_TOLERANCE:
        problems.append(f"axes are not aligned (dot={axis_alignment:.6g})")
    if problems:
        raise ValueError(
            f"Visual axis '{axis_id}' cannot join '{previous.branch_id}' to "
            f"'{current.branch_id}': " + "; ".join(problems)
        )


def _member_segments(member: BranchData, start_arc: float) -> List[VisualSegment]:
    raw_segments = member.definition.get("visual_segments")
    if raw_segments is None:
        return [VisualSegment(
            source_id=member.branch_id,
            start_arc=start_arc,
            length=member.total_length,
            radius=member.radius,
        )]
    if not isinstance(raw_segments, list) or not raw_segments:
        raise ValueError(
            f"Branch '{member.branch_id}' visual_segments must be a non-empty list"
        )

    result = []
    cursor = start_arc
    for index, raw in enumerate(raw_segments):
        try:
            source_id = str(raw["source_id"])
            length = scaled(float(raw["length"]))
            radius = scaled(float(raw["radius"]))

            raw_end_radius = raw.get("end_radius")

            end_radius = (
                scaled(float(raw_end_radius))
                if raw_end_radius is not None
                else None
            )

        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                f"Branch '{member.branch_id}' has invalid visual segment {index}"
            ) from exc

        # Validate both the normal radius and the optional distal radius.
        if (
            length <= 0.0
            or radius <= 0.0
            or (
                end_radius is not None
                and end_radius <= 0.0
            )
        ):
            raise ValueError(
                f"Branch '{member.branch_id}' visual segment {index} must have "
                "positive length and radii"
            )

        result.append(
            VisualSegment(
                source_id=source_id,
                start_arc=cursor,
                length=length,
                radius=radius,
                end_radius=end_radius,
            )
        )

        cursor += length

    visual_length = cursor - start_arc
    if not math.isclose(
        visual_length,
        member.total_length,
        rel_tol=1e-8,
        abs_tol=_CHAIN_TOLERANCE,
    ):
        raise ValueError(
            f"Branch '{member.branch_id}' visual_segments total {visual_length:.6g}m "
            f"does not match physical length {member.total_length:.6g}m"
        )
    return result


def build_visual_axes(
    branches: Iterable[BranchData],
    visual_parent_path: str = "/World/PlantVisual",
) -> List[VisualAxisData]:
    """Group resolved branches into validated, continuous visual axes."""
    resolved = list(branches)
    by_id = {branch.branch_id: branch for branch in resolved}
    grouped = defaultdict(list)
    axis_order = []
    for branch in resolved:
        raw_axis_id = branch.definition.get("visual_axis_id", branch.branch_id)
        if not isinstance(raw_axis_id, str) or not raw_axis_id.strip():
            raise ValueError(
                f"Branch '{branch.branch_id}' visual_axis_id must be a non-empty string"
            )
        axis_id = raw_axis_id
        if axis_id not in grouped:
            axis_order.append(axis_id)
        grouped[axis_id].append(branch)

    axes = []
    used_paths = {}
    for axis_id in axis_order:
        members = _ordered_members(axis_id, grouped[axis_id])
        for previous, current in zip(members, members[1:]):
            _validate_continuation(axis_id, previous, current)

        member_offsets = {}
        member_lengths = {}
        visual_segments = []
        link_paths = []
        link_bases = []
        link_orientations = []
        bone_starts = []
        bone_lengths = []
        cursor = 0.0
        for member in members:
            member_offsets[member.branch_id] = cursor
            member_segments = _member_segments(member, cursor)
            visual_segments.extend(member_segments)
            member_length = sum(segment.length for segment in member_segments)
            member_lengths[member.branch_id] = member_length
            for link_index, (path, base) in enumerate(
                zip(member.link_paths, member.link_bases)
            ):
                link_paths.append(path)
                link_bases.append(base)
                link_orientations.append(member.link_orientations[link_index])
                bone_starts.append(cursor)
                bone_lengths.append(member.link_lengths[link_index])
                cursor += member.link_lengths[link_index]

        safe_id = _path_component(axis_id)
        previous_axis = used_paths.get(safe_id)
        if previous_axis is not None and previous_axis != axis_id:
            raise ValueError(
                f"Visual axes '{previous_axis}' and '{axis_id}' map to the same USD path"
            )
        used_paths[safe_id] = axis_id
        visual_root = f"{visual_parent_path}/{safe_id}"
        skel_root = f"{visual_root}/SkelRoot"
        root_member = members[0]
        parent = by_id.get(root_member.parent_id)
        axes.append(VisualAxisData(
            axis_id=axis_id,
            members=members,
            member_offsets=member_offsets,
            member_lengths=member_lengths,
            visual_segments=visual_segments,
            link_paths=link_paths,
            link_bases=link_bases,
            link_orientations=link_orientations,
            bone_starts=bone_starts,
            bone_lengths=bone_lengths,
            start=root_member.start,
            axis=root_member.axis,
            orientation=root_member.orientation,
            total_length=cursor,
            visual_root_path=visual_root,
            skel_root_path=skel_root,
            skeleton_path=f"{skel_root}/Skeleton",
            animation_path=f"{skel_root}/SkelAnim",
            mesh_path=f"{skel_root}/BranchMesh",
            parent_radius=parent.radius if parent is not None else None,
            attachment_arcs=[],
        ))
    return axes
