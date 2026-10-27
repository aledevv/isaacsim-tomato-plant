"""Metric continuous tube mesh and UsdSkel authoring for visual axes."""

import bisect
import math
from typing import Iterable, List

from pxr import Gf, Sdf, UsdGeom, UsdShade, UsdSkel, Vt

from ..mesh_geometry import build_open_tube_topology, smooth_vertex_normals
from ..tree_config import PlantColors
from ..usd.materials import get_or_create_tomato_stem_material
from .model import VisualAxisData
from .schema import (
    ANIMATION_REL,
    BRANCH_ID_ATTR,
    PHYSICS_LINKS_REL,
    SCHEMA_VERSION,
    SCHEMA_VERSION_ATTR,
    VISUAL_AXIS_ID_ATTR,
)


_ARC_PRECISION = 12
_PETIOLULE_ROOT_PARENT_FRACTION = 0.72
_PETIOLULE_ROOT_SHOULDER_AMPLITUDE = 0.06
# Thin axes (petiolules, truss stalks): the root flare covers only this
# fraction of the axis, so a thin stalk does not become a cone sized by its
# parent; truss stalks also flare to at most this many times their radius.
_THIN_AXIS_FLARE_LENGTH_FRACTION = 0.3
_TRUSS_ROOT_MAX_RADIUS_SCALE = 1.5


def _pose_matrix(position: Gf.Vec3d, orientation: Gf.Quatf) -> Gf.Matrix4d:
    matrix = Gf.Matrix4d(1.0)
    matrix.SetTransform(Gf.Rotation(Gf.Quatd(orientation)), position)
    return matrix


def _quatf_from_matrix(matrix: Gf.Matrix4d) -> Gf.Quatf:
    quat = matrix.ExtractRotationQuat()
    return Gf.Quatf(float(quat.GetReal()), Gf.Vec3f(*quat.GetImaginary()))


def link_rest_world(axis: VisualAxisData, link_index: int) -> Gf.Matrix4d:
    """Return the world-space rest pose of a given physics link."""
    matrix = Gf.Matrix4d(1.0)
    matrix.SetTransform(
        Gf.Rotation(Gf.Quatd(axis.link_orientations[link_index])),
        axis.link_bases[link_index],
    )
    return matrix


def author_plain_mesh(
    stage,
    path: str,
    points,
    face_counts,
    face_indices,
    color,
    *,
    vertex_colors=None,
    material=None,
) -> None:
    """Author a non-skinned mesh with display color and optional material."""
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr().Set(UsdGeom.Tokens.rightHanded)
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateNormalsAttr().Set(
        Vt.Vec3fArray(smooth_vertex_normals(points, face_counts, face_indices))
    )
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)

    if vertex_colors is None:
        mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*color)]))
    else:
        color_primvar = mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.vertex)
        color_primvar.Set(Vt.Vec3fArray(vertex_colors))

    if material is not None:
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def _decompose(transforms):
    translations = []
    rotations = []
    for matrix in transforms:
        translations.append(Gf.Vec3f(*matrix.ExtractTranslation()))
        rotations.append(_quatf_from_matrix(matrix))
    return translations, rotations


def _axis_color(axis: VisualAxisData):
    branch = axis.definition
    branch_id = " ".join(
        [branch["id"], *(segment.source_id for segment in axis.visual_segments)]
    ).lower()
    kind = branch.get("kind", "").lower()
    if kind in ("petiole", "rachis") or "petiole" in branch_id or "rachis" in branch_id:
        return PlantColors.PETIOLE
    if kind == "petiolule" or "petiolule" in branch_id:
        return PlantColors.PETIOLULE
    return PlantColors.STEM


def _is_thin_axis(axis: VisualAxisData) -> bool:
    return _is_petiolule_axis(axis) or axis.definition.get("physics_profile") == "truss"


def _is_petiolule_axis(axis: VisualAxisData) -> bool:
    """Return True for standalone petiolule axes that need a subtle root flare."""
    branch = axis.definition
    kind = branch.get("kind", "").lower()
    branch_id = branch.get("id", "").lower()
    return kind == "petiolule" or "petiolule" in branch_id


def _smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def _gaussian(value: float, center: float, sigma: float) -> float:
    delta = (value - center) / max(sigma, 1e-8)
    return math.exp(-0.5 * delta * delta)


def _canonical_arc(value: float, total_length: float) -> float:
    return round(max(0.0, min(total_length, value)), _ARC_PRECISION)


def _radius_transition_half_width(axis: VisualAxisData, previous, current) -> float:
    profile = axis.profile

    return min(
        profile.radius_transition_half_width_m,
        previous.length * profile.radius_transition_max_fraction,
        current.length * profile.radius_transition_max_fraction,
    )


def _segment_radius(segment, arc: float) -> float:
    """Evaluate the radius inside one visual segment."""
    if segment.end_radius is None:
        return segment.radius
    t = (arc - segment.start_arc) / max(segment.length, 1e-8)
    return segment.radius + (segment.end_radius - segment.radius) * _smoothstep(
        t
    )


def build_axis_sample_arcs(axis: VisualAxisData) -> List[float]:
    """Build optimizer-independent axial samples in world-space meters."""
    spacing = axis.profile.axial_spacing_m
    if spacing <= 0.0:
        raise ValueError(
            f"Visual axis '{axis.axis_id}' axial_spacing_m must be positive"
        )

    arcs = {0.0, _canonical_arc(axis.total_length, axis.total_length)}
    regular_count = int(math.floor(axis.total_length / spacing))
    arcs.update(
        _canonical_arc(index * spacing, axis.total_length)
        for index in range(1, regular_count + 1)
    )
    arcs.update(
        _canonical_arc(segment.end_arc, axis.total_length)
        for segment in axis.visual_segments[:-1]
    )

    for segment in axis.visual_segments:
        if segment.end_radius is None:
            continue
        taper_samples = 5
        for index in range(taper_samples):
            t = index / float(taper_samples - 1)
            sample_arc = segment.start_arc + segment.length * t
            arcs.add(_canonical_arc(sample_arc, axis.total_length))

    for previous, current in zip(
        axis.visual_segments, axis.visual_segments[1:]
    ):
        boundary = previous.end_arc
        half_width = _radius_transition_half_width(axis, previous, current)
        sample_count = max(3, axis.profile.radius_transition_samples)

        for index in range(sample_count):
            t = index / float(sample_count - 1)
            transition_arc = boundary - half_width + 2.0 * half_width * t
            arcs.add(
                _canonical_arc(transition_arc, axis.total_length)
            )
    arcs.update(
        _canonical_arc(arc, axis.total_length)
        for arc in axis.attachment_arcs
    )
    return sorted(arcs)


def _profile_radius(axis: VisualAxisData, arc: float) -> float:
    segments = axis.visual_segments
    current_segment = segments[-1]
    for segment in segments:
        if arc <= segment.end_arc + 1e-12:
            current_segment = segment
            break
    radius = _segment_radius(current_segment, arc)

    for previous, current in zip(segments, segments[1:]):
        boundary = previous.end_arc
        half_width = _radius_transition_half_width(axis, previous, current)
        if half_width <= 0.0:
            continue
        transition_start = boundary - half_width
        transition_end = boundary + half_width

        if transition_start <= arc <= transition_end:
            radius_before = _segment_radius(previous, transition_start)
            radius_after = _segment_radius(current, transition_end)
            blend = _smoothstep(
                (arc - transition_start)
                / max(transition_end - transition_start, 1e-8)
            )
            return radius_before + (radius_after - radius_before) * blend

    return radius


def _visual_radius(axis: VisualAxisData, arc: float) -> float:
    radius = _profile_radius(axis, arc)
    profile = axis.profile
    bulge_sigma = max(axis.total_length * profile.junction_bulge_sigma, 1e-8)
    for center in axis.attachment_arcs:
        radius *= 1.0 + profile.junction_bulge_amplitude * _gaussian(
            arc,
            center,
            bulge_sigma,
        )

    if axis.parent_radius is not None:
        # Small petiolules should not inherit the full branch-style root flare.
        # Keep their base only slightly thicker than the nominal shaft while
        # preserving the stronger swelling used for structural branches.
        if _is_petiolule_axis(axis):
            root_parent_fraction = _PETIOLULE_ROOT_PARENT_FRACTION
            shoulder_amplitude = _PETIOLULE_ROOT_SHOULDER_AMPLITUDE
        else:
            root_parent_fraction = profile.root_parent_fraction
            shoulder_amplitude = profile.root_shoulder_amplitude

        flare_length = min(axis.total_length, max(0.042, axis.parent_radius * 3.2))
        root_target_max = None
        if _is_thin_axis(axis):
            flare_length = axis.total_length * _THIN_AXIS_FLARE_LENGTH_FRACTION
            if not _is_petiolule_axis(axis):
                root_target_max = _profile_radius(axis, 0.0) * _TRUSS_ROOT_MAX_RADIUS_SCALE
        if arc <= flare_length:
            q = arc / max(flare_length, 1e-8)
            fade = 1.0 - _smoothstep(q)
            root_target = max(radius, axis.parent_radius * root_parent_fraction)
            if root_target_max is not None:
                root_target = max(radius, min(root_target, root_target_max))
            radius += (root_target - radius) * fade
            radius *= (
                1.0
                + shoulder_amplitude
                * _gaussian(q, 0.42, 0.22)
                * fade
            )
    return radius


def _skin_weights(axis: VisualAxisData, arc: float):
    profile = axis.profile
    for child in range(1, len(axis.bone_starts)):
        center = axis.bone_starts[child]
        half_width = min(
            profile.skin_blend_half_width_m,
            axis.bone_lengths[child - 1] * 0.25,
            axis.bone_lengths[child] * 0.25,
        )
        if half_width > 0.0 and center - half_width <= arc <= center + half_width:
            blend = (arc - (center - half_width)) / (2.0 * half_width)
            return child - 1, child, 1.0 - blend, blend

    bone = bisect.bisect_right(axis.bone_starts, arc) - 1
    bone = max(0, min(bone, len(axis.bone_starts) - 1))
    return bone, bone, 1.0, 0.0


def _link_index_for_arc(axis: VisualAxisData, arc: float) -> int:
    if arc >= axis.total_length - 1e-12:
        return len(axis.bone_starts) - 1
    index = bisect.bisect_right(axis.bone_starts, arc) - 1
    return max(0, min(index, len(axis.bone_starts) - 1))


def centerline_tangent(axis: VisualAxisData, arc: float) -> Gf.Vec3d:
    index = _link_index_for_arc(axis, arc)
    rotation = Gf.Rotation(Gf.Quatd(axis.link_orientations[index]))
    return Gf.Vec3d(
        rotation.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
    ).GetNormalized()


def centerline_point(axis: VisualAxisData, arc: float) -> Gf.Vec3d:
    index = _link_index_for_arc(axis, arc)
    local_arc = max(0.0, min(axis.bone_lengths[index], arc - axis.bone_starts[index]))
    return axis.link_bases[index] + centerline_tangent(axis, arc) * local_arc


def build_straight_centerline(axis: VisualAxisData, arcs: Iterable[float]):
    """Sample a straight legacy axis or an explicit piecewise-linear rest pose."""
    arc_list = list(arcs)
    return arc_list, [centerline_point(axis, arc) for arc in arc_list]


def build_parallel_transport_frames(axis: VisualAxisData, arcs):
    """Parallel-transport cross sections along the legacy or explicit centerline."""
    arc_list = list(arcs)
    if not arc_list:
        return [], []
    rotation = Gf.Rotation(Gf.Quatd(axis.orientation))
    normal = Gf.Vec3d(
        rotation.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
    ).GetNormalized()
    previous_tangent = centerline_tangent(axis, arc_list[0])
    normal = (normal - previous_tangent * Gf.Dot(normal, previous_tangent)).GetNormalized()
    normals = []
    binormals = []
    for arc in arc_list:
        tangent = centerline_tangent(axis, arc)
        if Gf.Dot(previous_tangent, tangent) < 1.0 - 1e-12:
            transport = Gf.Rotation(previous_tangent, tangent)
            normal = Gf.Vec3d(transport.TransformDir(normal))
        normal = normal - tangent * Gf.Dot(normal, tangent)
        if normal.GetLength() <= 1e-12:
            fallback = Gf.Vec3d(1.0, 0.0, 0.0)
            if abs(Gf.Dot(fallback, tangent)) > 0.95:
                fallback = Gf.Vec3d(0.0, 1.0, 0.0)
            normal = fallback - tangent * Gf.Dot(fallback, tangent)
        normal.Normalize()
        binormal = Gf.Cross(tangent, normal).GetNormalized()
        normals.append(Gf.Vec3d(normal))
        binormals.append(Gf.Vec3d(binormal))
        previous_tangent = tangent
    return normals, binormals


def build_axis_tube_data(axis: VisualAxisData):
    """Return mesh topology and skin weights for one continuous visual axis."""
    radial_segments = axis.profile.radial_segments
    if radial_segments < 3:
        raise ValueError(
            f"Visual axis '{axis.axis_id}' radial_segments must be at least 3"
        )
    arcs = build_axis_sample_arcs(axis)
    arcs, centers = build_straight_centerline(axis, arcs)
    normals, binormals = build_parallel_transport_frames(axis, arcs)
    points = []
    joint_indices = []
    joint_weights = []

    for arc, center, normal, binormal in zip(arcs, centers, normals, binormals):
        radius = _visual_radius(axis, arc)
        bone0, bone1, weight0, weight1 = _skin_weights(axis, arc)
        for radial in range(radial_segments):
            theta = 2.0 * math.pi * radial / radial_segments
            point = center + radius * (
                normal * math.cos(theta) + binormal * math.sin(theta)
            )
            points.append(Gf.Vec3f(*point))
            joint_indices.extend((bone0, bone1))
            joint_weights.extend((weight0, weight1))

    face_counts, face_indices = build_open_tube_topology(
        len(arcs), radial_segments
    )
    return points, face_counts, face_indices, joint_indices, joint_weights


def _joint_names(count: int):
    names = ["Bone0"]
    for index in range(1, count):
        names.append(f"{names[-1]}/Bone{index}")
    return names


def author_visual_axis(stage, axis: VisualAxisData) -> None:
    """Author one smooth tube, skeleton, animation, and discovery relations."""
    UsdGeom.Xform.Define(stage, axis.visual_root_path)
    skel_root = UsdSkel.Root.Define(stage, axis.skel_root_path)
    skeleton = UsdSkel.Skeleton.Define(stage, axis.skeleton_path)
    animation = UsdSkel.Animation.Define(stage, axis.animation_path)

    names = _joint_names(len(axis.link_paths))
    bind = [
        _pose_matrix(base, orientation)
        for base, orientation in zip(axis.link_bases, axis.link_orientations)
    ]
    rest = [Gf.Matrix4d(bind[0])]
    for index in range(1, len(bind)):
        rest.append(bind[index] * bind[index - 1].GetInverse())
    translations, rotations = _decompose(rest)

    skeleton.CreateJointsAttr().Set(Vt.TokenArray(names))
    skeleton.CreateBindTransformsAttr().Set(Vt.Matrix4dArray(bind))
    skeleton.CreateRestTransformsAttr().Set(Vt.Matrix4dArray(rest))
    animation.CreateJointsAttr().Set(Vt.TokenArray(names))
    animation.CreateTranslationsAttr().Set(Vt.Vec3fArray(translations))
    animation.CreateRotationsAttr().Set(Vt.QuatfArray(rotations))
    animation.CreateScalesAttr().Set(Vt.Vec3hArray([
        Gf.Vec3h(1.0, 1.0, 1.0) for _ in axis.link_paths
    ]))
    UsdSkel.BindingAPI.Apply(skeleton.GetPrim()).CreateAnimationSourceRel().SetTargets([
        animation.GetPrim().GetPath()
    ])

    mesh = UsdGeom.Mesh.Define(stage, axis.mesh_path)
    points, face_counts, face_indices, joint_indices, joint_weights = (
        build_axis_tube_data(axis)
    )
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr().Set(UsdGeom.Tokens.rightHanded)
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(*_axis_color(axis))
    ]))
    stem_material = get_or_create_tomato_stem_material(stage)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(stem_material)

    binding = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
    binding.CreateSkeletonRel().SetTargets([skeleton.GetPrim().GetPath()])
    binding.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1.0))
    index_primvar = binding.CreateJointIndicesPrimvar(constant=False, elementSize=2)
    index_primvar.SetInterpolation(UsdGeom.Tokens.vertex)
    index_primvar.Set(Vt.IntArray(joint_indices))
    weight_primvar = binding.CreateJointWeightsPrimvar(constant=False, elementSize=2)
    weight_primvar.SetInterpolation(UsdGeom.Tokens.vertex)
    weight_primvar.Set(Vt.FloatArray(joint_weights))

    root_prim = skel_root.GetPrim()
    root_prim.CreateRelationship(PHYSICS_LINKS_REL, custom=True).SetTargets([
        Sdf.Path(path) for path in axis.link_paths
    ])
    root_prim.CreateRelationship(ANIMATION_REL, custom=True).SetTargets([
        animation.GetPrim().GetPath()
    ])
    root_prim.CreateAttribute(
        VISUAL_AXIS_ID_ATTR,
        Sdf.ValueTypeNames.String,
        custom=True,
    ).Set(axis.axis_id)
    root_prim.CreateAttribute(
        BRANCH_ID_ATTR,
        Sdf.ValueTypeNames.String,
        custom=True,
    ).Set(axis.members[0].branch_id)
    root_prim.CreateAttribute(
        SCHEMA_VERSION_ATTR,
        Sdf.ValueTypeNames.Int,
        custom=True,
    ).Set(SCHEMA_VERSION)
