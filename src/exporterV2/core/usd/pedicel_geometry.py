"""Procedural gravity-curved visual meshes for tomato pedicels."""

import hashlib
import math
from pxr import Gf, UsdGeom, Vt
from ..mesh_geometry import build_open_tube_topology, smooth_vertex_normals
from ..tree_config import PlantColors


# Cubic control-arm fractions of physical pedicel length
ROOT_TANGENT_ARM_FRACTION = 0.34
TIP_TANGENT_ARM_FRACTION = 0.42
SIDE_VARIATION_FRACTION = 0.025

RADIAL_SEGMENTS = 14
CURVE_SAMPLES = 15

ROOT_RADIUS_SCALE_RANGE = (1.15, 1.28)
MID_RADIUS_SCALE_RANGE = (0.82, 0.94)
TIP_RADIUS_SCALE_RANGE = (0.96, 1.08)


def _stable_unit(key: str, salt: str) -> float:
    digest = hashlib.blake2b(
        f"{key}|{salt}|truss-test-6a".encode("utf-8"),
        digest_size=8,
    ).digest()
    return int.from_bytes(digest, "big") / float((1 << 64) - 1)


def _stable_range(key: str, salt: str, low: float, high: float) -> float:
    return low + (high - low) * _stable_unit(key, salt)


def _length(v: Gf.Vec3d) -> float:
    return math.sqrt(float(Gf.Dot(v, v)))


def _normalized(v: Gf.Vec3d) -> Gf.Vec3d:
    v = Gf.Vec3d(v)
    length = _length(v)
    if length <= 1e-12:
        return v
    return v / length


def _cubic_point(p0, p1, p2, p3, t: float):
    u = 1.0 - t
    return (
        p0 * (u ** 3)
        + p1 * (3.0 * u * u * t)
        + p2 * (3.0 * u * t * t)
        + p3 * (t ** 3)
    )


def _cubic_tangent(p0, p1, p2, p3, t: float):
    u = 1.0 - t
    tangent = (
        (p1 - p0) * (3.0 * u * u)
        + (p2 - p1) * (6.0 * u * t)
        + (p3 - p2) * (3.0 * t * t)
    )
    return _normalized(tangent)


def _stable_side_offset(height: float, branch_id: str) -> Gf.Vec3d:
    phase = 2.0 * math.pi * _stable_unit(branch_id, "gravity_elbow_phase")
    direction = Gf.Vec3d(math.cos(phase), math.sin(phase), 0.0)
    amount = height * SIDE_VARIATION_FRACTION * (
        2.0 * _stable_unit(branch_id, "gravity_elbow_side") - 1.0
    )
    return direction * amount


def sample_gravity_elbow(height: float, branch_id: str, gravity_local: Gf.Vec3d):
    """Sample a root-diagonal -> world-down terminal cubic in pedicel local space."""
    gravity_local = _normalized(gravity_local)
    root_tangent = Gf.Vec3d(0.0, 0.0, 1.0)
    side_offset = _stable_side_offset(height, branch_id)

    p0 = Gf.Vec3d(0.0, 0.0, 0.0)
    p3 = Gf.Vec3d(0.0, 0.0, height)

    p1 = p0 + root_tangent * (height * ROOT_TANGENT_ARM_FRACTION) + side_offset
    p2 = (
        p3
        - gravity_local * (height * TIP_TANGENT_ARM_FRACTION)
        + side_offset * 0.25
    )

    centers = []
    tangents = []
    for index in range(CURVE_SAMPLES):
        t = index / float(CURVE_SAMPLES - 1)
        centers.append(_cubic_point(p0, p1, p2, p3, t))
        tangents.append(_cubic_tangent(p0, p1, p2, p3, t))
    return centers, tangents


def _transport_frames(tangents):
    first = tangents[0]
    reference = Gf.Vec3d(0.0, 0.0, 1.0)
    if abs(float(Gf.Dot(first, reference))) > 0.90:
        reference = Gf.Vec3d(1.0, 0.0, 0.0)

    normal = _normalized(Gf.Cross(reference, first))
    binormal = _normalized(Gf.Cross(first, normal))
    normals = [normal]
    binormals = [binormal]
    previous_tangent = first
    previous_normal = normal

    for tangent in tangents[1:]:
        axis = Gf.Cross(previous_tangent, tangent)
        if _length(axis) > 1e-10:
            axis = _normalized(axis)
            cosine = max(-1.0, min(1.0, float(Gf.Dot(previous_tangent, tangent))))
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


def _radius_profile(base_radius: float, branch_id: str):
    root_scale = _stable_range(branch_id, "root_radius", *ROOT_RADIUS_SCALE_RANGE)
    mid_scale = _stable_range(branch_id, "mid_radius", *MID_RADIUS_SCALE_RANGE)
    tip_scale = _stable_range(branch_id, "tip_radius", *TIP_RADIUS_SCALE_RANGE)

    radii = []
    for index in range(CURVE_SAMPLES):
        t = index / float(CURVE_SAMPLES - 1)
        if t < 0.45:
            q = t / 0.45
            q = q * q * (3.0 - 2.0 * q)
            scale = root_scale + (mid_scale - root_scale) * q
        else:
            q = (t - 0.45) / 0.55
            q = q * q * (3.0 - 2.0 * q)
            scale = mid_scale + (tip_scale - mid_scale) * q
        radii.append(base_radius * scale)
    return radii


def _tube_mesh_data(centers, tangents, radii):
    normals, binormals = _transport_frames(tangents)
    points = []
    for center, normal, binormal, radius in zip(centers, normals, binormals, radii):
        for radial_index in range(RADIAL_SEGMENTS):
            theta = 2.0 * math.pi * radial_index / RADIAL_SEGMENTS
            point = center + radius * (
                normal * math.cos(theta) + binormal * math.sin(theta)
            )
            points.append(Gf.Vec3f(*point))

    counts, indices = build_open_tube_topology(len(centers), RADIAL_SEGMENTS)

    start_center = len(points)
    points.append(Gf.Vec3f(*centers[0]))
    end_center = len(points)
    points.append(Gf.Vec3f(*centers[-1]))
    end_row = (len(centers) - 1) * RADIAL_SEGMENTS
    for radial_index in range(RADIAL_SEGMENTS):
        nxt = (radial_index + 1) % RADIAL_SEGMENTS
        counts.extend((3, 3))
        indices.extend((start_center, nxt, radial_index))
        indices.extend((end_center, end_row + radial_index, end_row + nxt))

    return points, counts, indices


def create_gravity_elbow_mesh(
    stage,
    parent_link_path: str,
    centers,
    tangents,
    radius: float,
    branch_id: str,
):
    """Author a curved pedicel tube mesh at the specified parent link path."""
    radii = _radius_profile(radius, branch_id)
    points, counts, indices = _tube_mesh_data(centers, tangents, radii)

    mesh = UsdGeom.Mesh.Define(stage, f"{parent_link_path}/GravityElbowPedicelVisual")
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(indices))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateNormalsAttr().Set(
        Vt.Vec3fArray(smooth_vertex_normals(points, counts, indices))
    )
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateDisplayColorAttr().Set(
        Vt.Vec3fArray([Gf.Vec3f(*PlantColors.PEDICEL)])
    )
    return mesh
