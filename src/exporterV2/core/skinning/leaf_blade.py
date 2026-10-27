"""Realistic 3D leaf blade generation for vegetative structures.

This implements the Merlice cultivar mathematical leaf width profile (Coussement et al. 2017)
and applies longitudinal midrib folding and static gravity sag.
"""

import math
import json
from typing import Iterable
from pxr import Gf, Sdf
from ..tree_config import PlantColors
from ..usd.materials import get_or_create_tomato_leaf_material
from .mesh import (
    _is_petiolule_axis,
    author_plain_mesh,
    link_rest_world,
)


LEAF_STATIONS = 18
LEAF_LENGTH_FRACTION = 5.35
LEAF_HALF_WIDTH_FRACTION = 0.33

LEAF_LONGITUDINAL_FOLD_FRACTION = 0.064
LEAF_FOLD_EXPONENT = 0.78
LEAF_ARCH_LIFT_FRACTION = 0.08
LEAF_TIP_SAG_FRACTION = 0.133
LEAF_TIP_SAG_EXPONENT = 1.85


def _normalized(vector: Gf.Vec3d) -> Gf.Vec3d:
    vector = Gf.Vec3d(vector)
    if vector.GetLength() <= 1e-10:
        raise ValueError("Cannot normalize a zero-length vector")
    vector.Normalize()
    return vector


def _merlice_leaflet_width(t: float) -> float:
    """Normalized leaflet width based on Coussement et al. (2017) Merlice model."""
    pos_norm = 1.0 - t
    max_w = 0.6
    k1 = 2.0
    k2 = 2.2

    if pos_norm <= max_w:
        return 1.0 - ((max_w - pos_norm) / max_w) ** k1
    else:
        return 1.0 - ((pos_norm - max_w) / (1.0 - max_w)) ** k2


def author_leaf_blade(
    stage,
    path: str,
    root: Gf.Vec3d,
    forward: Gf.Vec3d,
    *,
    length: float,
    half_width: float,
    fold_depth: float,
    arch_lift: float,
    tip_sag: float,
    color: tuple,
    world_to_link: Gf.Matrix4d,
    shape: dict | None = None,
) -> None:
    """2D blade with longitudinal midrib fold and one gentle gravity arch."""
    forward = _normalized(forward)
    world_up = Gf.Vec3d(0.0, 0.0, 1.0)
    side = Gf.Cross(world_up, forward)
    if side.GetLength() <= 1e-8:
        side = Gf.Cross(Gf.Vec3d(0.0, 1.0, 0.0), forward)
    side = _normalized(side)
    sheet_normal = _normalized(Gf.Cross(forward, side))

    points = []
    for index in range(LEAF_STATIONS):
        t = index / float(LEAF_STATIONS - 1)
        width_profile = _merlice_leaflet_width(t)
        width = half_width * width_profile

        arch_offset = arch_lift * (4.0 * t * (1.0 - t))
        gravity_offset = tip_sag * t**LEAF_TIP_SAG_EXPONENT
        center = root + forward * (length * t) + world_up * (arch_offset - gravity_offset)

        edge_drop = fold_depth * math.sin(math.pi * t) ** LEAF_FOLD_EXPONENT

        p1 = center + side * width - sheet_normal * edge_drop
        p2 = center
        p3 = center - side * width - sheet_normal * edge_drop

        points.extend((
            Gf.Vec3f(*world_to_link.Transform(p1)),
            Gf.Vec3f(*world_to_link.Transform(p2)),
            Gf.Vec3f(*world_to_link.Transform(p3)),
        ))

    counts, indices = [], []
    for station in range(LEAF_STATIONS - 1):
        a, b = station * 3, (station + 1) * 3
        counts.extend((3, 3, 3, 3))
        indices.extend((
            a, a + 1, b + 1,
            a, b + 1, b,
            a + 1, a + 2, b + 2,
            a + 1, b + 2, b + 1
        ))

    material = get_or_create_tomato_leaf_material(stage)
    author_plain_mesh(
        stage,
        path,
        points,
        counts,
        indices,
        color,
        material=material,
    )
    if shape is not None:
        _author_shape_metadata(stage, path, shape, world_to_link.Transform(root),
                               world_to_link.Transform(root + forward * length - world_up * tip_sag))


def _author_shape_metadata(stage, path, shape, base, tip):
    prim = stage.GetPrimAtPath(path)
    values = {
        "leafShapeBackend": (Sdf.ValueTypeNames.String, shape["backend"]),
        "leafletRole": (Sdf.ValueTypeNames.String, shape["role"]),
        "leafShapeSeed": (Sdf.ValueTypeNames.Int64, shape["seed"]),
        "leafShapeStructuralId": (Sdf.ValueTypeNames.String, shape["id"]),
        "leafShapeProvider": (Sdf.ValueTypeNames.String, shape.get("provenance", {}).get("provider", "merlice")),
        "leafShapeProvenance": (Sdf.ValueTypeNames.String, json.dumps(shape.get("provenance", {}), sort_keys=True)),
        "leafShapeBase": (Sdf.ValueTypeNames.Double3, Gf.Vec3d(base)),
        "leafShapeTip": (Sdf.ValueTypeNames.Double3, Gf.Vec3d(tip)),
    }
    for key, (kind, value) in values.items():
        prim.CreateAttribute(f"autotom:{key}", kind, custom=True).Set(value)


def author_petiolule_leaf_blades(stage, visual_axes: Iterable, leaf_shapes=None) -> int:
    """Find all petiolules and author a realistic leaf blade at their tip."""
    count = 0
    for axis in visual_axes:
        if not _is_petiolule_axis(axis):
            continue

        root = axis.start + axis.axis * axis.total_length
        forward = axis.axis

        petiolule_length = axis.total_length
        leaf_length = min(0.09, max(0.04, petiolule_length * LEAF_LENGTH_FRACTION))

        half_width = leaf_length * LEAF_HALF_WIDTH_FRACTION
        fold_depth = leaf_length * LEAF_LONGITUDINAL_FOLD_FRACTION
        arch_lift = leaf_length * LEAF_ARCH_LIFT_FRACTION
        tip_sag = leaf_length * LEAF_TIP_SAG_FRACTION

        world_to_link = link_rest_world(axis, -1).GetInverse()
        author_leaf_blade(
            stage,
            f"{axis.link_paths[-1]}/LeafBlade",
            root,
            forward,
            length=leaf_length,
            half_width=half_width,
            fold_depth=fold_depth,
            arch_lift=arch_lift,
            tip_sag=tip_sag,
            color=PlantColors.LEAF_BLADE,
            world_to_link=world_to_link,
            shape=(leaf_shapes.for_leaf(axis.definition["leaflet_id"]) if leaf_shapes is not None else None),
        )
        count += 1

    return count
