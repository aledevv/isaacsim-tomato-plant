"""Leaflets: a short organic stalk (petiolule) plus a blade at its tip.

Leaflets are visual only. They are parented to the rigid body of the petiole
or leaf rachis they grow from, and their mass is added to that body so the
leaf still bends under its own weight.

Blades are baked 3D leaflets referenced from ``assets/leaves`` (``baked``, see
``leaf_shapes/baked.py``) or the procedural blade in
``core/skinning/leaf_blade.py`` (``simple``).
"""

from __future__ import annotations

import math
from types import SimpleNamespace
from typing import Any, Iterable

from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from ..core import tree_config
from ..core.skinning import VisualAxisData, VisualProfile, VisualSegment
from ..core.skinning.adapter import _quat_from_column_rotation
from ..core.skinning.leaf_blade import (
    LEAF_ARCH_LIFT_FRACTION,
    LEAF_HALF_WIDTH_FRACTION,
    LEAF_LENGTH_FRACTION,
    LEAF_LONGITUDINAL_FOLD_FRACTION,
    LEAF_TIP_SAG_FRACTION,
    author_leaf_blade,
)
from ..core.skinning.visual_segmented import author_segmented_visual_axis
from ..leaf_shapes.baked import reference_baked_leaf
from ..leaf_shapes.seeds import leaflet_seed

# The skeleton's leaflet stalks are thicker than real petiolules; the visual
# tube uses this fraction of the skeleton radius (no collider depends on it).
PETIOLULE_VISUAL_RADIUS_SCALE = 0.45
# The petiolule tapers to this fraction of its base radius.
PETIOLULE_TIP_RADIUS_FRACTION = 0.85
# Blade length is the petiolule length times LEAF_LENGTH_FRACTION, clamped.
BLADE_LENGTH_MIN_M = 0.04
BLADE_LENGTH_MAX_M = 0.09
# Where the blade weight acts, as a fraction of the blade length.
BLADE_CENTROID_FRACTION = 0.4


class LeafVisualError(ValueError):
    pass


def _custom(prim, name: str, value_type, value) -> None:
    prim.CreateAttribute(name, value_type, custom=True).Set(value)


def blade_length(petiolule_length: float) -> float:
    return min(BLADE_LENGTH_MAX_M, max(BLADE_LENGTH_MIN_M, petiolule_length * LEAF_LENGTH_FRACTION))


def baked_blade_frame(base, forward, *, droop_deg: float, roll_deg: float) -> Gf.Matrix4d:
    """World frame of a baked blade: rows = side (asset X), forward (Y), upper side (Z), base.

    The blade starts along the petiolule with its upper side facing the sky,
    then tips down by ``droop_deg`` and rolls by ``roll_deg`` around its axis.
    """

    forward = Gf.Vec3d(forward).GetNormalized()
    up = Gf.Vec3d(0.0, 0.0, 1.0)
    upper = up - forward * Gf.Dot(up, forward)
    if upper.GetLength() < 1e-6:  # petiolule pointing straight up or down
        upper = Gf.Vec3d(0.0, 1.0, 0.0) - forward * forward[1]
    upper = upper.GetNormalized()
    droop, roll = math.radians(droop_deg), math.radians(roll_deg)
    forward, upper = (
        forward * math.cos(droop) - upper * math.sin(droop),
        forward * math.sin(droop) + upper * math.cos(droop),
    )
    side = Gf.Cross(forward, upper)
    side, upper = (
        side * math.cos(roll) + upper * math.sin(roll),
        upper * math.cos(roll) - side * math.sin(roll),
    )
    return Gf.Matrix4d(*side, 0.0, *forward, 0.0, *upper, 0.0, *base, 1.0)


def blade_tilt(leaves: dict[str, Any], seed: int, plant_id, leaflet_id: str) -> tuple[float, float]:
    """Droop and roll angles (degrees) for one leaflet, jittered by its own seed."""

    jitter = leaflet_seed(seed, plant_id, leaflet_id, "terminal")
    u = 2.0 * (jitter & 0xFFFF) / 0xFFFF - 1.0
    v = 2.0 * ((jitter >> 16) & 0xFFFF) / 0xFFFF - 1.0
    return (
        float(leaves["droop_deg"]) + float(leaves["droop_jitter_deg"]) * u,
        float(leaves["roll_jitter_deg"]) * v,
    )


def author_leaf_visuals(
    stage,
    records: Iterable[dict[str, Any]],
    visual_profile: dict[str, Any],
    leaf_shapes=None,
    *,
    baked=None,
    leaves: dict[str, Any] | None = None,
    plant_id=None,
) -> list[dict[str, Any]]:
    """Author every leaflet record under its host rigid body.

    ``baked`` (from ``leaf_shapes.baked.prepare_baked_leaves``) swaps the blade
    for a reference to a baked 3D leaflet, tilted by the ``leaves`` settings of
    config/visuals.yaml; otherwise the blade is generated from ``leaf_shapes``.
    """

    records = list(records)
    if not records:
        return []
    scale = tree_config.GLOBAL_SCALE
    body_by_node = {}
    for prim in stage.Traverse():
        if prim.GetAttribute("autotom:entityKind").Get() != "physical_link":
            continue
        node_id = prim.GetAttribute("autotom:canonicalPrimitiveId").Get()
        if node_id is not None:
            body_by_node[str(node_id)] = str(prim.GetPath())
    profile = VisualProfile(
        radial_segments=int(visual_profile["radial_segments"]),
        axial_spacing_m=float(visual_profile["axial_spacing_m"]),
        radius_transition_samples=int(visual_profile["radius_transition_samples"]),
    )

    authored = []
    for record in records:
        host_path = body_by_node.get(record["host_axis_id"])
        if host_path is None:
            raise LeafVisualError(f"leaflet {record['axis_id']} has no host body {record['host_axis_id']}")
        host_prim = stage.GetPrimAtPath(host_path)
        world_to_host = UsdGeom.Xformable(host_prim).ComputeLocalToWorldTransform(0).GetInverse()
        frame = record["rest_frame"]
        start = Gf.Vec3d(*(float(frame[row][3]) * scale for row in range(3)))
        head = Gf.Vec3d(*(float(frame[row][2]) for row in range(3))).GetNormalized()
        length = float(record["length"]) * scale
        radius = float(record["radius"]) * scale * PETIOLULE_VISUAL_RADIUS_SCALE

        root_path = f"{host_path}/RigidLeafVisuals/{record['id']}"
        root_xform = UsdGeom.Xform.Define(stage, root_path)
        root = root_xform.GetPrim()
        orientation = _quat_from_column_rotation(frame)
        desired_world = Gf.Matrix4d(1.0)
        desired_world.SetTransform(Gf.Rotation(Gf.Quatd(orientation)), start)
        root_xform.AddTransformOp().Set(desired_world * world_to_host)
        actual_world = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(0)
        frame_error = max(
            abs(float(actual_world[row][column] - desired_world[row][column]))
            for row in range(4)
            for column in range(4)
        )
        if frame_error > 1e-8:
            raise LeafVisualError(f"leaflet {record['axis_id']} frame error {frame_error:.3g}")
        _custom(root, "autotom:entityKind", Sdf.ValueTypeNames.String, "rigid_leaf_visual")
        _custom(root, "autotom:attachmentMode", Sdf.ValueTypeNames.String, "rigid_visual")
        _custom(root, "autotom:canonicalOrganId", Sdf.ValueTypeNames.String, record["organ_id"])
        _custom(root, "autotom:canonicalPrimitiveId", Sdf.ValueTypeNames.String, record["axis_id"])
        _custom(root, "autotom:skeletonNodeId", Sdf.ValueTypeNames.String, record["node_id"])
        _custom(root, "autotom:role", Sdf.ValueTypeNames.String, record["role"])
        _custom(root, "autotom:hostPrimitiveId", Sdf.ValueTypeNames.String, record["host_axis_id"])

        member = SimpleNamespace(
            definition={"id": record["id"], "kind": "petiolule"},
            spec=SimpleNamespace(visual=profile),
            centered_terminal=False,
            centered_terminal_host=False,
        )
        visual_axis = VisualAxisData(
            axis_id=record["axis_id"],
            members=[member],
            member_offsets={record["id"]: 0.0},
            member_lengths={record["id"]: length},
            visual_segments=[
                VisualSegment(
                    source_id=record["axis_id"],
                    start_arc=0.0,
                    length=length,
                    radius=radius,
                    end_radius=radius * PETIOLULE_TIP_RADIUS_FRACTION,
                )
            ],
            link_paths=[root_path],
            link_bases=[start],
            link_orientations=[orientation],
            bone_starts=[0.0],
            bone_lengths=[length],
            start=start,
            axis=head,
            orientation=orientation,
            total_length=length,
            visual_root_path=root_path,
            skel_root_path=f"{root_path}/UnusedSkelRoot",
            skeleton_path=f"{root_path}/UnusedSkeleton",
            animation_path=f"{root_path}/UnusedAnimation",
            mesh_path=f"{root_path}/UnusedMesh",
            parent_radius=float(host_prim.GetAttribute("autotom:visualRadius").Get()),
            attachment_arcs=[],
        )
        visual_stats = author_segmented_visual_axis(stage, visual_axis)

        blade = blade_length(length)
        if baked is not None:
            droop, roll = blade_tilt(leaves, int(leaves["seed"]), plant_id, record["leaflet_id"])
            frame = baked_blade_frame(start + head * length, head, droop_deg=droop, roll_deg=roll)
            leaf = baked.for_leaf(record["leaflet_id"])
            blade_prim = reference_baked_leaf(
                stage, f"{root_path}/LeafBlade", baked.leaf_set, leaf, blade, frame * actual_world.GetInverse()
            )
            _custom(blade_prim, "autotom:leafletRole", Sdf.ValueTypeNames.String, record["leaflet_role"])
            _custom(blade_prim, "autotom:leafShapeStructuralId", Sdf.ValueTypeNames.String, record["leaflet_id"])
            _custom(blade_prim, "autotom:leafDroopDeg", Sdf.ValueTypeNames.Double, droop)
        else:
            author_leaf_blade(
                stage,
                f"{root_path}/LeafBlade",
                start + head * length,
                head,
                length=blade,
                half_width=blade * LEAF_HALF_WIDTH_FRACTION,
                fold_depth=blade * LEAF_LONGITUDINAL_FOLD_FRACTION,
                arch_lift=blade * LEAF_ARCH_LIFT_FRACTION,
                tip_sag=blade * LEAF_TIP_SAG_FRACTION,
                color=tree_config.PlantColors.LEAF_BLADE,
                # Blade vertices are expressed in the petiolule frame.
                world_to_link=actual_world.GetInverse(),
                shape=(leaf_shapes.for_leaf(record["leaflet_id"]) if leaf_shapes is not None else None),
            )
        added_mass = float(record.get("aggregated_mass_kg", 0.0))
        if added_mass > 0.0:
            _add_point_mass(host_prim, world_to_host.Transform(start + head * (length + BLADE_CENTROID_FRACTION * blade)), added_mass)
        _custom(root, "autotom:aggregatedMassKg", Sdf.ValueTypeNames.Double, added_mass)
        _custom(root, "autotom:massSource", Sdf.ValueTypeNames.String, str(record.get("mass_source", "none")))
        authored.append(
            {
                **record,
                "host_body_path": host_path,
                "root_path": root_path,
                "blade_length": blade,
                "visual_stats": visual_stats,
            }
        )
    return authored


def _add_point_mass(body_prim, point_local, mass: float) -> None:
    """Add ``mass`` at ``point_local`` to the body, updating its centre of mass."""

    mass_api = UsdPhysics.MassAPI(body_prim)
    previous_mass = float(mass_api.GetMassAttr().Get())
    previous_com = Gf.Vec3d(*mass_api.GetCenterOfMassAttr().Get())
    combined = previous_mass + mass
    mass_api.GetMassAttr().Set(combined)
    mass_api.GetCenterOfMassAttr().Set(Gf.Vec3f(*((previous_com * previous_mass + point_local * mass) / combined)))
    current = float(body_prim.GetAttribute("autotom:aggregatedLeafVisualMassKg").Get() or 0.0)
    _custom(body_prim, "autotom:aggregatedLeafVisualMassKg", Sdf.ValueTypeNames.Double, current + mass)
