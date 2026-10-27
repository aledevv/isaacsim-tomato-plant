"""Truss dressing on top of the organic branch meshes.

* truss rachis: the organic mesh gets the truss colour and material;
* both are drawn at ``trusses.rachis_thickness`` / ``pedicel_thickness`` of
  the skeleton radius (colliders keep the full radius);
* pedicel (``pedicels.gravity_curve: true``): the straight mesh is hidden and
  replaced by a curved "gravity elbow" tube (``core/usd/pedicel_geometry.py``)
  so the tomato hangs naturally.
"""

from __future__ import annotations

from typing import Any, Iterable

from pxr import Gf, Sdf, UsdGeom, UsdShade

from ..core.tree_config import PlantColors
from ..core.usd.materials import get_or_create_tomato_truss_material
from ..core.usd.pedicel_geometry import create_gravity_elbow_mesh, sample_gravity_elbow


class TrussVisualError(ValueError):
    pass


def author_truss_visuals(stage, branches: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    styles = {
        spec["canonical_axis_id"]: (branch["visual_style"], branch.get("visual_radius_scale", 1.0))
        for branch in branches
        if branch.get("visual_style")
        for spec in branch.get("link_specs", ())
    }
    if not styles:
        return []
    authored = []
    for prim in list(stage.Traverse()):
        if prim.GetAttribute("autotom:entityKind").Get() != "physical_link":
            continue
        node_id = prim.GetAttribute("autotom:canonicalPrimitiveId").Get()
        if str(node_id) not in styles:
            continue
        style, radius_scale = styles[str(node_id)]
        organic = next(
            (
                child
                for child in prim.GetChildren()
                if child.IsA(UsdGeom.Mesh) and child.GetName().startswith("OrganicVisual_")
            ),
            None,
        )
        if style == "historical_truss_rachis":
            if organic is None:
                raise TrussVisualError(f"truss link {prim.GetPath()} has no organic mesh")
            UsdGeom.Mesh(organic).CreateDisplayColorAttr().Set([PlantColors.TRUSS_RACHIS])
            UsdShade.MaterialBindingAPI.Apply(organic).Bind(get_or_create_tomato_truss_material(stage))
            visual_path = str(organic.GetPath())
        elif style == "historical_pedicel":
            if organic is not None:
                UsdGeom.Imageable(organic).MakeInvisible()
            length = float(prim.GetAttribute("autotom:sourceLength").Get())
            radius = float(prim.GetAttribute("autotom:visualRadius").Get()) * radius_scale
            rotation = Gf.Rotation(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractRotationQuat())
            gravity_local = rotation.GetInverse().TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
            centers, tangents = sample_gravity_elbow(length, str(node_id), gravity_local)
            visual = create_gravity_elbow_mesh(stage, str(prim.GetPath()), centers, tangents, radius, str(node_id))
            visual_path = str(visual.GetPath())
        else:
            raise TrussVisualError(f"unknown truss visual style {style!r}")
        prim.CreateAttribute("autotom:visualStyle", Sdf.ValueTypeNames.String, custom=True).Set(style)
        authored.append({"node_id": str(node_id), "body_path": str(prim.GetPath()), "visual_path": visual_path, "style": style})
    return sorted(authored, key=lambda item: item["node_id"])
