"""Stem colour: one green for every stem-like mesh (``stems.color``).

Stems, side shoots, petioles, rachides, petiolules, truss rachides, pedicels
and the decorative fork twigs all get the same colour and the shared stem
material, so the plant reads as one organism. Leaves, calyces and tomatoes
keep their own materials.
"""

from __future__ import annotations

from pxr import Gf, Sdf, UsdGeom, UsdShade

from ..core.usd.materials import (
    TOMATO_STEM_MATERIAL_PATH,
    TOMATO_TRUSS_MATERIAL_PATH,
    get_or_create_tomato_stem_material,
)

STEM_ROUGHNESS = 0.6
_STEM_MESH_NAMES = ("GravityElbowPedicelVisual", "TerminalForkYoungShoot")


def _is_stem_mesh(prim) -> bool:
    if prim.GetName().startswith("OrganicVisual_") or prim.GetName() in _STEM_MESH_NAMES:
        return True
    bound = UsdShade.MaterialBindingAPI(prim).GetDirectBindingRel().GetTargets()
    return any(str(path) in (TOMATO_STEM_MATERIAL_PATH, TOMATO_TRUSS_MATERIAL_PATH) for path in bound)


def apply_stem_color(stage, color) -> int:
    """Recolour every stem-like mesh; returns how many were changed."""

    rgb = Gf.Vec3f(*(float(c) for c in color))
    material = get_or_create_tomato_stem_material(stage)
    shader = UsdShade.Shader.Get(stage, f"{TOMATO_STEM_MATERIAL_PATH}/Shader")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(rgb)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(STEM_ROUGHNESS)
    count = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh) and _is_stem_mesh(prim):
            UsdGeom.Mesh(prim).CreateDisplayColorAttr().Set([rgb])
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
            count += 1
    return count
