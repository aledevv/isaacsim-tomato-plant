"""Tomato calyx: five small green sepals where the pedicel meets the fruit.

Purely decorative (``fruits.calyx`` in config/visuals.yaml): a mesh next to
the tomato sphere, under the same prim, so it follows the fruit when it
detaches. It has no collider and no mass. The shape is photo-inspired, not
measured.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdShade

from ..core.tree_config import PlantColors
from ..core.usd.materials import get_or_create_tomato_stem_material


def calyx_mesh(radius: float, axis) -> tuple[np.ndarray, list[int], list[int]]:
    """Sepals on a sphere of ``radius`` centred at the origin, around ``axis``
    (unit vector from the fruit centre towards the pedicel)."""

    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    helper = np.eye(3)[np.argmin(np.abs(axis))]
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    points: list[np.ndarray] = []
    counts: list[int] = []
    indices: list[int] = []
    # Five tapered sepals with a raised central ridge and slightly lifted tips,
    # lying just outside the sphere surface.
    for blade in range(5):
        theta = 2 * math.pi * blade / 5
        radial = math.cos(theta) * u + math.sin(theta) * v
        side = np.cross(axis, radial)
        start = len(points)
        for row in range(13):
            t = row / 12
            distance = radius * (0.055 + 0.70 * t)
            width = radius * (0.09 + 0.10 * math.sin(math.pi * t)) * (1 - t) + radius * 0.001
            for across in (-1, 0, 1):
                offset = side * width * across + radial * distance
                height = math.sqrt(max(radius**2 - float(offset @ offset), 0.0))
                height += radius * (0.012 + 0.065 * t**5 + (0.018 if across == 0 else 0))
                points.append(offset + axis * height)
            if row:
                previous = start + (row - 1) * 3
                for col in range(2):
                    indices.extend([previous + col, previous + col + 1, previous + col + 4, previous + col + 3])
                    counts.append(4)
    # A small hub closes the base of the sepals around the pedicel insertion.
    hub = len(points)
    points.append(axis * radius * 1.045)
    for i in range(20):
        theta = i * math.tau / 20
        points.append(axis * radius * 1.012 + radius * 0.16 * (math.cos(theta) * u + math.sin(theta) * v))
    for i in range(20):
        indices.extend([hub, hub + 1 + i, hub + 1 + (i + 1) % 20])
        counts.append(3)
    return np.asarray(points), counts, indices


def _pedicel_points(stage) -> np.ndarray:
    """World points of every visible pedicel mesh."""

    cache = UsdGeom.XformCache()
    chunks = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh) or "_pedicel_" not in str(prim.GetPath()):
            continue
        if UsdGeom.Imageable(prim).ComputeVisibility() == UsdGeom.Tokens.invisible:
            continue
        points = UsdGeom.Mesh(prim).GetPointsAttr().Get()
        if not points:
            continue
        local = np.c_[np.asarray(points, dtype=float), np.ones(len(points))]
        chunks.append((local @ np.asarray(cache.GetLocalToWorldTransform(prim)))[:, :3])
    return np.vstack(chunks) if chunks else np.zeros((0, 3))


def author_calyces(stage) -> list[dict[str, Any]]:
    """Add a calyx to every tomato sphere, facing its pedicel."""

    spheres = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Sphere) and prim.GetName() == "Sphere"]
    if not spheres:
        return []
    pedicels = _pedicel_points(stage)
    material = get_or_create_tomato_stem_material(stage)
    cache = UsdGeom.XformCache()
    authored = []
    for sphere in spheres:
        holder = sphere.GetParent()
        radius = float(UsdGeom.Sphere(sphere).GetRadiusAttr().Get())
        to_world = cache.GetLocalToWorldTransform(holder)
        center = np.asarray(to_world.ExtractTranslation())
        elbow = holder.GetParent().GetChild("GravityElbowPedicelVisual")
        if elbow.IsValid():
            # Last point of the curved pedicel is the centre of its tip.
            tip = UsdGeom.Mesh(elbow).GetPointsAttr().Get()[-1]
            world_axis = np.asarray(cache.GetLocalToWorldTransform(elbow).Transform(Gf.Vec3d(tip))) - center
        elif len(pedicels):
            nearest = pedicels[np.argmin(np.linalg.norm(pedicels - center, axis=1))]
            world_axis = nearest - center
        else:
            world_axis = np.array([0.0, 0.0, 1.0])
        if np.linalg.norm(world_axis) < 1e-9:
            world_axis = np.array([0.0, 0.0, 1.0])
        local_axis = to_world.GetInverse().TransformDir(Gf.Vec3d(*world_axis))
        points, counts, indices = calyx_mesh(radius, tuple(local_axis))
        mesh = UsdGeom.Mesh.Define(stage, holder.GetPath().AppendChild("Calyx"))
        mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([PlantColors.PEDICEL])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
        mesh.GetPrim().CreateAttribute("autotom:visualOnly", Sdf.ValueTypeNames.Bool, custom=True).Set(True)
        authored.append({"fruit_path": str(holder.GetPath()), "calyx_path": str(mesh.GetPath())})
    return authored
