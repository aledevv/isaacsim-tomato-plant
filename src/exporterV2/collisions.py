"""Initial-overlap check for the authored colliders.

Overlapping colliders at rest make PhysX push bodies apart on the first step.
``auto_filter_initial_overlaps`` finds each overlapping pair of rigid bodies
and disables collisions between exactly that pair; ``collider_overlaps``
reports what is left.
"""

from __future__ import annotations

import math
from typing import Any

from pxr import Gf, UsdGeom, UsdPhysics

from .core.usd.collision import add_collision_filter


def _segment_distance(a0, a1, b0, b1) -> float:
    """Shortest distance between finite 3-D line segments."""

    u = a1 - a0
    v = b1 - b0
    w = a0 - b0
    aa = u * u
    bb = u * v
    cc = v * v
    dd = u * w
    ee = v * w
    denominator = aa * cc - bb * bb
    small = 1e-15

    # Spheres are represented by zero-length swept segments.  Handle both
    # point/segment orders explicitly: the general segment algorithm below
    # assumes non-degenerate directions and used to miss sphere-as-right-hand
    # overlaps depending solely on deterministic path ordering.
    if aa < small and cc < small:
        return math.sqrt(w * w)
    if aa < small:
        t = max(0.0, min(1.0, ee / cc))
        delta = w - t * v
        return math.sqrt(delta * delta)
    if cc < small:
        s = max(0.0, min(1.0, -dd / aa))
        delta = w + s * u
        return math.sqrt(delta * delta)

    s_num = 0.0
    s_den = denominator
    t_num = 0.0
    t_den = denominator

    if denominator < small:
        s_num = 0.0
        s_den = 1.0
        t_num = ee
        t_den = cc
    else:
        s_num = bb * ee - cc * dd
        t_num = aa * ee - bb * dd
        if s_num < 0.0:
            s_num = 0.0
            t_num = ee
            t_den = cc
        elif s_num > s_den:
            s_num = s_den
            t_num = ee + bb
            t_den = cc

    if t_num < 0.0:
        t_num = 0.0
        if -dd < 0.0:
            s_num = 0.0
        elif -dd > aa:
            s_num = s_den
        else:
            s_num = -dd
            s_den = aa
    elif t_num > t_den:
        t_num = t_den
        if -dd + bb < 0.0:
            s_num = 0.0
        elif -dd + bb > aa:
            s_num = s_den
        else:
            s_num = -dd + bb
            s_den = aa

    sc = 0.0 if abs(s_num) < small else s_num / s_den
    tc = 0.0 if abs(t_num) < small else t_num / t_den
    delta = w + sc * u - tc * v
    return math.sqrt(delta * delta)


def _rigid_body_path(prim) -> str | None:
    current = prim
    while current and current.IsValid() and not current.IsPseudoRoot():
        if current.HasAPI(UsdPhysics.RigidBodyAPI):
            return str(current.GetPath())
        current = current.GetParent()
    return None


def _collider_records(stage):
    """Return deterministic swept-segment records for authored colliders.

    Capsules are exact. Cylinders use their finite centerline and radius as a
    conservative narrow phase, while spheres are zero-length segments.
    """

    records = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        body_path = _rigid_body_path(prim)
        if body_path is None:
            continue
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        center = matrix.ExtractTranslation()
        if prim.IsA(UsdGeom.Capsule):
            shape = UsdGeom.Capsule(prim)
            shape_type = "capsule"
            radius = float(shape.GetRadiusAttr().Get())
            half_spine = float(shape.GetHeightAttr().Get()) * 0.5
        elif prim.IsA(UsdGeom.Cylinder):
            shape = UsdGeom.Cylinder(prim)
            shape_type = "cylinder"
            radius = float(shape.GetRadiusAttr().Get())
            half_spine = float(shape.GetHeightAttr().Get()) * 0.5
        elif prim.IsA(UsdGeom.Sphere):
            shape = UsdGeom.Sphere(prim)
            shape_type = "sphere"
            radius = float(shape.GetRadiusAttr().Get())
            half_spine = 0.0
        else:
            continue
        direction = Gf.Vec3d(
            matrix.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
        ).GetNormalized()
        start = center - direction * half_spine
        end = center + direction * half_spine
        records.append(
            {
                "path": str(prim.GetPath()),
                "body_path": body_path,
                "shape": shape_type,
                "start": start,
                "end": end,
                "radius": radius,
                "aabb_min": tuple(
                    min(float(start[i]), float(end[i])) - radius for i in range(3)
                ),
                "aabb_max": tuple(
                    max(float(start[i]), float(end[i])) + radius for i in range(3)
                ),
            }
        )
    return sorted(records, key=lambda item: item["path"])


def _filters(stage) -> dict[str, set[str]]:
    """Every FilteredPairsAPI relationship: source prim path -> target prim paths."""

    filters = {}
    for prim in stage.Traverse():
        targets = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets()
        if targets:
            filters[str(prim.GetPath())] = {str(path) for path in targets}
    return filters


def _covers(path: str, collider: dict[str, Any]) -> bool:
    """A filter on ``path`` applies to the collider, its body or any parent."""

    return any(
        candidate == path or candidate.startswith(path + "/")
        for candidate in (collider["path"], collider["body_path"])
    )


def _is_filtered(filters: dict[str, set[str]], left: dict[str, Any], right: dict[str, Any]) -> bool:
    for source, targets in filters.items():
        for a, b in ((left, right), (right, left)):
            if _covers(source, a) and any(_covers(target, b) for target in targets):
                return True
    return False


def _body_metadata(stage, body_path: str) -> dict[str, Any]:
    prim = stage.GetPrimAtPath(body_path)
    return {
        "node_id": prim.GetAttribute("autotom:canonicalPrimitiveId").Get(),
        "canonical_organ_id": prim.GetAttribute("autotom:canonicalOrganId").Get(),
        "role": prim.GetAttribute("autotom:role").Get(),
    }


def _aabb_overlap(left, right) -> bool:
    return all(
        left["aabb_min"][axis] <= right["aabb_max"][axis]
        and right["aabb_min"][axis] <= left["aabb_max"][axis]
        for axis in range(3)
    )


def collider_overlaps(stage) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    colliders = _collider_records(stage)
    filters = _filters(stage)
    filtered = []
    active = []
    for index, left in enumerate(colliders):
        for right in colliders[index + 1 :]:
            if left["body_path"] == right["body_path"]:
                continue
            if not _aabb_overlap(left, right):
                continue
            distance = _segment_distance(
                left["start"], left["end"], right["start"], right["end"]
            )
            depth = left["radius"] + right["radius"] - distance
            if depth <= 1e-9:
                continue
            is_filtered = _is_filtered(filters, left, right)
            record = {
                "body_a": left["body_path"],
                "body_b": right["body_path"],
                "collider_a": left["path"],
                "collider_b": right["path"],
                "shape_a": left["shape"],
                "shape_b": right["shape"],
                "depth": round(float(depth), 12),
                "filtered": is_filtered,
                "metadata_a": _body_metadata(stage, left["body_path"]),
                "metadata_b": _body_metadata(stage, right["body_path"]),
            }
            (filtered if is_filtered else active).append(record)
    key = lambda value: (
        value["body_a"], value["body_b"], value["collider_a"], value["collider_b"]
    )
    return sorted(filtered, key=key), sorted(active, key=key)


def auto_filter_initial_overlaps(stage) -> list[dict[str, Any]]:
    """Filter each still-active initial overlap once at rigid-body granularity."""

    _filtered, active = collider_overlaps(stage)
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for record in active:
        pair = tuple(sorted((record["body_a"], record["body_b"])))
        grouped.setdefault(pair, []).append(record)
    applied = []
    for (body_a, body_b), contacts in sorted(grouped.items()):
        add_collision_filter(stage, body_a, body_b)
        add_collision_filter(stage, body_b, body_a)
        deepest = max(contact["depth"] for contact in contacts)
        applied.append(
            {
                "body_a": body_a,
                "body_b": body_b,
                "reason": "auto_filtered_initial_authored_overlap",
                "depth": deepest,
                "contact_count": len(contacts),
                "contacts": contacts,
                "permanent_pair_filter": True,
            }
        )
    return applied




# PhysX contact offset measured for every body of the day-160 plant in the
# 2026-09-16 branch collision experiment; pairs closer than twice this at rest
# (or 1 mm) stay filtered so they do not start in contact.
CONTACT_OFFSET_M = 0.0005555556


def enable_branch_collisions(stage, stem_path: str = "/World/Stem") -> int:
    """Let distant branches, petioles and leaf rachides collide with each other.

    The plant is one articulation and its self-collision is off, so its parts
    pass through each other. This turns self-collision on and filters every
    collider pair that must stay off: same body, jointed or within two joints,
    same branch, same organ, any pair with a fruit, truss rachis or pedicel,
    and pairs already within contact range at rest. Collider geometry, masses
    and drives are untouched. Returns the number of collider pairs that can
    touch.
    """

    import numpy as np
    from collections import defaultdict
    from itertools import combinations

    from pxr import Sdf

    def value(prim, name):
        attr = prim.GetAttribute(name)
        return attr.Get() if attr else None

    bodies = {str(p.GetPath()): p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    joints = [UsdPhysics.Joint(p) for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
    shapes = _collider_records(stage)
    adjacency = defaultdict(set)
    for joint in joints:
        a, b = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        if a and b:
            adjacency[str(a[0])].add(str(b[0]))
            adjacency[str(b[0])].add(str(a[0]))
    nearby = {p: adjacency[p].union(*(adjacency[q] for q in adjacency[p])) for p in bodies}
    margin = max(0.001, 2 * CONTACT_OFFSET_M)
    roles = [value(bodies[x["body_path"]], "autotom:role") for x in shapes]

    def eligible(i, k):
        x, y = shapes[i], shapes[k]
        bp, bq = x["body_path"], y["body_path"]
        p, q = bodies[bp], bodies[bq]
        if (bp == bq or bq in nearby[bp]
                or {"fruit", "truss_rachis", "pedicel"} & {roles[i], roles[k]}
                or value(p, "autotom:branchId") == value(q, "autotom:branchId")
                or value(p, "autotom:canonicalOrganId") == value(q, "autotom:canonicalOrganId")):
            return False
        gap = _segment_distance(x["start"], x["end"], y["start"], y["end"]) - x["radius"] - y["radius"]
        return gap > margin

    # Start from what PhysX does with self-collision off: only fruits touch the plant.
    allowed = np.zeros((len(shapes), len(shapes)), dtype=bool)
    for i, k in combinations(range(len(shapes)), 2):
        if "fruit" in (roles[i], roles[k]):
            allowed[i, k] = allowed[k, i] = True
    for joint in joints:
        a, b = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        if a and b and not joint.GetCollisionEnabledAttr().Get():
            left = [i for i, x in enumerate(shapes) if x["body_path"] == str(a[0])]
            right = [i for i, x in enumerate(shapes) if x["body_path"] == str(b[0])]
            allowed[np.ix_(left, right)] = allowed[np.ix_(right, left)] = False
    # Existing filters (on any prim, for any target subtree) become collider pairs.
    for prim in stage.Traverse():
        rel = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
        if not rel:
            continue
        roots = [str(t) for t in rel.GetTargets()]
        here = str(prim.GetPath())
        left = [i for i, x in enumerate(shapes) if x["path"] == here or x["path"].startswith(here + "/")]
        right = [i for i, x in enumerate(shapes) if any(x["path"] == r or x["path"].startswith(r + "/") for r in roots)]
        allowed[np.ix_(left, right)] = allowed[np.ix_(right, left)] = False
        rel.ClearTargets(True)
    for i, k in combinations(range(len(shapes)), 2):
        if eligible(i, k):
            allowed[i, k] = allowed[k, i] = True

    for i, x in enumerate(shapes):
        blocked = [Sdf.Path(shapes[k]["path"]) for k in range(i + 1, len(shapes))
                   if not allowed[i, k] and shapes[k]["body_path"] != x["body_path"]]
        if blocked:
            UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(x["path"])).CreateFilteredPairsRel().SetTargets(blocked)
    stage.GetPrimAtPath(stem_path).CreateAttribute(
        "physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool
    ).Set(True)
    return int(np.triu(allowed, 1).sum())
