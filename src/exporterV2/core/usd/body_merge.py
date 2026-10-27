"""Merge articulation links into their upstream rigid body on an authored stage.

Used by joint-budget optimization (``joints.optimize: full``): the stage is first
built and audited with one body per canonical link, then selected links are
absorbed into the body upstream of them. Absorbing link ``X`` into ``P``
(the ``body0`` of the tree joint whose ``body1`` is ``X``):

* every child prim of ``X`` (meshes, colliders, leaf visuals, joint prims)
  moves under ``P/Merged_<X>``, an Xform carrying the rest transform of ``X``
  in ``P``, so all geometry keeps its exact world pose;
* mass is summed and the centre of mass mass-weighted; inertia is left to
  PhysX, which derives it from the combined collision shapes;
* the joint ``P``-``X`` is deleted, and every other joint or filtered pair
  that referenced ``X`` is re-targeted to ``P`` with its frame re-expressed;
* the angular compliance of the removed joint ``J`` moves to the joint ``U``
  upstream of ``P``: ``1/K_U' = 1/K_U + w / K_J``, damping scaled by
  ``sqrt(K_U' / K_U)``. The weight ``w`` depends on ``stiffness_policy``:

  - ``"load"``: ``w = (M_J d_J) / (M_U d_U)`` clamped to [0, 1], where ``M``
    is the rest-pose gravity moment carried by a joint and ``d`` its distance
    to the centre of mass of the part downstream of ``J``. This keeps the
    small-rotation displacement of that part under self-weight;
  - ``"tip"``: ``w = (d_J / d_U)**2`` clamped to [0, 1], where ``d`` is the
    distance from a joint to the farthest link endpoint downstream of ``J``.
    This keeps the tip displacement under a point load at the tip;
  - ``"blend"``: the mean of the ``load`` and ``tip`` weights;
  - ``"series"``: ``w = 1``, which keeps the rotation under an end moment but
    over-predicts sag under gravity (the root carries the full moment);
  - ``"keep"``: ``w = 0``, the merged section becomes rigid (legacy
    optimizer behaviour).
"""

from __future__ import annotations

from typing import Any, Iterable

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


STIFFNESS_POLICIES = ("load", "tip", "blend", "series", "keep")
_ANGULAR_AXES = ("rotX", "rotY", "rotZ", "angular")
_TRACK_ATTR = "autotom:mergeTrackPath"
_TIME = Usd.TimeCode.Default()


class BodyMergeError(ValueError):
    """Raised when a requested merge would not preserve the authored model."""


def _is_joint(prim) -> bool:
    return prim.IsA(UsdPhysics.Joint)


def _target(prim, name: str) -> str | None:
    rel = prim.GetRelationship(name)
    targets = rel.GetTargets() if rel else []
    return str(targets[0]) if targets else None


def _tree_joint(stage, body: str):
    """Return the articulation joint whose body1 is ``body``."""

    found = [
        prim
        for prim in stage.Traverse()
        if _is_joint(prim)
        and _target(prim, "physics:body1") == body
        and not prim.GetAttribute("physics:excludeFromArticulation").Get()
    ]
    if len(found) != 1:
        raise BodyMergeError(f"{body} has {len(found)} upstream tree joints, expected 1")
    return found[0]


def _world(stage, path: str) -> Gf.Matrix4d:
    return UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(_TIME)


def _joint_frame(prim, side: int) -> Gf.Matrix4d:
    pos = prim.GetAttribute(f"physics:localPos{side}").Get() or Gf.Vec3f(0.0)
    rot = prim.GetAttribute(f"physics:localRot{side}").Get() or Gf.Quatf(1.0)
    return Gf.Matrix4d(Gf.Rotation(Gf.Quatd(rot)), Gf.Vec3d(pos))


def _set_joint_frame(prim, side: int, matrix: Gf.Matrix4d) -> None:
    prim.GetAttribute(f"physics:localPos{side}").Set(Gf.Vec3f(matrix.ExtractTranslation()))
    quat = matrix.ExtractRotationQuat()
    prim.GetAttribute(f"physics:localRot{side}").Set(Gf.Quatf(quat.GetNormalized()))


def _drive_axes(prim) -> list[str]:
    return [
        schema.split(":", 1)[1]
        for schema in prim.GetAppliedSchemas()
        if schema.startswith("PhysicsDriveAPI:") and schema.split(":", 1)[1] in _ANGULAR_AXES
    ]


def _bodies_below(stage, body: str) -> list[str]:
    children: dict[str, list[str]] = {}
    for prim in stage.Traverse():
        if _is_joint(prim):
            parent, child = _target(prim, "physics:body0"), _target(prim, "physics:body1")
            if parent and child:
                children.setdefault(parent, []).append(child)
    found, stack = [], [body]
    while stack:
        current = stack.pop()
        found.append(current)
        stack.extend(children.get(current, ()))
    return found


def _mass_and_com(stage, bodies: list[str]) -> tuple[float, Gf.Vec3d]:
    mass, moment = 0.0, Gf.Vec3d(0.0)
    for path in bodies:
        api = UsdPhysics.MassAPI(stage.GetPrimAtPath(path))
        m = float(api.GetMassAttr().Get() or 0.0)
        com = Gf.Vec3d(api.GetCenterOfMassAttr().Get() or Gf.Vec3f(0.0))
        mass += m
        moment += _world(stage, path).Transform(com) * m
    return mass, (moment / mass if mass > 0 else moment)


def _joint_world_position(stage, joint) -> Gf.Vec3d:
    body = _target(joint, "physics:body1")
    local = Gf.Vec3d(joint.GetAttribute("physics:localPos1").Get() or Gf.Vec3f(0.0))
    return _world(stage, body).Transform(local)


def _load_weight(stage, removed, upstream) -> float:
    """Share of the downstream part's gravity sag produced by ``removed``."""

    _, reference = _mass_and_com(stage, _bodies_below(stage, _target(removed, "physics:body1")))

    def moment_times_arm(joint) -> float:
        pivot = _joint_world_position(stage, joint)
        mass, com = _mass_and_com(stage, _bodies_below(stage, _target(joint, "physics:body1")))
        lever = com - pivot
        moment = mass * (lever[0] ** 2 + lever[1] ** 2) ** 0.5
        return moment * (reference - pivot).GetLength()

    up = moment_times_arm(upstream)
    if up <= 0.0:
        return 1.0
    return min(max(moment_times_arm(removed) / up, 0.0), 1.0)


def _tip_weight(stage, removed, upstream) -> float:
    """Share of the tip displacement under a tip load produced by ``removed``.

    A point load P at the tip T rotates joint j by P*d_j/K_j and moves T by
    P*d_j**2/K_j (d_j = |T - joint j|), so one equivalent joint at U keeps the
    tip displacement with w = (d_J / d_U)**2. T is the farthest link endpoint
    downstream of the removed joint.
    """

    pivot = _joint_world_position(stage, removed)
    tip, best = pivot, -1.0
    for path in _bodies_below(stage, _target(removed, "physics:body1")):
        prim = stage.GetPrimAtPath(path)
        length = prim.GetAttribute("autotom:sourceLength").Get() or 0.0
        end = _world(stage, path).Transform(Gf.Vec3d(0.0, 0.0, float(length)))
        distance = (end - pivot).GetLength()
        if distance > best:
            tip, best = end, distance
    d_up = (tip - _joint_world_position(stage, upstream)).GetLength()
    if d_up <= 0.0:
        return 1.0
    return min(max((best / d_up) ** 2, 0.0), 1.0)


def _transfer_compliance(stage, removed, upstream, policy: str) -> dict[str, Any]:
    removed_axes = {
        axis: UsdPhysics.DriveAPI(removed, axis).GetStiffnessAttr().Get() or 0.0
        for axis in _drive_axes(removed)
    }
    record = {"removed_joint": str(removed.GetPath()), "removed_stiffness": removed_axes}
    if not removed_axes:
        record["status"] = "rigid_joint_removed"
        return record
    if policy == "keep":
        record["status"] = "kept"
        return record
    if upstream is None or upstream.GetTypeName() != "PhysicsJoint":
        record["status"] = "compliance_lost_no_driven_upstream"
        return record
    record["upstream_joint"] = str(upstream.GetPath())
    weights = {"series": lambda: 1.0, "load": lambda: _load_weight(stage, removed, upstream),
               "tip": lambda: _tip_weight(stage, removed, upstream),
               "blend": lambda: 0.5 * (_load_weight(stage, removed, upstream)
                                       + _tip_weight(stage, removed, upstream))}
    weight = weights[policy]()
    record["weight"] = weight
    changed = {}
    for axis in _drive_axes(upstream):
        k_removed = removed_axes.get(axis)
        drive = UsdPhysics.DriveAPI(upstream, axis)
        k_up = drive.GetStiffnessAttr().Get() or 0.0
        if not k_removed or not k_up:
            continue
        k_new = 1.0 / (1.0 / k_up + weight / k_removed)
        damping = drive.GetDampingAttr().Get() or 0.0
        drive.GetStiffnessAttr().Set(k_new)
        drive.GetDampingAttr().Set(damping * (k_new / k_up) ** 0.5)
        changed[axis] = {"before": k_up, "after": k_new}
    record["status"] = policy if changed else "compliance_lost_axis_mismatch"
    record["upstream_stiffness"] = changed
    return record


def _snapshot(stage) -> dict[str, Any]:
    geometry = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Gprim):
            key = str(prim.GetPath())
            prim.CreateAttribute(_TRACK_ATTR, Sdf.ValueTypeNames.String, custom=True).Set(key)
            geometry[key] = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(_TIME)
    mass = 0.0
    moment = Gf.Vec3d(0.0)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.HasAPI(UsdPhysics.MassAPI):
            api = UsdPhysics.MassAPI(prim)
            m = float(api.GetMassAttr().Get() or 0.0)
            com = Gf.Vec3d(api.GetCenterOfMassAttr().Get() or Gf.Vec3f(0.0))
            mass += m
            moment += _world(stage, str(prim.GetPath())).Transform(com) * m
    return {"geometry": geometry, "mass": mass, "moment": moment}


def _counts(stage) -> dict[str, int]:
    prims = list(stage.Traverse())
    return {
        "rigid_bodies": sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in prims),
        "d6_joints": sum(p.GetTypeName() == "PhysicsJoint" for p in prims),
        "fixed_joints": sum(p.GetTypeName() == "PhysicsFixedJoint" for p in prims),
        "meshes": sum(p.IsA(UsdGeom.Mesh) for p in prims),
        "colliders": sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in prims),
    }


def _merge_one(stage, absorbed: str, policy: str) -> dict[str, Any]:
    prim = stage.GetPrimAtPath(absorbed)
    if not prim or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise BodyMergeError(f"{absorbed} is not a rigid body")
    joint = _tree_joint(stage, absorbed)
    joint_type = joint.GetTypeName()
    survivor = _target(joint, "physics:body0")
    if survivor is None:
        raise BodyMergeError(f"{absorbed} is anchored to the world and cannot be merged")
    survivor_prim = stage.GetPrimAtPath(survivor)
    for api in ("physics:diagonalInertia", "physics:principalAxes"):
        for source in (prim, survivor_prim):
            if source.GetAttribute(api).HasAuthoredValue():
                raise BodyMergeError(f"{source.GetPath()} authors {api}; merging needs it unset")
    upstream = None
    try:
        upstream = _tree_joint(stage, survivor)
    except BodyMergeError:
        upstream = None  # survivor is the world-anchored root

    in_survivor = _world(stage, absorbed) * _world(stage, survivor).GetInverse()
    compliance = _transfer_compliance(stage, joint, upstream, policy)

    # Mass and centre of mass, expressed in the survivor frame.
    absorbed_mass = UsdPhysics.MassAPI(prim)
    survivor_mass = UsdPhysics.MassAPI(survivor_prim)
    m_a = float(absorbed_mass.GetMassAttr().Get())
    m_s = float(survivor_mass.GetMassAttr().Get())
    com_a = in_survivor.Transform(Gf.Vec3d(absorbed_mass.GetCenterOfMassAttr().Get()))
    com_s = Gf.Vec3d(survivor_mass.GetCenterOfMassAttr().Get())
    survivor_mass.GetMassAttr().Set(m_a + m_s)
    survivor_mass.GetCenterOfMassAttr().Set(Gf.Vec3f((com_s * m_s + com_a * m_a) / (m_a + m_s)))
    for name in ("autotom:aggregatedLeafVisualMassKg",):
        if prim.GetAttribute(name).HasAuthoredValue():
            total = (survivor_prim.GetAttribute(name).Get() or 0.0) + prim.GetAttribute(name).Get()
            survivor_prim.CreateAttribute(name, Sdf.ValueTypeNames.Double, custom=True).Set(total)
    organs = list(survivor_prim.GetAttribute("autotom:representedOrganIds").Get() or [])
    for organ in prim.GetAttribute("autotom:representedOrganIds").Get() or []:
        if organ not in organs:
            organs.append(organ)
    survivor_prim.CreateAttribute(
        "autotom:representedOrganIds", Sdf.ValueTypeNames.StringArray, custom=True
    ).Set(organs)
    merged_attr = survivor_prim.CreateAttribute(
        "autotom:mergedLinkPaths", Sdf.ValueTypeNames.StringArray, custom=True
    )
    merged_attr.Set(
        [*(merged_attr.Get() or []), absorbed, *(prim.GetAttribute("autotom:mergedLinkPaths").Get() or [])]
    )

    # Move children under a wrapper carrying the absorbed rest transform.
    wrapper = f"{survivor}/Merged_{prim.GetName()}"
    UsdGeom.Xform.Define(stage, wrapper).AddTransformOp().Set(in_survivor)
    layer = stage.GetRootLayer()
    joint_path = str(joint.GetPath())
    for child in prim.GetChildren():
        if str(child.GetPath()) == joint_path:
            continue
        if not Sdf.CopySpec(layer, child.GetPath(), layer, Sdf.Path(f"{wrapper}/{child.GetName()}")):
            raise BodyMergeError(f"cannot copy {child.GetPath()}")
    own_filters = [
        str(t) for t in UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets()
    ] if prim.HasAPI(UsdPhysics.FilteredPairsAPI) else []
    stage.RemovePrim(joint_path)
    stage.RemovePrim(absorbed)
    moved_children = {f"{absorbed}/": f"{wrapper}/"}

    def mapped(path: str) -> str:
        if path == absorbed:
            return survivor
        for old, new in moved_children.items():
            if path.startswith(old):
                return new + path[len(old):]
        return path

    # Re-target joints and filtered pairs.
    if own_filters:
        UsdPhysics.FilteredPairsAPI.Apply(survivor_prim)
        rel = UsdPhysics.FilteredPairsAPI(survivor_prim).GetFilteredPairsRel()
        for target in own_filters:
            rel.AddTarget(target)
    for other in stage.Traverse():
        if _is_joint(other):
            for side in (0, 1):
                body = _target(other, f"physics:body{side}")
                if body == absorbed:
                    other.GetRelationship(f"physics:body{side}").SetTargets([survivor])
                    _set_joint_frame(other, side, _joint_frame(other, side) * in_survivor)
        for rel in other.GetRelationships():
            targets = [str(t) for t in rel.GetTargets()]
            remapped = [mapped(t) for t in targets]
            if rel.GetName() == "physics:filteredPairs":
                own = str(other.GetPath())
                unique = []
                for target in remapped:
                    if target != own and target not in unique:
                        unique.append(target)
                remapped = unique
            if remapped != targets:
                rel.SetTargets(remapped)
    return {
        "absorbed": absorbed,
        "survivor": survivor,
        "mass_kg": m_a,
        "removed_joint_type": joint_type,
        "compliance": compliance,
    }


def merge_rigid_links(
    stage,
    absorbed_paths: Iterable[str],
    *,
    stiffness_policy: str = "blend",
    tolerance: float = 1e-6,
) -> dict[str, Any]:
    """Absorb each listed body into its upstream body and audit the result."""

    if stiffness_policy not in STIFFNESS_POLICIES:
        raise BodyMergeError(f"stiffness_policy must be one of {STIFFNESS_POLICIES}")
    absorbed_paths = list(absorbed_paths)
    if len(set(absorbed_paths)) != len(absorbed_paths):
        raise BodyMergeError("duplicate absorbed bodies")
    before_counts = _counts(stage)
    before = _snapshot(stage)
    merges = [_merge_one(stage, path, stiffness_policy) for path in absorbed_paths]

    errors = []
    after_counts = _counts(stage)
    removed_d6 = sum(m["removed_joint_type"] == "PhysicsJoint" for m in merges)
    expected = {
        "rigid_bodies": before_counts["rigid_bodies"] - len(merges),
        "d6_joints": before_counts["d6_joints"] - removed_d6,
        "fixed_joints": before_counts["fixed_joints"] - (len(merges) - removed_d6),
        "meshes": before_counts["meshes"],
        "colliders": before_counts["colliders"],
    }
    for key, value in expected.items():
        if after_counts[key] != value:
            errors.append(f"{key} {after_counts[key]} != expected {value}")
    seen = set()
    for prim in stage.Traverse():
        attr = prim.GetAttribute(_TRACK_ATTR)
        if not attr or not attr.HasAuthoredValue():
            continue
        key = attr.Get()
        seen.add(key)
        world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(_TIME)
        if not Gf.IsClose(world, before["geometry"][key], tolerance):
            errors.append(f"geometry moved: {key} -> {prim.GetPath()}")
        prim.RemoveProperty(_TRACK_ATTR)
    missing = set(before["geometry"]) - seen
    if missing:
        errors.append(f"{len(missing)} geometry prims lost, e.g. {sorted(missing)[0]}")
    after = _snapshot(stage)
    for prim in stage.Traverse():
        if prim.HasAttribute(_TRACK_ATTR):
            prim.RemoveProperty(_TRACK_ATTR)
    if abs(after["mass"] - before["mass"]) > tolerance * max(1.0, before["mass"]):
        errors.append(f"total mass {after['mass']} != {before['mass']}")
    if before["mass"] > 0 and (
        after["moment"] / after["mass"] - before["moment"] / before["mass"]
    ).GetLength() > tolerance:
        errors.append("plant centre of mass moved")
    removed = set(absorbed_paths)
    for prim in stage.Traverse():
        for rel in prim.GetRelationships():
            for target in rel.GetTargets():
                if str(target) in removed or any(str(target).startswith(p + "/") for p in removed):
                    errors.append(f"{prim.GetPath()}.{rel.GetName()} still targets {target}")
    if errors:
        raise BodyMergeError("; ".join(errors[:10]))
    return {
        "stiffness_policy": stiffness_policy,
        "before": before_counts,
        "after": after_counts,
        "merges": merges,
    }
