"""Reproducible, opt-in USD ablations. Never edits the source USD."""
from __future__ import annotations

import math
from collections import Counter

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def value(prim, name, default=None):
    if prim is None or not prim:
        return default
    attr = prim.GetAttribute(name)
    result = attr.Get() if attr else None
    return default if result is None else result


def bodies_and_joints(stage):
    bodies = {str(p.GetPath()): p for p in stage.Traverse()
              if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    joints = [UsdPhysics.Joint(p) for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
    return bodies, joints


def attachment_records(stage):
    bodies, joints = bodies_and_joints(stage)
    records = []
    for joint in joints:
        a, b = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        if not a or not b or str(b[0]) not in bodies:
            continue
        if value(bodies[str(b[0])], "autotom:role") != "fruit":
            continue
        mass0 = float(UsdPhysics.MassAPI(bodies[str(a[0])]).GetMassAttr().Get())
        mass1 = float(UsdPhysics.MassAPI(bodies[str(b[0])]).GetMassAttr().Get())
        records.append({"joint": str(joint.GetPath()), "parent": str(a[0]),
                        "fruit": str(b[0]), "mass_ratio": mass1 / mass0,
                        "fruit_mass_kg": mass1, "parent_mass_kg": mass0})
    return sorted(records, key=lambda r: (r["mass_ratio"], r["fruit"]))


def joint_frame(stage, joint, side):
    targets = getattr(joint, f"GetBody{side}Rel")().GetTargets()
    pos = getattr(joint, f"GetLocalPos{side}Attr")().Get()
    rot = getattr(joint, f"GetLocalRot{side}Attr")().Get()
    local = Gf.Matrix4d(1)
    local.SetRotate(Gf.Quatd(rot))
    local.SetTranslateOnly(Gf.Vec3d(pos))
    world = UsdGeom.Xformable(stage.GetPrimAtPath(targets[0])).ComputeLocalToWorldTransform(0) if targets else Gf.Matrix4d(1)
    return local * world


def fruit_geometry(stage):
    """Compare the rendered fruit, its collision shape and its attachment in SI."""
    result = []
    for record in attachment_records(stage):
        body = stage.GetPrimAtPath(record["fruit"])
        joint = UsdPhysics.Joint(stage.GetPrimAtPath(record["joint"]))
        anchor = joint_frame(stage, joint, 1).ExtractTranslation()
        shapes = []
        for prim in Usd.PrimRange(body):
            if not prim.IsA(UsdGeom.Sphere):
                continue
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            center = matrix.ExtractTranslation()
            radius = float(UsdGeom.Sphere(prim).GetRadiusAttr().Get())
            scales = [matrix.TransformDir(Gf.Vec3d(*(1 if i == j else 0 for i in range(3)))).GetLength()
                      for j in range(3)]
            shapes.append({"path": str(prim.GetPath()), "type": "Sphere",
                           "visual_and_collider_share_prim": prim.HasAPI(UsdPhysics.CollisionAPI),
                           "world_center_m": list(center), "world_radii_m": [radius * s for s in scales],
                           "anchor_distance_from_center_m": (anchor - center).GetLength(),
                           "anchor_depth_from_sphere_surface_m": radius * scales[0] - (anchor - center).GetLength()
                           if max(scales) - min(scales) < 1e-9 else None})
        result.append({"fruit": record["fruit"], "world_attachment_m": list(anchor), "shapes": shapes})
    return result


def audit(stage):
    bodies, joints = bodies_and_joints(stage)
    errors, records, masses = [], [], []
    incoming = Counter()
    for path, prim in bodies.items():
        mass = float(UsdPhysics.MassAPI(prim).GetMassAttr().Get() or 0)
        com = UsdPhysics.MassAPI(prim).GetCenterOfMassAttr().Get()
        if mass <= 0 or not math.isfinite(mass) or not np.isfinite(com).all():
            errors.append(f"invalid mass/COM: {path}")
        masses.append({"body": path, "mass_kg": mass, "com": list(com)})
    for joint in joints:
        for side in (0, 1):
            targets = getattr(joint, f"GetBody{side}Rel")().GetTargets()
            if len(targets) > 1 or (targets and str(targets[0]) not in bodies):
                errors.append(f"invalid body{side}: {joint.GetPath()}")
        targets = joint.GetBody1Rel().GetTargets()
        if targets:
            incoming[str(targets[0])] += 1
        f0, f1 = joint_frame(stage, joint, 0), joint_frame(stage, joint, 1)
        position_error = float((f0.ExtractTranslation() - f1.ExtractTranslation()).GetLength())
        q0, q1 = f0.ExtractRotationQuat().GetNormalized(), f1.ExtractRotationQuat().GetNormalized()
        dot = abs(float(Gf.Dot(q0, q1)))
        angle = 2 * math.acos(min(1, dot))
        if position_error > 1e-6 or 1 - dot > 1e-6:
            errors.append(f"rest frame mismatch: {joint.GetPath()}")
        records.append({"joint": str(joint.GetPath()), "position_error_m": position_error,
                        "angle_error_rad": angle, "break_force_n": (
                            float(joint.GetBreakForceAttr().Get())
                            if math.isfinite(float(joint.GetBreakForceAttr().Get())) else None),
                        "excluded": bool(joint.GetExcludeFromArticulationAttr().Get())})
    errors.extend(f"expected one incoming joint: {p}, got {incoming[p]}" for p in bodies if incoming[p] != 1)
    if UsdGeom.GetStageMetersPerUnit(stage) != 1 or UsdPhysics.GetStageKilogramsPerUnit(stage) != 1:
        errors.append("expected meters and kilograms")
    return {"errors": errors, "body_count": len(bodies), "joint_count": len(joints),
            "masses": masses, "joints": records, "attachments": attachment_records(stage),
            "fruit_geometry": fruit_geometry(stage),
            "collider_count": sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse()),
            "filtered_pairs": {str(p.GetPath()): [str(x) for x in UsdPhysics.FilteredPairsAPI(p).GetFilteredPairsRel().GetTargets()]
                               for p in stage.Traverse() if p.HasAPI(UsdPhysics.FilteredPairsAPI)}}


def prepare_stage(stage, config):
    """Ablate a private stage copy; preserve retained bodies' world rest poses."""
    break_force = float(config.get("break_force", 6.0))
    if not math.isfinite(break_force) or break_force <= 0:
        raise ValueError("break force must be finite and positive")
    bodies, joints = bodies_and_joints(stage)
    fruits = attachment_records(stage)
    original_paths = set(bodies)
    if not fruits:
        raise ValueError("source USD must contain the physical fruit layer")
    selected = next((r for r in fruits if r["fruit"] == config.get("fruit")), fruits[-1])
    if config.get("fruit") and selected["fruit"] != config["fruit"]:
        raise ValueError("requested fruit not in source stage")
    scenario = config["scenario"]
    root = next(p for p in bodies if bodies[p].GetChild("RootFixedJoint"))
    keep = set(bodies)
    if scenario == "no-fruit":
        keep -= {r["fruit"] for r in fruits}
    elif scenario in {"single", "truss", "trusses", "stem-truss"}:
        supports = {p for p, prim in bodies.items()
                    if value(prim, "autotom:branchKind") in {"truss_rachis", "pedicel"}}
        if scenario == "single":
            keep = {root, selected["parent"], selected["fruit"]}
        else:
            parent_of = {str(j.GetBody1Rel().GetTargets()[0]): str(j.GetBody0Rel().GetTargets()[0])
                         for j in joints if j.GetBody1Rel().GetTargets() and j.GetBody0Rel().GetTargets()}
            truss_branch = value(bodies[parent_of[selected["parent"]]], "autotom:branchId")
            if scenario in {"truss", "stem-truss"}:
                supports = {p for p in supports if value(bodies[p], "autotom:branchId") == truss_branch
                            or value(bodies.get(parent_of.get(p)), "autotom:branchId") == truss_branch}
            keep = {root} | supports | {r["fruit"] for r in fruits if r["parent"] in supports}
            if scenario == "stem-truss":
                keep |= {p for p, prim in bodies.items() if value(prim, "autotom:branchKind") == "stem"}
    reanchored = []
    for joint in joints:
        a, b = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        if not b or str(b[0]) not in keep:
            stage.RemovePrim(joint.GetPath())
        elif a and str(a[0]) not in keep:
            if scenario == "stem-truss":
                raise ValueError("stem-truss requires a truss directly attached to the retained stem")
            frame = joint_frame(stage, joint, 0)
            root_world = UsdGeom.Xformable(bodies[root]).ComputeLocalToWorldTransform(0)
            local = frame * root_world.GetInverse()
            joint.CreateBody0Rel().SetTargets([root])
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(local.ExtractTranslation()))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(local.ExtractRotationQuat()))
            reanchored.append(str(joint.GetPath()))
    for path in sorted(set(bodies) - keep, key=len, reverse=True):
        stage.RemovePrim(path)
    # Remove relationships to discarded objects, keeping all retained filters.
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            rel = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
            rel.SetTargets([p for p in rel.GetTargets() if stage.GetPrimAtPath(p)])
    bodies, joints = bodies_and_joints(stage)
    if config["attachment"] == "internal":
        if scenario not in {"single", "truss"} or config["breakable"]:
            raise ValueError("internal attachment is restricted to small, unbreakable controls")
        edits = Sdf.BatchNamespaceEdit()
        remap = {}
        for record in attachment_records(stage):
            old = record["fruit"]
            new = "/World/Stem/" + old.rsplit("/", 1)[-1]
            edits.Add(old, new)
            remap[old] = new
        # Update all relationship targets explicitly before moving body specs.
        for prim in stage.Traverse():
            for rel in prim.GetRelationships():
                rel.SetTargets([Sdf.Path(remap.get(str(p), str(p))) for p in rel.GetTargets()])
        if not stage.GetRootLayer().Apply(edits):
            raise ValueError("failed to relocate internal fruit controls")
        bodies, joints = bodies_and_joints(stage)
    for joint in joints:
        if joint.GetPrim().GetName() == "TerminalBodyFixedJoint":
            joint.CreateBreakForceAttr().Set(break_force if config["breakable"] and not config.get("arm_after_settle") else float("inf"))
            joint.CreateExcludeFromArticulationAttr().Set(config["attachment"] == "external")
        if any(value(stage.GetPrimAtPath(p), "autotom:branchKind") in {"truss_rachis", "pedicel"}
               for p in joint.GetBody1Rel().GetTargets()):
            for axis in ("rotX", "rotY"):
                drive = UsdPhysics.DriveAPI(joint.GetPrim(), axis)
                if drive:
                    drive.GetStiffnessAttr().Set(drive.GetStiffnessAttr().Get() * config["stiffness_scale"])
                    # Preserve the requested damping ratio when stiffness
                    # changes: c = 2*zeta*sqrt(k*I), with unchanged inertia.
                    drive.GetDampingAttr().Set(drive.GetDampingAttr().Get() * config["damping_ratio"] / 4.0
                                               * math.sqrt(config["stiffness_scale"]))
    def attr(prim, name, kind, val):
        prim.CreateAttribute(name, kind).Set(val)
    scene = stage.GetPrimAtPath("/World/PhysicsScene")
    attr(scene, "physxScene:solverType", Sdf.ValueTypeNames.Token, config["solver"])
    attr(scene, "physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool, config["gpu"])
    attr(scene, "physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Int, config["hz"])
    stem = stage.GetPrimAtPath("/World/Stem")
    for name, val in (("Position", config["art_position"]), ("Velocity", config["art_velocity"])):
        attr(stem, f"physxArticulation:solver{name}IterationCount", Sdf.ValueTypeNames.Int, val)
    for prim in bodies.values():
        if value(prim, "autotom:role") == "fruit":
            for name, val in (("Position", config["fruit_position"]), ("Velocity", config["fruit_velocity"])):
                attr(prim, f"physxRigidBody:solver{name}IterationCount", Sdf.ValueTypeNames.Int, val)
    result = audit(stage)
    result.update(selected_fruit=selected["fruit"], reanchored_joints=reanchored,
                  removed_bodies=sorted(original_paths - keep))
    return result
