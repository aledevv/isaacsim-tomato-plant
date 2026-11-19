"""Static estimate of the rest bend of every D6 joint of an exported plant.

For each joint with a bending drive, the moment of the weight of everything
downstream of the joint (bodies reached through joints, fruits included) is
divided by the drive stiffness (N m per degree), which gives the bend angle at
rest under gravity in degrees. It is a hand calculation on the USD, not a
simulation: the angles are small, so the joints are treated as linear springs
and the geometry as undeformed.

    uv run python validation/branch_stiffness/rest_angles.py output/plant.usda
    uv run python validation/branch_stiffness/rest_angles.py output/plant.usda --json output/rest_angles.json
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics

G = 9.81

# Organ of a joint from the name of its link chain (first match wins).
ORGANS = (
    ("_pedicel_", "pedicel"),
    ("Truss_", "truss rachis"),
    ("Leaf_", "leaf"),
    ("Branch_", "branch"),
    ("trunk", "stem"),
)


def organ_of(path: str) -> str:
    for token, organ in ORGANS:
        if token in path:
            return organ
    return "other"


def rest_angles(usd: Path) -> list[dict]:
    stage = Usd.Stage.Open(str(usd))
    cache = UsdGeom.XformCache()
    bodies: dict[str, tuple[float, Gf.Vec3d]] = {}
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.HasAPI(UsdPhysics.MassAPI):
            mass_api = UsdPhysics.MassAPI(prim)
            mass = mass_api.GetMassAttr().Get() or 0.0
            com = mass_api.GetCenterOfMassAttr().Get()
            if com is None or not all(math.isfinite(c) for c in com):
                com = Gf.Vec3f(0.0)  # unauthored: USD's fallback is -inf; use the body origin
            world = cache.GetLocalToWorldTransform(prim).Transform(Gf.Vec3d(*com))
            bodies[str(prim.GetPath())] = (float(mass), world)

    children: dict[str, list[str]] = defaultdict(list)
    drives = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        body0 = joint.GetBody0Rel().GetTargets()
        body1 = joint.GetBody1Rel().GetTargets()
        if not body0 or not body1:
            continue
        parent, child = str(body0[0]), str(body1[0])
        children[parent].append(child)
        stiffness = prim.GetAttribute("drive:rotX:physics:stiffness")
        if stiffness and stiffness.Get():
            limit = prim.GetAttribute("limit:rotX:physics:high")
            local = joint.GetLocalPos0Attr().Get()
            position = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(parent)).Transform(Gf.Vec3d(*local))
            drives.append((str(prim.GetPath()), child, float(stiffness.Get()),
                           float(limit.Get()) if limit and limit.Get() is not None else None, position))

    def downstream(body: str) -> list[str]:
        seen, stack = set(), [body]
        while stack:
            current = stack.pop()
            if current not in seen:
                seen.add(current)
                stack.extend(children.get(current, ()))
        return [b for b in seen if b in bodies]

    rows = []
    for path, child, stiffness, limit, position in drives:
        torque_x = torque_y = mass_total = 0.0
        for body in downstream(child):
            mass, centre = bodies[body]
            arm = centre - position
            weight = -mass * G  # force along -Z; torque = arm x force
            torque_x += arm[1] * weight
            torque_y += -arm[0] * weight
            mass_total += mass
        torque = math.hypot(torque_x, torque_y)
        rows.append({
            "joint": path,
            "organ": organ_of(path),
            "angle_deg": torque / stiffness,  # stiffness is per degree
            "limit_deg": limit,
            "stiffness_nm_per_deg": stiffness,
            "torque_nm": torque,
            "downstream_mass_g": 1e3 * mass_total,
        })
    return rows


def summary(rows: list[dict]) -> dict[str, dict]:
    by_organ: dict[str, list[float]] = defaultdict(list)
    for row in rows:
        by_organ[row["organ"]].append(row["angle_deg"])
    return {
        organ: {"joints": len(values), "median_deg": statistics.median(values), "max_deg": max(values)}
        for organ, values in sorted(by_organ.items())
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("usd", type=Path)
    parser.add_argument("--json", type=Path, help="also write every joint to this file")
    args = parser.parse_args(argv)
    rows = rest_angles(args.usd)
    if not rows:
        print("[ERROR] no D6 joint with a bending drive found")
        return 1
    over = sum(1 for r in rows if r["limit_deg"] and r["angle_deg"] > r["limit_deg"])
    print(f"{len(rows)} joints with a bending drive, {over} above their limit")
    print("| organ | joints | median (deg) | max (deg) |")
    print("|---|---:|---:|---:|")
    for organ, values in summary(rows).items():
        print(f"| {organ} | {values['joints']} | {values['median_deg']:.2f} | {values['max_deg']:.2f} |")
    if args.json:
        args.json.write_text(json.dumps({"usd": str(args.usd), "summary": summary(rows), "joints": rows}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
