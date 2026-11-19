"""Per-organ gravity-settle fidelity of optimized stages against a reference.

``run`` (Isaac Sim Python) settles a stage headless with the same reset and
cadence as ``isaac_app.py`` and writes, for every visual mesh, its rest and
settled world position (the centre of its local point bounds, carried by its
rigid body). ``compare`` (plain Python) matches meshes across variants and
reports the error against the reference.

Meshes are keyed independently of link merging: the leaflet id for leaf
visuals (``Leaflet_g<id>_<side><nn>``), otherwise the innermost link
component (``Merged_`` prefix stripped) plus the mesh name.

    ~/isaacsim/python.sh validation/complexity/fidelity_bench.py run \
        --usd stage.usda --output stage.fidelity.json --duration 5
    uv run python validation/complexity/fidelity_bench.py compare \
        --reference ref.fidelity.json variant=variant.fidelity.json ...
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import time
from pathlib import Path

SOURCE_ROOT = Path(__file__).resolve().parents[2] / "src"
_LEAFLET = re.compile(r"Leaflet_g\d+_[a-z]+\d\d")
ROLES = (
    ("Leaflet_", "leaflet"),
    ("_pedicel_", "pedicel"),
    ("_rachis_Link_", "rachis"),
    ("_petiole_Link_", "petiole"),
    ("Branch_", "branch"),
    ("trunk_", "stem"),
)


def mesh_key(path: str) -> str:
    parts = path.split("/")
    name = parts[-1]
    leaflet = _LEAFLET.search(path)
    if leaflet:
        return f"{leaflet.group(0)}/{name}"
    for part in reversed(parts[:-1]):
        if "_Link_" in part:
            return f"{part.removeprefix('Merged_')}/{name}"
    return path


def role_of(key: str) -> str:
    for token, role in ROLES:
        if token in key:
            return role
    return "other"


def _rest_points(stage) -> dict[str, dict]:
    import numpy as np
    from pxr import Gf, UsdGeom, UsdPhysics

    bodies = {str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    records: dict[str, dict] = {}
    duplicates = set()
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        body = prim.GetParent()
        while body and not body.IsPseudoRoot() and str(body.GetPath()) not in bodies:
            body = body.GetParent()
        if not body or body.IsPseudoRoot():
            continue
        points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get() or [], dtype=np.float64)
        if not len(points):
            continue
        centre = (points.min(axis=0) + points.max(axis=0)) / 2.0
        mesh_world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        body_world = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(0)
        world = mesh_world.Transform(Gf.Vec3d(*centre))
        local = body_world.GetInverse().Transform(world)
        key = mesh_key(str(prim.GetPath()))
        if key in records:
            duplicates.add(key)
        records[key] = {
            "body": str(body.GetPath()),
            "local": [float(v) for v in local],
            "rest": [float(v) for v in world],
        }
    for key in duplicates:
        records.pop(key)
    return records


def run(args: argparse.Namespace) -> int:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        import numpy as np
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.prims import RigidPrim
        from isaacsim.core.utils.stage import is_stage_loading

        sys.path.insert(0, str(SOURCE_ROOT))
        from exporterV2.isaac_app import (
            _open_stage_and_wait,
            _register_runtime_physics_scene,
            _suspend_gravity_for_reset,
        )

        usd = args.usd.expanduser().resolve()
        context = omni.usd.get_context()
        if _open_stage_and_wait(context, app, usd, is_stage_loading) != usd:
            raise RuntimeError(f"cannot open {usd}")
        stage = context.get_stage()
        records = _rest_points(stage)
        world = World(stage_units_in_meters=1.0, physics_dt=1.0 / args.physics_hz, rendering_dt=1.0 / 60.0)
        _register_runtime_physics_scene(stage)
        restore = _suspend_gravity_for_reset(stage)
        try:
            world.reset()
        finally:
            restore()
        world.set_simulation_dt(physics_dt=1.0 / args.physics_hz, rendering_dt=1.0 / 60.0)
        if int(round(1.0 / world.get_physics_dt())) != args.physics_hz:
            raise RuntimeError("physics rate mismatch after reset")
        body_paths = sorted({record["body"] for record in records.values()})
        view = RigidPrim(body_paths, name="fidelity_bodies", reset_xform_properties=False,
                         prepare_contact_sensors=False)
        view.initialize()
        steps = int(round(args.duration * args.physics_hz))
        started = time.perf_counter()
        for _ in range(steps):
            world.step(render=False)
        wall = time.perf_counter() - started
        positions, orientations = view.get_world_poses()
        positions = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
        orientations = np.asarray(orientations, dtype=np.float64).reshape(-1, 4)
        pose = dict(zip(view.prim_paths, zip(positions, orientations), strict=True))
        for record in records.values():
            p, (w, x, y, z) = pose[record["body"]]
            v = np.asarray(record["local"])
            u = np.array([x, y, z])
            rotated = v + 2.0 * np.cross(u, np.cross(u, v) + w * v)
            settled = rotated + p
            if not np.all(np.isfinite(settled)):
                raise RuntimeError(f"non-finite pose for {record['body']}")
            record["settled"] = [float(c) for c in settled]
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps({
            "schema_version": "autotom_fidelity_settle/1.0",
            "usd": str(usd),
            "physics_hz": args.physics_hz,
            "duration_s": args.duration,
            "steps": steps,
            "wall_seconds": wall,
            "rigid_bodies": len(body_paths),
            "meshes": records,
        }, indent=1, sort_keys=True))
        print(f"[OK] fidelity {usd.name}: {len(records)} meshes, {len(body_paths)} bodies, {wall:.1f}s", flush=True)
        return 0
    finally:
        app.close()


def compare(args: argparse.Namespace) -> int:
    import numpy as np

    reference = json.loads(args.reference.read_text())["meshes"]
    rows = []
    for item in args.variants:
        label, _, raw = item.partition("=")
        document = json.loads(Path(raw).read_text())
        meshes = document["meshes"]
        common = sorted(reference.keys() & meshes.keys())
        by_role: dict[str, list[float]] = {}
        rel: list[float] = []
        for key in common:
            ref_settled = np.asarray(reference[key]["settled"])
            ref_rest = np.asarray(reference[key]["rest"])
            error = float(np.linalg.norm(np.asarray(meshes[key]["settled"]) - ref_settled))
            by_role.setdefault(role_of(key), []).append(error)
            sag = float(np.linalg.norm(ref_settled - ref_rest))
            if sag > 1e-3:
                rel.append(error / sag)
        errors = np.concatenate([np.asarray(v) for v in by_role.values()]) if by_role else np.zeros(1)
        rows.append({
            "variant": label,
            "bodies": document["rigid_bodies"],
            "matched": f"{len(common)}/{len(reference)}",
            "wall_s": round(document["wall_seconds"], 1),
            "mean_mm": 1e3 * float(errors.mean()),
            "p95_mm": 1e3 * float(np.percentile(errors, 95)),
            "max_mm": 1e3 * float(errors.max()),
            "rel_sag_median": float(np.median(rel)) if rel else 0.0,
            **{f"{role}_mean_mm": 1e3 * float(np.mean(values)) for role, values in sorted(by_role.items())},
        })
    columns = list(dict.fromkeys(key for row in rows for key in row))
    print("| " + " | ".join(columns) + " |")
    print("|" + "---|" * len(columns))
    for row in rows:
        cells = [f"{row[c]:.2f}" if isinstance(row.get(c), float) else str(row.get(c, "")) for c in columns]
        print("| " + " | ".join(cells) + " |")
    if args.output:
        args.output.write_text(json.dumps(rows, indent=1))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    run_parser = sub.add_parser("run")
    run_parser.add_argument("--usd", type=Path, required=True)
    run_parser.add_argument("--output", type=Path, required=True)
    run_parser.add_argument("--duration", type=float, default=5.0)
    run_parser.add_argument("--physics-hz", type=int, default=480)
    compare_parser = sub.add_parser("compare")
    compare_parser.add_argument("--reference", type=Path, required=True)
    compare_parser.add_argument("--output", type=Path)
    compare_parser.add_argument("variants", nargs="+")
    args = parser.parse_args(argv)
    return run(args) if args.command == "run" else compare(args)


if __name__ == "__main__":
    raise SystemExit(main())
