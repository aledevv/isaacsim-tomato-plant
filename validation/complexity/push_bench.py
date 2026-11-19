"""Lateral push test: dynamic response of optimized stages vs a reference.

Several stages are placed side by side in one composite scene (one active
PhysicsScene). The plants settle under gravity, then the same probe organs
of every plant receive the same horizontal force for ``--push-duration``
seconds and the response is recorded for ``--observe`` seconds.

Probes (chosen on the first stage, matched by mesh key in the others): the
most cantilevered lateral branch tip, truss rachis tip and leaf blade. The
force on a probe is ``--push-g`` times the weight of the plant part downstream
of the probe's body in the first stage, applied at the probe point,
horizontal and tangential to the stem axis.

``--gui`` repeats settle/push/observe cycles with real-time pacing and draws
the forces as red arrows; without it, one cycle runs headless and a JSON
report with trajectories and metrics is written.

    ~/isaacsim/python.sh validation/complexity/push_bench.py \
        --stage ref=ref.usda --stage b40=b40.usda --gui
    ~/isaacsim/python.sh validation/complexity/push_bench.py \
        --stage ref=ref.usda --stage b40=b40.usda --settle 6 --output push.json
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE_ROOT = HERE.parents[1] / "src"
sys.path.insert(0, str(HERE))
from fidelity_bench import mesh_key  # noqa: E402

PROBE_ROLES = ("lateral_tip", "truss_tip", "leaf_blade")


class _RealtimePacer:
    """Sleep so that simulated time does not run ahead of wall time (GUI only)."""

    def __init__(self) -> None:
        self._start = time.perf_counter()

    def wait(self, simulated_seconds: float) -> None:
        ahead = simulated_seconds - (time.perf_counter() - self._start)
        if ahead > 0.0:
            time.sleep(ahead)


def build_composite(stages: list[tuple[str, Path]], output: Path) -> tuple[Path, dict[str, str]]:
    """Reference every stage's /World side by side; keep one PhysicsScene."""

    from pxr import Gf, Usd, UsdGeom

    extents = []
    for _, path in stages:
        source = Usd.Stage.Open(str(path))
        bound = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]).ComputeWorldBound(
            source.GetPrimAtPath("/World/Stem")
        ).ComputeAlignedRange()
        extents.append(bound)
    width = max(r.GetSize()[0] for r in extents)
    spacing = 1.3 * width
    composite = Usd.Stage.CreateNew(str(output))
    composite.SetMetadata("upAxis", "Z")
    composite.SetMetadata("metersPerUnit", 1.0)
    UsdGeom.Xform.Define(composite, "/World")
    roots = {}
    for index, (label, path) in enumerate(stages):
        root = f"/World/{label}"
        prim = UsdGeom.Xform.Define(composite, root)
        prim.GetPrim().GetReferences().AddReference(str(path.resolve()), "/World")
        prim.AddTranslateOp().Set(Gf.Vec3d(index * spacing, 0.0, 0.0))
        if index:
            composite.GetPrimAtPath(f"{root}/PhysicsScene").SetActive(False)
        roots[label] = root
    composite.SetDefaultPrim(composite.GetPrimAtPath("/World"))
    composite.GetRootLayer().Save()
    return output, roots


def _records(stage, root: str) -> dict[str, dict]:
    """Mesh centre per key under ``root``: body, point in body frame, rest."""

    import numpy as np
    from pxr import Gf, UsdGeom, UsdPhysics

    bodies = {
        str(p.GetPath())
        for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith(root + "/")
    }
    records = {}
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not prim.IsA(UsdGeom.Mesh) or not path.startswith(root + "/"):
            continue
        body = prim.GetParent()
        while body and not body.IsPseudoRoot() and str(body.GetPath()) not in bodies:
            body = body.GetParent()
        if not body or body.IsPseudoRoot():
            continue
        points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get() or [], dtype=np.float64)
        if not len(points):
            continue
        # Use the far end of the mesh along its local Z (the organ tip).
        tip = points[np.argmax(points[:, 2])]
        world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).Transform(Gf.Vec3d(*tip))
        body_world = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(0)
        records[mesh_key(path)] = {
            "body": str(body.GetPath()),
            "local": list(body_world.GetInverse().Transform(world)),
            "rest": list(world),
        }
    return records


def _downstream_mass(stage, body: str) -> float:
    from pxr import UsdPhysics

    children: dict[str, list[str]] = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint):
            b0 = prim.GetRelationship("physics:body0").GetTargets()
            b1 = prim.GetRelationship("physics:body1").GetTargets()
            if b0 and b1:
                children.setdefault(str(b0[0]), []).append(str(b1[0]))
    mass, stack = 0.0, [body]
    while stack:
        current = stack.pop()
        mass += float(UsdPhysics.MassAPI(stage.GetPrimAtPath(current)).GetMassAttr().Get() or 0.0)
        stack.extend(children.get(current, ()))
    return mass


def choose_probes(records: dict[str, dict], axis_xy) -> dict[str, str]:
    """Most cantilevered (largest horizontal distance from the stem) per role."""

    def reach(key):
        x, y, _ = records[key]["rest"]
        return math.hypot(x - axis_xy[0], y - axis_xy[1])

    candidates = {
        "lateral_tip": [k for k in records if k.startswith("Branch_")],
        "truss_tip": [k for k in records if k.startswith("Truss_") and "_rachis_Link_" in k],
        "leaf_blade": [k for k in records if k.startswith("Leaflet_")],
    }
    return {role: max(sorted(keys), key=reach) for role, keys in candidates.items() if keys}


def _metrics(times, displacement, direction, push_end):
    import numpy as np

    t = np.asarray(times)
    d = np.asarray(displacement)
    along = d @ np.asarray(direction)
    norm = np.linalg.norm(d, axis=1)
    peak = int(np.argmax(norm))
    after = t >= push_end
    residual = float(norm[-1])
    # Oscillation after release: zero crossings of the detrended signal.
    signal = along[after] - along[after][-1] if after.any() else np.zeros(1)
    crossings = np.where(np.diff(np.signbit(signal)))[0]
    frequency = None
    if len(crossings) >= 3:
        half_periods = np.diff(t[after][crossings])
        frequency = float(1.0 / (2.0 * np.mean(half_periods)))
    return {
        "peak_mm": 1e3 * float(norm[peak]),
        "peak_time_s": float(t[peak]),
        "residual_mm": 1e3 * residual,
        "frequency_hz": frequency,
        "zero_crossings": int(len(crossings)),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--stage", action="append", required=True, help="LABEL=PATH, first is the reference")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--physics-hz", type=int, default=60)
    parser.add_argument("--settle", type=float, default=3.0)
    parser.add_argument("--push-duration", type=float, default=0.2)
    parser.add_argument("--push-g", type=float, default=1.0)
    parser.add_argument("--observe", type=float, default=4.0)
    parser.add_argument("--slow", type=float, default=1.0,
                        help="GUI playback speed (0.3 = slow motion); physics is unchanged")
    args = parser.parse_args(argv)
    stages = []
    for item in args.stage:
        label, _, raw = item.partition("=")
        stages.append((label, Path(raw).expanduser().resolve()))
    output = args.output or stages[-1][1].with_suffix(".push.json")

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": not args.gui})
    try:
        composite_path, roots = build_composite(stages, output.with_suffix(".composite.usda"))
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

        context = omni.usd.get_context()
        if _open_stage_and_wait(context, app, composite_path, is_stage_loading) != composite_path:
            raise RuntimeError(f"cannot open {composite_path}")
        stage = context.get_stage()
        records = {label: _records(stage, root) for label, root in roots.items()}
        reference_label = stages[0][0]
        ref = records[reference_label]
        root_offset = {
            label: np.asarray(stage.GetPrimAtPath(root).GetAttribute("xformOp:translate").Get())
            for label, root in roots.items()
        }
        trunk = [k for k in ref if k.startswith("trunk_Link_01")]
        axis = np.asarray(ref[trunk[0]]["rest"])[:2] if trunk else np.zeros(2)
        probes = choose_probes(ref, axis)
        targets = []  # (label, role, body, local point, force vector)
        for role, key in probes.items():
            rest = np.asarray(ref[key]["rest"])
            radial = rest[:2] - axis
            tangent = np.array([-radial[1], radial[0], 0.0])
            tangent /= np.linalg.norm(tangent)
            magnitude = args.push_g * 9.81 * _downstream_mass(stage, ref[key]["body"])
            for label in roots:
                if key not in records[label]:
                    raise RuntimeError(f"probe {key} missing in {label}")
                record = records[label][key]
                targets.append((label, role, record["body"], np.asarray(record["local"]), tangent * magnitude))

        dt = 1.0 / args.physics_hz
        world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=1.0 / 60.0)
        _register_runtime_physics_scene(stage)
        restore = _suspend_gravity_for_reset(stage)
        try:
            world.reset()
        finally:
            restore()
        world.set_simulation_dt(physics_dt=dt, rendering_dt=1.0 / 60.0)
        body_paths = sorted({t[2] for t in targets})
        view = RigidPrim(body_paths, name="push_bodies", reset_xform_properties=False,
                         prepare_contact_sensors=False)
        view.initialize()
        index = {path: i for i, path in enumerate(view.prim_paths)}

        def points_now():
            positions, orientations = view.get_world_poses()
            positions = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
            orientations = np.asarray(orientations, dtype=np.float64).reshape(-1, 4)
            result = []
            for label, role, body, local, force in targets:
                w, x, y, z = orientations[index[body]]
                u = np.array([x, y, z])
                result.append(local + 2.0 * np.cross(u, np.cross(u, local) + w * local) + positions[index[body]])
            return np.asarray(result)

        state = {"t": 0.0, "cycle_start": args.settle, "pushing": False, "log": None, "base": None}
        cycle = args.push_duration + args.observe

        def on_step(step_size):
            state["t"] += step_size
            phase = state["t"] - state["cycle_start"]
            if phase >= 0.0 and state["base"] is None:
                state["base"] = points_now()
                state["log"] = {"t": [], "points": []}
            state["pushing"] = 0.0 <= phase < args.push_duration
            if state["pushing"]:
                points = points_now()
                # One call per target keeps each force at its own point.
                for i, (label, role, body, local, force) in enumerate(targets):
                    view.apply_forces_and_torques_at_pos(
                        forces=force.reshape(1, 3), positions=points[i].reshape(1, 3),
                        indices=np.array([index[body]]), is_global=True,
                    )
            if state["log"] is not None and phase <= cycle:
                state["log"]["t"].append(phase)
                state["log"]["points"].append(points_now() - state["base"])

        world.add_physics_callback("push_bench", on_step)

        if args.gui:
            from isaacsim.core.utils.viewports import set_camera_view
            from pxr import UsdPhysics
            from push_overlay import PushOverlay

            plants = {}
            for label, root in roots.items():
                prims = [p for p in stage.Traverse() if str(p.GetPath()).startswith(root + "/")]
                bodies = sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in prims)
                d6 = sum(p.GetTypeName() == "PhysicsJoint" for p in prims)
                rests = np.asarray([r["rest"] for r in records[label].values()])
                top = rests[np.argmax(rests[:, 2])]
                plants[label] = {
                    "caption": f"{label}  |  {d6} D6  |  {bodies} bodies",
                    "caption_at": [float(top[0]), float(top[1]), float(top[2]) + 0.12],
                    "centre": rests.mean(axis=0),
                }
            centres = np.asarray([info["centre"] for info in plants.values()])
            middle = centres.mean(axis=0)
            span = max(1.0, np.ptp(centres[:, 0]) + 1.0)
            # Look from the side the pushed organs face, so they are in front.
            probe_side = np.mean([records[reference_label][key]["rest"][1] - axis[1] for key in probes.values()])
            sign = 1.0 if probe_side >= 0 else -1.0
            set_camera_view(eye=middle + np.array([0.0, sign * 1.5 * span, 0.35 * span]), target=middle)
            overlay = PushOverlay(targets, plants)
            pacer = _RealtimePacer()
            print("[GUI] plants left to right:", ", ".join(roots), flush=True)
            try:
                while app.is_running():
                    world.step(render=True)
                    pacer.wait(state["t"] / max(args.slow, 1e-3))
                    points = points_now()
                    phase = state["t"] - state["cycle_start"]
                    if phase < 0.0:
                        name = "SETTLING"
                    elif state["pushing"]:
                        name = "PUSH"
                    else:
                        name = "FREE RESPONSE"
                    displacement = points - state["base"] if state["base"] is not None else None
                    speed = (f"playback {args.slow:g}x  |  simulated {state['t']:.1f} s  |  "
                             f"physics {args.physics_hz} Hz, {len(roots)} plants")
                    overlay.update(points, name, phase, state["pushing"],
                                   0.0 <= phase < args.push_duration + 1.0, displacement, speed)
                    if phase > cycle:
                        state["cycle_start"] = state["t"] + 1.0
                        state["base"] = None
                        state["log"] = None
            finally:
                overlay.close()
            return 0

        total = args.settle + cycle
        started = time.perf_counter()
        while state["t"] < total - 1e-9:
            world.step(render=False)
        wall = time.perf_counter() - started
        times = state["log"]["t"]
        trajectories = np.asarray(state["log"]["points"])
        report = {
            "schema_version": "autotom_push_bench/1.0",
            "stages": {label: str(path) for label, path in stages},
            "physics_hz": args.physics_hz,
            "settle_s": args.settle,
            "push_duration_s": args.push_duration,
            "push_g": args.push_g,
            "observe_s": args.observe,
            "wall_seconds": wall,
            "probes": probes,
            "results": [],
        }
        for i, (label, role, body, local, force) in enumerate(targets):
            direction = force / np.linalg.norm(force)
            report["results"].append({
                "stage": label,
                "probe": role,
                "key": probes[role],
                "force_n": float(np.linalg.norm(force)),
                **_metrics(times, trajectories[:, i, :], direction, args.push_duration),
                "trajectory": {"t": times, "along_mm": [1e3 * float(v) for v in trajectories[:, i, :] @ direction]},
            })
        output.write_text(json.dumps(report, indent=1))
        for result in report["results"]:
            freq = f"{result['frequency_hz']:.2f} Hz" if result["frequency_hz"] else "-"
            print(f"[PUSH] {result['stage']:>12} {result['probe']:>11} F={result['force_n']*1e3:.1f} mN "
                  f"peak={result['peak_mm']:.1f} mm at {result['peak_time_s']:.2f}s "
                  f"residual={result['residual_mm']:.1f} mm f={freq}", flush=True)
        print(f"[OK] push report {output} ({wall:.1f}s)", flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
