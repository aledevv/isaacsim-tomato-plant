"""Interactive-regime cost benchmark: GUI, RTX viewport, 60 Hz physics.

One stage per process (run it once per variant). After a warm-up the script
measures, frame by frame and without real-time pacing (capacity, not the
60 FPS cap):

1. ``idle``    -- simulation playing, plant at rest: physics + rendering;
2. ``push``    -- the same repeated pushes as ``push_bench.py``: plant moving;
3. ``paused``  -- timeline paused: rendering only;
4. ``physics`` -- ``world.step(render=False)``: physics only.

A playing frame is broken down into physics (4), static rendering (3) and
the rest (syncing and re-rendering moving geometry).
Also reports RSS, GPU memory, the frame budget used at 60 FPS (16.7 ms) and
20 FPS (50 ms), and the hardware/software stamp.

    ~/isaacsim/python.sh validation/complexity/interactive_bench.py \
        --usd stage.usda --output stage.interactive.json
"""

from __future__ import annotations

import argparse
import json
import platform
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
SOURCE_ROOT = HERE.parents[1] / "src"
sys.path.insert(0, str(HERE))
from push_bench import _downstream_mass, _records, choose_probes  # noqa: E402


def _gpu() -> dict:
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=name,driver_version,memory.used,memory.total",
             "--format=csv,noheader,nounits"], capture_output=True, text=True, timeout=10,
        ).stdout.strip().splitlines()[0]
        name, driver, used, total = [v.strip() for v in out.split(",")]
        return {"name": name, "driver": driver, "memory_used_mib": int(used), "memory_total_mib": int(total)}
    except Exception as exc:  # noqa: BLE001 - telemetry must not fail the run
        return {"error": str(exc)}


def _cpu() -> str:
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return platform.processor()


def _stats(samples_s: list[float]) -> dict:
    import numpy as np

    ms = 1e3 * np.asarray(samples_s)
    return {
        "frames": int(len(ms)),
        "mean_ms": float(ms.mean()),
        "p50_ms": float(np.percentile(ms, 50)),
        "p95_ms": float(np.percentile(ms, 95)),
        "p99_ms": float(np.percentile(ms, 99)),
        "fps_p50": float(1e3 / np.percentile(ms, 50)),
        "budget_60fps_p50": float(np.percentile(ms, 50) / (1e3 / 60)),
        "budget_20fps_p50": float(np.percentile(ms, 50) / 50.0),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--physics-hz", type=int, default=60)
    parser.add_argument("--warmup", type=int, default=240)
    parser.add_argument("--frames", type=int, default=600)
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=900)
    parser.add_argument("--push-g", type=float, default=1.0)
    parser.add_argument("--push-duration", type=float, default=0.2)
    parser.add_argument("--push-period", type=float, default=2.0)
    parser.add_argument("--solver", choices=("authored", "TGS", "PGS"), default="authored")
    parser.add_argument("--device", choices=("authored", "cpu", "gpu"), default="authored")
    parser.add_argument("--threads", type=int, default=None, help="PhysX CPU worker threads")
    parser.add_argument("--headless", action="store_true", help="debug only; the benchmark targets the GUI")
    args = parser.parse_args(argv)
    usd = args.usd.expanduser().resolve()

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless, "width": args.width, "height": args.height})
    try:
        import numpy as np
        import omni.timeline
        import omni.usd
        import psutil
        from isaacsim.core.api import World
        from isaacsim.core.prims import RigidPrim
        from isaacsim.core.utils.stage import is_stage_loading
        from isaacsim.core.utils.viewports import set_camera_view

        sys.path.insert(0, str(SOURCE_ROOT))
        from complexity_profile import stage_complexity
        from exporterV2.isaac_app import (
            _open_stage_and_wait,
            _register_runtime_physics_scene,
            _suspend_gravity_for_reset,
        )

        if args.threads:
            import carb.settings
            from omni.physx.bindings._physx import SETTING_NUM_THREADS

            carb.settings.get_settings().set(SETTING_NUM_THREADS, args.threads)
        load_started = time.perf_counter()
        context = omni.usd.get_context()
        if _open_stage_and_wait(context, app, usd, is_stage_loading) != usd:
            raise RuntimeError(f"cannot open {usd}")
        stage = context.get_stage()
        records = _records(stage, "/World")
        trunk = [k for k in records if k.startswith("trunk_Link_01")]
        axis = np.asarray(records[trunk[0]]["rest"])[:2] if trunk else np.zeros(2)
        probes = choose_probes(records, axis)
        targets = []
        for role, key in probes.items():
            rest = np.asarray(records[key]["rest"])
            radial = rest[:2] - axis
            tangent = np.array([-radial[1], radial[0], 0.0]) / max(np.linalg.norm(radial), 1e-9)
            force = tangent * args.push_g * 9.81 * _downstream_mass(stage, records[key]["body"])
            targets.append((records[key]["body"], np.asarray(records[key]["local"]), force))

        dt = 1.0 / args.physics_hz
        # Override on the authored PhysicsScene (World.reset re-applies it),
        # the same way fruit_experiments.py selects its solver and device.
        from pxr import Sdf, UsdPhysics

        scene = next(p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
        if args.device != "authored":
            scene.CreateAttribute("physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool).Set(args.device == "gpu")
            scene.CreateAttribute("physxScene:broadphaseType", Sdf.ValueTypeNames.Token).Set(
                "GPU" if args.device == "gpu" else "MBP")
        if args.solver != "authored":
            scene.CreateAttribute("physxScene:solverType", Sdf.ValueTypeNames.Token).Set(args.solver)
        world = World(stage_units_in_meters=1.0, physics_dt=dt, rendering_dt=1.0 / 60.0)
        context_api = world.get_physics_context()
        if args.device != "authored":
            context_api.enable_gpu_dynamics(args.device == "gpu")
        if args.solver != "authored":
            context_api.set_solver_type(args.solver)
        _register_runtime_physics_scene(stage)
        restore = _suspend_gravity_for_reset(stage)
        try:
            world.reset()
        finally:
            restore()
        world.set_simulation_dt(physics_dt=dt, rendering_dt=1.0 / 60.0)
        load_seconds = time.perf_counter() - load_started

        rests = np.asarray([r["rest"] for r in records.values()])
        centre = rests.mean(axis=0)
        side = np.mean([records[k]["rest"][1] - axis[1] for k in probes.values()])
        set_camera_view(eye=centre + np.array([0.0, (1 if side >= 0 else -1) * 1.6, 0.6]), target=centre)

        bodies = sorted({t[0] for t in targets})
        view = RigidPrim(bodies, name="interactive_push", reset_xform_properties=False,
                         prepare_contact_sensors=False)
        view.initialize()
        index = {path: i for i, path in enumerate(view.prim_paths)}
        state = {"t": 0.0, "pushing_enabled": False}

        def on_step(step_size):
            state["t"] += step_size
            if not state["pushing_enabled"] or (state["t"] % args.push_period) >= args.push_duration:
                return
            positions, orientations = view.get_world_poses()
            positions = np.asarray(positions).reshape(-1, 3)
            orientations = np.asarray(orientations).reshape(-1, 4)
            for body, local, force in targets:
                w, x, y, z = orientations[index[body]]
                u = np.array([x, y, z])
                point = local + 2.0 * np.cross(u, np.cross(u, local) + w * local) + positions[index[body]]
                view.apply_forces_and_torques_at_pos(
                    forces=force.reshape(1, 3), positions=point.reshape(1, 3),
                    indices=np.array([index[body]]), is_global=True,
                )

        world.add_physics_callback("interactive_push", on_step)

        def timed(n, step):
            samples = []
            for _ in range(n):
                started = time.perf_counter()
                step()
                samples.append(time.perf_counter() - started)
            return samples

        process = psutil.Process()
        timed(args.warmup, lambda: world.step(render=True))
        idle = timed(args.frames, lambda: world.step(render=True))
        state["pushing_enabled"] = True
        push = timed(args.frames, lambda: world.step(render=True))
        state["pushing_enabled"] = False
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        app.update()
        paused = timed(args.frames // 2, app.update)
        timeline.play()
        app.update()
        physics = timed(args.frames, lambda: world.step(render=False))

        model = stage_complexity(usd)["model"]
        report = {
            "schema_version": "autotom_interactive_bench/1.0",
            "usd": str(usd),
            "physics": {"solver": scene.GetAttribute("physxScene:solverType").Get(),
                        "gpu_dynamics": bool(scene.GetAttribute("physxScene:enableGPUDynamics").Get()),
                        "context_solver": context_api.get_solver_type(),
                        "context_gpu_dynamics": bool(context_api.is_gpu_dynamics_enabled()),
                        "threads": args.threads, "requested": {"solver": args.solver, "device": args.device}},
            "regime": {"physics_hz": args.physics_hz, "render_hz": 60, "gui": not args.headless,
                       "viewport": [args.width, args.height], "pacing": "none (capacity)"},
            "model": {k: model[k] for k in ("rigid_bodies", "dofs", "joints", "collision_shapes",
                                             "visible_triangles", "visible_mesh_prims")},
            "load_seconds": load_seconds,
            "phases": {"idle": _stats(idle), "push": _stats(push), "paused": _stats(paused),
                       "physics": _stats(physics)},
            "memory": {"rss_mib": process.memory_info().rss / 2**20, "gpu": _gpu()},
            "hardware": {"cpu": _cpu(), "cores": psutil.cpu_count(logical=True),
                         "python": platform.python_version()},
            "probes": probes,
        }
        phases = report["phases"]
        # A playing frame is physics + static rendering + the cost of syncing and
        # re-rendering moving geometry (transform updates, acceleration-structure
        # refits). "idle - paused" would wrongly book the last part as physics.
        report["breakdown_ms_p50"] = {
            "physics": phases["physics"]["p50_ms"],
            "render_static": phases["paused"]["p50_ms"],
            "sync_and_dynamic_render": phases["idle"]["p50_ms"] - phases["physics"]["p50_ms"]
            - phases["paused"]["p50_ms"],
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=1))
        print(f"[OK] {usd.name} [{report['physics']['solver']}/{'gpu' if report['physics']['gpu_dynamics'] else 'cpu'}]: idle {phases['idle']['p50_ms']:.1f} ms ({phases['idle']['fps_p50']:.0f} FPS) | "
              f"push {phases['push']['p50_ms']:.1f} ms p95 {phases['push']['p95_ms']:.1f} | "
              f"render-only {phases['paused']['p50_ms']:.1f} ms | physics-only {phases['physics']['p50_ms']:.2f} ms",
              flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
