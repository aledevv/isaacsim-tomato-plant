"""Headless CPU probe of vegetative-branch stiffness in the main scene.

Run with Isaac Sim's Python:

    ~/isaacsim/python.sh validation/branch_stiffness/branch_stiffness_probe.py \
        --usd output/plant.usda --hz 60 --out output/probe_60hz.json

Phases: settle under gravity, then push each probed chain tip with a small
horizontal force, release it, and record the free oscillation.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--usd", type=Path, required=True)
parser.add_argument("--solver", choices=("TGS", "PGS"), help="override the authored solver")
parser.add_argument("--out", type=Path, required=True)
parser.add_argument("--settle", type=float, default=20.0)
parser.add_argument("--force", type=float, default=0.05, help="push force [N]")
parser.add_argument("--hold", type=float, default=3.0)
parser.add_argument("--free", type=float, default=5.0)
parser.add_argument("--hz", type=int, default=0, help="physics rate (default: the rate authored in the USD)")
args, _ = parser.parse_known_args()
sys.argv = sys.argv[:1]

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import carb  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading  # noqa: E402
from omni.physx import get_physx_interface, get_physx_simulation_interface  # noqa: E402
from pxr import PhysicsSchemaTools, UsdPhysics, UsdUtils  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from exporterV2 import isaac_app as ia  # noqa: E402

context = omni.usd.get_context()
ia._open_stage_and_wait(context, app, args.usd.resolve(), is_stage_loading)
stage = context.get_stage()
hz = args.hz or ia._authored_physics_hz(stage) or 60
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / hz, rendering_dt=1.0 / 60.0)
if args.solver:
    world.get_physics_context().set_solver_type(args.solver)
ia._register_runtime_physics_scene(stage)
restore = ia._suspend_gravity_for_reset(stage)
try:
    world.reset()
finally:
    restore()
world.set_simulation_dt(physics_dt=1.0 / hz, rendering_dt=1.0 / 60.0)
dt = world.get_physics_dt()

bodies = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
chains: dict[str, list[str]] = {}
for path in bodies:
    if "/Vegetative/" in path and "_Link_" in path:
        chains.setdefault(path.rsplit("/", 1)[0], []).append(path)
tips = {chain.rsplit("/", 1)[1]: sorted(links)[-1] for chain, links in chains.items()}
pushed = [name for name in tips if name.startswith("Branch_")]
pushed += sorted(n for n in tips if n.startswith("Leaf_") and n.endswith("_rachis"))[2:5]

physx = get_physx_interface()
sim = get_physx_simulation_interface()
stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()


def pos(path):
    return list(physx.get_rigidbody_transformation(path)["position"])


def dist(a, b):
    return math.dist(a, b)


rest0 = {name: pos(path) for name, path in tips.items()}
all0 = {path: pos(path) for path in bodies}
active = {"path": None, "vec": None}


def on_step(_):
    if active["path"]:
        sim.apply_force_at_pos(
            stage_id, PhysicsSchemaTools.sdfPathToInt(active["path"]),
            carb.Float3(*active["vec"]), carb.Float3(*pos(active["path"])), "Force",
        )


world.add_physics_callback("probe_force", on_step)


def run(seconds, track=None):
    trace = []
    for _ in range(int(round(seconds / dt))):
        world.step(render=False)
        if track:
            trace.append(pos(track))
    return trace


# Settle: record per-second tip drift to judge whether motion has stopped.
drift = []
prev = {n: pos(p) for n, p in tips.items()}
for second in range(int(args.settle)):
    run(1.0)
    now = {n: pos(p) for n, p in tips.items()}
    drift.append(max(dist(now[n], prev[n]) for n in tips))
    prev = now
rest = prev
max_body_move = max(dist(pos(p), all0[p]) for p in bodies)
finite = all(all(math.isfinite(c) for c in pos(p)) for p in bodies)

pushes = {}
for name in pushed:
    path = tips[name]
    base = pos(path)
    # Horizontal push perpendicular to the trunk axis, away from it.
    radial = [base[0], base[1]]
    norm = math.hypot(*radial) or 1.0
    direction = [radial[0] / norm, radial[1] / norm, 0.0] if norm > 1e-3 else [1.0, 0.0, 0.0]
    active.update(path=path, vec=[args.force * c for c in direction])
    held = run(args.hold, track=path)
    active.update(path=None, vec=None)
    free = run(args.free, track=path)
    proj = lambda p: sum((p[i] - base[i]) * direction[i] for i in range(3))  # noqa: E731
    held_d = [proj(p) for p in held]
    free_d = [proj(p) for p in free]
    static = sum(held_d[-int(0.5 / dt):]) / int(0.5 / dt)
    # Oscillation: zero crossings of the displacement about its final value.
    final = sum(free_d[-int(1.0 / dt):]) / int(1.0 / dt)
    centered = [d - final for d in free_d]
    crossings = [i for i in range(1, len(centered)) if centered[i - 1] * centered[i] < 0]
    freq = None
    if len(crossings) >= 3:
        freq = (len(crossings) - 1) / 2.0 / ((crossings[-1] - crossings[0]) * dt)
    pushes[name] = {
        "static_displacement_mm": 1e3 * static,
        "stiffness_N_per_m": args.force / static if static > 1e-6 else None,
        "peak_hold_mm": 1e3 * max(held_d),
        "overshoot_after_release_mm": 1e3 * min(centered),
        "zero_crossings": len(crossings),
        "frequency_hz": freq,
        "residual_after_free_mm": 1e3 * (final),
        "returned_to_rest_mm": 1e3 * dist(pos(path), base),
        "free_trace_mm": [round(1e3 * d, 4) for d in free_d],
    }

report = {
    "usd": str(args.usd),
    "hz": hz,
    "force_N": args.force,
    "finite": finite,
    "max_body_move_during_settle_m": max_body_move,
    "settle_tip_drift_m_per_s": drift,
    "rest_sag": {
        name: {
            "drop_mm": 1e3 * (rest0[name][2] - rest[name][2]),
            "move_mm": 1e3 * dist(rest0[name], rest[name]),
        }
        for name in tips
    },
    "pushes": pushes,
}
args.out.write_text(json.dumps(report, indent=2) + "\n")
print("[PROBE] wrote", args.out, flush=True)
app.close()
