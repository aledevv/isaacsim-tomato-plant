"""Gao et al. (2024) three-point bending test, rebuilt in PhysX with our branch joints.

Gao et al. bent 80 mm tomato-stalk pieces between two supports, pushed the
middle down and computed E = F L1^3 / (48 S I) from the force F and the
midpoint deflection S (Agriculture 14(4):531, Eq. 1-2, Table 1).

Here the same specimen is built from N rigid links joined by D6 bending
drives whose stiffness comes from the exporter's own calculate_physics_params
(k = E I / l).  The beam rests on two static cylinders, a growing force is
applied at its middle, and E is backed out with Gao's formula.

Run with Isaac Sim's Python from the repository root:

    ~/isaacsim/python.sh validation/three_point_bending/gao_three_point_bending.py \
        --links 8,16,32 --hz 7680 --out output/gao/span57_7680hz.json
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--out", type=Path, required=True)
parser.add_argument("--links", default="2,4,8,16")
parser.add_argument("--span-mm", type=float, default=57.0,
                    help="support distance; Gao do not state it, ~57 mm from their Fig. 2b")
parser.add_argument("--young-mpa", type=float, default=50.64)
parser.add_argument("--forces", default="0.5,1,2,3,4", help="midpoint loads [N]")
parser.add_argument("--preloads", default="0.05", help="loads before the measured series [N]")
parser.add_argument("--hz", type=int, default=1920)
parser.add_argument("--hold", type=float, default=1.0, help="seconds per load level")
parser.add_argument("--record", type=Path,
                    help="also ramp the load on the last case and save 60 fps link poses here")
parser.add_argument("--ramp", type=float, default=4.0, help="ramp duration for --record [s]")
parser.add_argument("--density-scale", type=float, default=100.0,
                    help="heavier links settle faster; statics are unaffected (gravity off)")
args, _ = parser.parse_known_args()
sys.argv = sys.argv[:1]

# Gao et al. Table 1 (means).
LENGTH = 0.080
OUTER_D = 11.1e-3
BAST = 3.64e-3
DENSITY = 769.96
R_OUT = OUTER_D / 2
R_IN = (OUTER_D - 2 * BAST) / 2
I_GAO = math.pi * (OUTER_D**4 - (OUTER_D - 2 * BAST) ** 4) / 64
SUPPORT_R = 5.0e-3

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True})

import carb  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from omni.physx import get_physx_interface, get_physx_simulation_interface  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, UsdGeom, UsdPhysics, UsdShade, UsdUtils  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from exporterV2 import isaac_app as ia  # noqa: E402
from exporterV2.core.tree_config import calculate_physics_params, compute_mass  # noqa: E402


def build(stage, n_links: int, young: float) -> list[str]:
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityMagnitudeAttr(0.0)
    physx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx.CreateEnableGPUDynamicsAttr(False)
    physx.CreateBroadphaseTypeAttr("MBP")
    physx.CreateSolverTypeAttr("TGS")
    physx.CreateTimeStepsPerSecondAttr(args.hz)

    link = LENGTH / n_links
    mass = compute_mass(R_OUT, link, density=DENSITY * args.density_scale, inner_radius=R_IN)
    k_deg, d_deg = calculate_physics_params(R_OUT, link, mass, young_modulus=young, inner_radius=R_IN)
    paths = []
    for i in range(n_links):
        path = f"/World/Beam/Link_{i:02d}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp().Set(Gf.Vec3d(-LENGTH / 2 + (i + 0.5) * link, 0.0, 0.2e-3))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)
        # A full-length box per link gives the beam a flat, continuous
        # underside, so it touches each support exactly where the support is
        # (capsules or spheres on short links would shift the contact point).
        cap = UsdGeom.Cube.Define(stage, path + "/Collider")
        cap.CreateSizeAttr(1.0)
        cap.AddScaleOp().Set(Gf.Vec3f(link, OUTER_D, OUTER_D))
        UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
        px = PhysxSchema.PhysxCollisionAPI.Apply(cap.GetPrim())
        px.CreateContactOffsetAttr(0.5e-3)
        px.CreateRestOffsetAttr(0.0)
        paths.append(path)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(paths[0]))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(paths[0]))
    art.CreateSolverPositionIterationCountAttr(32)
    # Short links overlap their neighbours' neighbours; only the supports should touch.
    art.CreateEnabledSelfCollisionsAttr(False)
    for i in range(n_links - 1):
        joint = UsdPhysics.Joint.Define(stage, f"/World/Beam/Joint_{i:02d}")
        joint.CreateBody0Rel().SetTargets([paths[i]])
        joint.CreateBody1Rel().SetTargets([paths[i + 1]])
        joint.CreateLocalPos0Attr(Gf.Vec3f(link / 2, 0, 0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(-link / 2, 0, 0))
        prim = joint.GetPrim()
        for dof in ("transX", "transY", "transZ", "rotX"):
            limit = UsdPhysics.LimitAPI.Apply(prim, dof)
            limit.CreateLowAttr(1.0)
            limit.CreateHighAttr(-1.0)
        for dof in ("rotY", "rotZ"):
            drive = UsdPhysics.DriveAPI.Apply(prim, dof)
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(k_deg)
            drive.CreateDampingAttr(d_deg)
            drive.CreateTargetPositionAttr(0.0)
    # Gao's supports are fixed rollers; zero friction lets the beam ends slide
    # inward as it bends, as the 48 E I formula assumes.
    material = UsdShade.Material.Define(stage, "/World/Frictionless")
    mat_api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    mat_api.CreateStaticFrictionAttr(0.0)
    mat_api.CreateDynamicFrictionAttr(0.0)
    mat_api.CreateRestitutionAttr(0.0)
    for side in (-1, 1):
        sup = UsdGeom.Capsule.Define(stage, f"/World/Support_{'L' if side < 0 else 'R'}")
        sup.CreateAxisAttr("Y")
        sup.CreateRadiusAttr(SUPPORT_R)
        sup.CreateHeightAttr(0.03)
        sup.AddTranslateOp().Set(Gf.Vec3d(side * args.span_mm * 1e-3 / 2, 0.0, -(R_OUT + SUPPORT_R)))
        UsdPhysics.CollisionAPI.Apply(sup.GetPrim())
        PhysxSchema.PhysxCollisionAPI.Apply(sup.GetPrim()).CreateContactOffsetAttr(0.5e-3)
        UsdShade.MaterialBindingAPI.Apply(sup.GetPrim()).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics")
    return paths, k_deg


def centerline_z(physx, paths, n_links, x):
    """Beam axis height at x, interpolated through the link frames."""
    link = LENGTH / n_links
    i = min(max(int((x + LENGTH / 2) / link), 0), n_links - 1)
    t = physx.get_rigidbody_transformation(paths[i])
    p = Gf.Vec3d(*t["position"])
    q = Gf.Quatd(t["rotation"][3], *t["rotation"][:3])
    local = x - (-LENGTH / 2 + (i + 0.5) * link)
    return (p + Gf.Rotation(q).TransformDir(Gf.Vec3d(local, 0, 0)))[2]


def run_case(n_links: int, record: Path | None = None) -> dict:
    context = omni.usd.get_context()
    context.new_stage()
    app.update()
    stage = context.get_stage()
    paths, k_deg = build(stage, n_links, args.young_mpa * 1e6)
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / args.hz, rendering_dt=1.0 / 60.0)
    world.get_physics_context().set_solver_type("TGS")
    # World re-applies 9.81 m/s2; the test is load-only, like Gao's machine.
    world.get_physics_context().set_gravity(0.0)
    ia._register_runtime_physics_scene(stage)
    world.reset()
    world.set_simulation_dt(physics_dt=1.0 / args.hz, rendering_dt=1.0 / 60.0)
    physx = get_physx_interface()
    sim = get_physx_simulation_interface()
    stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    half = args.span_mm * 1e-3 / 2
    # Load point: the beam middle, shared by the two middle links (N even).
    mid = [paths[n_links // 2 - 1], paths[n_links // 2]] if n_links % 2 == 0 else [paths[n_links // 2]]
    load = {"F": 0.0}

    def on_step(_):
        for path in mid:
            t = physx.get_rigidbody_transformation(path)
            sim.apply_force_at_pos(
                stage_id, PhysicsSchemaTools.sdfPathToInt(path),
                carb.Float3(0.0, 0.0, -load["F"] / len(mid)),
                carb.Float3(0.0, 0.0, t["position"][2]), "Force",
            )

    world.add_physics_callback("gao_load", on_step)
    steps = int(args.hold * args.hz)
    rows = []
    for force in [float(f) for f in args.preloads.split(",")] + [float(f) for f in args.forces.split(",")]:
        load["F"] = force
        samples = []
        for step in range(steps):
            world.step(render=False)
            if step >= steps - int(0.1 * args.hz):
                z_mid = centerline_z(physx, paths, n_links, 0.0)
                z_sup = 0.5 * (centerline_z(physx, paths, n_links, -half) + centerline_z(physx, paths, n_links, half))
                samples.append(z_sup - z_mid)
        rows.append({"force_N": force, "deflection_mm": 1e3 * sum(samples) / len(samples),
                     "jitter_mm": 1e3 * (max(samples) - min(samples))})
    if record:
        # A slow load ramp for figures and video: unload, then rise to the
        # largest load and hold, saving every link pose at 60 frames/s.
        f_max = max(float(f) for f in args.forces.split(","))
        every = args.hz // 60
        frames = []
        total = int((args.ramp + 1.5) * args.hz)
        for step in range(total):
            t = step / args.hz
            load["F"] = 0.05 if t < 0.5 else min(f_max, 0.05 + (f_max - 0.05) * (t - 0.5) / args.ramp)
            world.step(render=False)
            if step % every == 0:
                z_mid = centerline_z(physx, paths, n_links, 0.0)
                z_sup = 0.5 * (centerline_z(physx, paths, n_links, -half) + centerline_z(physx, paths, n_links, half))
                poses = []
                for path in paths:
                    tr = physx.get_rigidbody_transformation(path)
                    poses.append([*tr["position"], *tr["rotation"]])
                frames.append({"t": t, "force_N": load["F"], "deflection_mm": 1e3 * (z_sup - z_mid),
                               "links_pos_xyz_quat_xyzw": poses})
        record.parent.mkdir(parents=True, exist_ok=True)
        record.write_text(json.dumps({
            "n_links": n_links, "link_mm": 1e3 * LENGTH / n_links, "span_mm": args.span_mm,
            "outer_d_mm": OUTER_D * 1e3, "support_r_mm": SUPPORT_R * 1e3, "hz": args.hz,
            "frames": frames}) + "\n")
    world.remove_physics_callback("gao_load")
    world.stop()
    world.clear_instance()
    # Gao read E off the force-deflection line; use its slope above the 0.05 N preload.
    xs = [r["deflection_mm"] * 1e-3 for r in rows]
    ys = [r["force_N"] for r in rows]
    mx, my = sum(xs) / len(xs), sum(ys) / len(ys)
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sum((x - mx) ** 2 for x in xs)
    span = args.span_mm * 1e-3
    e_back = slope * span**3 / (48 * I_GAO)
    return {
        "n_links": n_links,
        "link_mm": 1e3 * LENGTH / n_links,
        "joint_stiffness_Nm_per_deg": k_deg,
        "rows": rows,
        "slope_N_per_mm": slope * 1e-3,
        "young_backed_out_MPa": e_back / 1e6,
        "young_input_MPa": args.young_mpa,
        "beam_theory_deflection_per_N_mm": 1e3 * span**3 / (48 * args.young_mpa * 1e6 * I_GAO),
    }


links = [int(n) for n in args.links.split(",")]
results = [run_case(n, args.record if i == len(links) - 1 else None) for i, n in enumerate(links)]
report = {
    "specimen": {"length_mm": 80, "outer_d_mm": 11.1, "bast_mm": 3.64, "density": DENSITY,
                 "span_mm": args.span_mm, "I_mm4": I_GAO * 1e12},
    "hz": args.hz,
    "density_scale": args.density_scale,
    "cases": results,
}
args.out.parent.mkdir(parents=True, exist_ok=True)
args.out.write_text(json.dumps(report, indent=2) + "\n")
for r in results:
    print(f"[GAO] N={r['n_links']:2d} link={r['link_mm']:.1f}mm  E_back={r['young_backed_out_MPa']:.2f} MPa "
          f"(input {r['young_input_MPa']}) slope={r['slope_N_per_mm']:.3f} N/mm", flush=True)
app.close()
