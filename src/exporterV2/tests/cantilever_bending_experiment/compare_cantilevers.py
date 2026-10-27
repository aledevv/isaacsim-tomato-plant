import os
import sys
import argparse

parser = argparse.ArgumentParser(description="Compare Cantilevers Side-by-Side")
parser.add_argument("--n", type=int, default=10, choices=[3, 5, 10, 15, 20], help="Number of links (N) to compare")
parser.add_argument(
    "--benchmark",
    default="synthetic_solid_40cm",
    choices=["synthetic_solid_40cm", "tomato_gao_20cm"],
    help="Benchmark geometry/material set to compare",
)
args = parser.parse_args()

# Isaac Sim headless setup
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni
import omni.kit.viewport.utility
from isaacsim.core.api import World
from pxr import Usd, UsdGeom, Gf

def compare_cantilevers(n_links, benchmark):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(script_dir))))
    data_dir = os.path.join(repo_root, "data", "usd_models", "physics_tests")
    
    suffix = f"fixed_d6_biaxial_N{n_links}.usda"
    baseline_usd = os.path.join(data_dir, f"cantilever_{benchmark}_legacy_current_{suffix}")
    fixes_usd = os.path.join(data_dir, f"cantilever_{benchmark}_new_physics_{suffix}")
    
    if not os.path.exists(baseline_usd) or not os.path.exists(fixes_usd):
        print(f"[Error] Missing USD files for {benchmark} N={n_links}. Did you run the generator?")
        simulation_app.close()
        sys.exit(1)
        
    print(f"\n[INFO] Loading comparison for {benchmark} N={n_links}...")
    
    # Create empty stage
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    # Add light
    omni.kit.commands.execute(
        'CreatePrimWithDefaultXform',
        prim_type='DomeLight',
        attributes={'inputs:intensity': 1000}
    )
    
    # Load baseline
    print("[INFO] Loading Baseline (Old Physics) at Y = -0.3 m")
    baseline_prim_path = "/World/Baseline"
    baseline_prim = stage.DefinePrim(baseline_prim_path, "Xform")
    baseline_prim.GetReferences().AddReference(baseline_usd)
    UsdGeom.Xformable(baseline_prim).AddTranslateOp().Set(Gf.Vec3d(0, -0.3, 0))
    
    # Load after_fixes
    print("[INFO] Loading Fixed (New Physics) at Y = +0.3 m")
    fixes_prim_path = "/World/Fixed"
    fixes_prim = stage.DefinePrim(fixes_prim_path, "Xform")
    fixes_prim.GetReferences().AddReference(fixes_usd)
    UsdGeom.Xformable(fixes_prim).AddTranslateOp().Set(Gf.Vec3d(0, 0.3, 0))
    
    # Initialize World
    world = World(stage_units_in_meters=1.0)
    world.reset()
    
    print("\n" + "=" * 80)
    print("  ✓ Comparison ready!")
    print("  - The one on the left/back (Y=-0.3) is the OLD behavior")
    print("  - The one on the right/front (Y=+0.3) is the NEW behavior")
    print("  Close the window to exit")
    print("=" * 80 + "\n")
    
    while simulation_app.is_running():
        world.step(render=True)
        
    world.stop()
    world.clear_instance()

if __name__ == "__main__":
    compare_cantilevers(args.n, args.benchmark)
    simulation_app.close()
