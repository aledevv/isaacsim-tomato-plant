"""Export the plant variants of the complexity study.

Same plant, same geometry, fewer D6 joints: the reference and the optimizer
budgets 120, 60 and 40 with every compliance policy. Tomatoes are visual
(no rigid bodies), as in the study, so the reference has 206 D6 joints and
216 bodies.

    uv run python validation/complexity/make_variants.py --out output/complexity
    uv run python validation/complexity/make_variants.py --out output/complexity \
        --budgets 40 --policies load,tip,blend,keep,series
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from exporterV2.config import load_physics_config, load_visuals_config  # noqa: E402
from exporterV2.exporter import export_plant  # noqa: E402
from plant_skeleton import load_skeleton  # noqa: E402

POLICIES = ("load", "tip", "blend", "keep", "series")


def variant_physics(base, budget: int | None, policy: str):
    physics = copy.deepcopy(base)
    physics.values["fruits"]["mode"] = "visual"
    if budget is not None:
        physics.values["joints"]["optimize"] = "full"
        physics.values["joints"]["joint_target"] = budget
        physics.values["joints"]["merge_stiffness"] = policy
    return physics


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--input", type=Path, default=ROOT / "input" / "skeleton.json")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--budgets", default="120,60,40")
    parser.add_argument("--policies", default="blend", help=f"comma list of {', '.join(POLICIES)}")
    args = parser.parse_args(argv)
    policies = [p for p in args.policies.split(",") if p]
    unknown = set(policies) - set(POLICIES)
    if unknown:
        parser.error(f"unknown policy: {', '.join(sorted(unknown))}")
    skeleton = load_skeleton(args.input)
    physics = load_physics_config()
    visuals = load_visuals_config()
    args.out.mkdir(parents=True, exist_ok=True)
    jobs = [("ref", None, "blend")]
    jobs += [(f"b{b}_{p}", int(b), p) for b in args.budgets.split(",") if b for p in policies]
    for label, budget, policy in jobs:
        result = export_plant(skeleton, args.out / f"{label}.usda", variant_physics(physics, budget, policy), visuals)
        print(f"[OK] {label}: {result.usd_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
