"""Presentation figures for the complexity study (static PNG, light surface).

    uv run python validation/complexity/make_figures.py \
        --push push_all.json --fidelity fidelity_compare.json \
        --skeleton ref=ref.usda --skeleton b120=b120.usda ... --out figures/

Palette: the dataviz skill's validated categorical slots 1-4, assigned in
fixed order; the reference is neutral ink. Two slots are below 3:1 contrast
on the light surface; the relief is the always-present legend plus the
labelled points of the cost/fidelity figure.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
# Colour follows the compliance policy (dataviz categorical slots, fixed order).
POLICY_COLOURS = {
    "load": "#2a78d6",
    "keep": "#eb6834",
    "series": "#1baf7a",
    "g206": "#eda100",
    "tip": "#e87ba4",
    "blend": "#008300",
}
POLICY_NAMES = {
    "load": "load (gravity-weighted)",
    "tip": "tip (tip-load-weighted)",
    "blend": "blend (mean of load and tip)",
    "keep": "keep (legacy: merged = rigid)",
    "series": "series (full compliance)",
    "g206": "206 D6, 255 bodies",
}
TRAJECTORY_STAGES = ("ref", "b40_load", "b40_tip", "b40_blend", "b40_keep")


def policy_of(variant: str) -> str:
    return "g206" if variant.startswith("g206") else variant.split("_", 1)[1]


def colour_of(variant: str) -> str:
    return INK if variant == "ref" else POLICY_COLOURS[policy_of(variant)]


def name_of(variant: str) -> str:
    if variant == "ref":
        return "reference (206 D6, 216 bodies)"
    if variant.startswith("g206"):
        return POLICY_NAMES["g206"]
    budget, policy = variant.split("_", 1)
    return f"{budget[1:]} D6, {policy}"


PROBES = {"lateral_tip": "Lateral branch tip", "truss_tip": "Truss rachis tip", "leaf_blade": "Leaf blade"}


def _style():
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": GRID, "axes.labelcolor": INK_2, "xtick.color": INK_2, "ytick.color": INK_2,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
        "axes.spines.top": False, "axes.spines.right": False,
        "font.size": 11, "axes.titlesize": 13, "axes.titleweight": "bold", "axes.titlecolor": INK,
        "lines.linewidth": 2.0, "legend.frameon": False,
    })


def push_trajectories(push: dict, out: Path) -> Path:
    results = push["results"]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6), sharex=True)
    for ax, (probe, title) in zip(axes, PROBES.items()):
        ax.axvspan(0, push["push_duration_s"], color=GRID, alpha=0.7, lw=0)
        ax.text(push["push_duration_s"] + 0.04, 0.03, "push", transform=ax.get_xaxis_transform(),
                ha="left", va="bottom", color=INK_2, fontsize=9)
        ax.axhline(0, color=INK_2, lw=0.8)
        for stage in TRAJECTORY_STAGES:
            row = next((r for r in results if r["stage"] == stage and r["probe"] == probe), None)
            if row is None:
                continue
            t, y = row["trajectory"]["t"], row["trajectory"]["along_mm"]
            ax.plot(t, y, color=colour_of(stage), lw=2.6 if stage == "ref" else 1.8,
                    zorder=3 if stage == "ref" else 2, label=name_of(stage))
        force = next(r["force_n"] for r in results if r["probe"] == probe)
        ax.set_title(f"{title}  (F = {force:.2f} N)", loc="left")
        ax.set_xlabel("time since push start [s]")
    axes[0].set_ylabel("displacement along push [mm]")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=5, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle("Lateral push equal to the downstream weight: 40-D6 plants with each compliance policy vs reference",
                 x=0.01, ha="left", fontsize=14, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0.07, 1, 0.95))
    path = out / "push_trajectories.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def cost_vs_fidelity(fidelity: list[dict], push: dict, out: Path) -> Path:
    rows = {r["variant"]: r for r in fidelity}
    ref_wall = rows["ref"]["wall_s"]
    ref_peak = {r["probe"]: r["peak_mm"] for r in push["results"] if r["stage"] == "ref"}
    # Policies only change drive gains, so every policy at one budget has the
    # same topology and cost: use the median wall time per budget, which
    # removes run-to-run timing noise (up to ~20% observed).
    by_budget: dict[str, list[float]] = {}
    for variant, row in rows.items():
        by_budget.setdefault(variant.split("_", 1)[0], []).append(row["wall_s"])
    budget_wall = {budget: float(np.median(walls)) for budget, walls in by_budget.items()}

    def speed(variant):
        return ref_wall / budget_wall[variant.split("_", 1)[0]]

    def dynamic(variant):
        values = [abs(r["peak_mm"] - ref_peak[r["probe"]]) / ref_peak[r["probe"]] * 100
                  for r in push["results"] if r["stage"] == variant]
        return float(np.mean(values)) if values else None

    fig, (left, right) = plt.subplots(1, 2, figsize=(13, 5.0))
    policies = dict.fromkeys(policy_of(v) for v in rows if v != "ref")
    for policy in policies:
        variants = sorted((v for v in rows if v != "ref" and policy_of(v) == policy),
                          key=speed)
        colour = POLICY_COLOURS[policy]
        for ax, metric in ((left, lambda v: rows[v]["mean_mm"]), (right, dynamic)):
            points = [(speed(v), metric(v)) for v in variants if metric(v) is not None]
            if not points:
                continue
            xs, ys = zip(*points)
            ax.plot(xs, ys, color=colour, lw=1.4, alpha=0.7, zorder=2)
            ax.scatter(xs, ys, s=70, color=colour, edgecolor=SURFACE, linewidth=2, zorder=3,
                       label=POLICY_NAMES[policy])
            ax.annotate(policy, (xs[-1], ys[-1]), xytext=(8, 0), textcoords="offset points",
                        va="center", fontsize=9, color=INK)
    left.set_title("Static fidelity (gravity settle)", loc="left")
    left.set_ylabel("mean organ position error [mm]")
    right.set_title("Dynamic fidelity (lateral push)", loc="left")
    right.set_ylabel("mean |peak - reference peak| [% of reference]")
    for ax in (left, right):
        ax.set_xlabel("physics speed-up vs reference [x]")
        ax.set_ylim(bottom=0)
        ax.set_xlim(0.8, 3.0)
    handles, labels = left.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, bbox_to_anchor=(0.5, -0.01))
    fig.suptitle("Cost vs fidelity per compliance policy (day 160; points = 120 / 60 / 40 D6)",
                 x=0.01, y=0.985, ha="left", fontsize=14, fontweight="bold", color=INK)
    fig.text(0.01, 0.925, "Speed-up: median wall time per budget of 2400 physics steps at 480 Hz, "
             "RTX 4080 Laptop / i9-13900HK", ha="left", color=INK_2, fontsize=10)
    fig.tight_layout(rect=(0, 0.1, 1, 0.9))
    path = out / "cost_vs_fidelity.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def _skeleton(usd: Path):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(usd))
    bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    segments = []
    for index, body in enumerate(bodies):
        for prim in Usd.PrimRange(body):
            if prim.IsA(UsdGeom.Capsule):
                capsule = UsdGeom.Capsule(prim)
                half = 0.5 * capsule.GetHeightAttr().Get() + capsule.GetRadiusAttr().Get()
                world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
                a = world.Transform(Gf.Vec3d(0, 0, -half))
                b = world.Transform(Gf.Vec3d(0, 0, half))
                segments.append((index, (a[0], a[2]), (b[0], b[2])))
    d6 = []
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsJoint":
            target = prim.GetRelationship("physics:body1").GetTargets()[0]
            world = UsdGeom.Xformable(stage.GetPrimAtPath(target)).ComputeLocalToWorldTransform(0)
            p = world.Transform(Gf.Vec3d(prim.GetAttribute("physics:localPos1").Get()))
            d6.append((p[0], p[2]))
    return segments, np.asarray(d6), len(bodies)


def complexity_skeletons(stages: list[tuple[str, Path]], out: Path) -> Path:
    fig, axes = plt.subplots(1, len(stages), figsize=(4.2 * len(stages), 6.2), sharey=True)
    tones = ("#8a8984", "#c9c8c2")
    for ax, (label, usd) in zip(np.atleast_1d(axes), stages):
        segments, d6, bodies = _skeleton(usd)
        for index, a, b in segments:
            ax.plot([a[0], b[0]], [a[1], b[1]], color=tones[index % 2], lw=2.2, solid_capstyle="round")
        if len(d6):
            ax.scatter(d6[:, 0], d6[:, 1], s=26, color="#2a78d6", edgecolor=SURFACE, linewidth=1.2, zorder=3)
        ax.set_title(f"{len(d6)} D6 joints | {bodies} bodies", loc="center")
        ax.text(0.5, -0.04, name_of(label), transform=ax.transAxes, ha="center", va="top",
                color=INK_2, fontsize=10)
        ax.set_aspect("equal")
        ax.grid(False)
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)
    fig.suptitle("Same plant, fewer joints: blue = flexible D6 joint, grey tones = rigid bodies",
                 x=0.01, ha="left", fontsize=14, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0.02, 1, 0.94))
    path = out / "complexity_skeletons.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--push", type=Path, required=True)
    parser.add_argument("--fidelity", type=Path, required=True)
    parser.add_argument("--skeleton", action="append", default=[], help="LABEL=USD")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    _style()
    push = json.loads(args.push.read_text())
    fidelity = json.loads(args.fidelity.read_text())
    paths = [push_trajectories(push, args.out), cost_vs_fidelity(fidelity, push, args.out)]
    if args.skeleton:
        stages = [(item.partition("=")[0], Path(item.partition("=")[2])) for item in args.skeleton]
        paths.append(complexity_skeletons(stages, args.out))
    for path in paths:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
