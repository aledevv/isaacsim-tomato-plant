"""Fit the interactive cost model and write the cost table and figure.

For each physics device it fits ``physics_ms = a + b*DOF + c*bodies`` on
the per-variant median of ``interactive_bench.py`` reports (least squares),
with R^2 and leave-one-out error, and then predicts the largest DOF count
whose full frame (physics + static render + sync/dynamic render, the last
two taken as their mean over the variants) fits 60 FPS and 20 FPS.

    uv run python validation/complexity/analyze.py \
        --interactive results/interactive/*.json --out results/ --figures figures/
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from make_figures import INK, INK_2, SURFACE, _style  # noqa: E402

DEVICE_COLOURS = {"gpu": "#2a78d6", "cpu": "#eb6834"}


def load(paths: list[Path]) -> dict:
    """Group reports by (device, variant) and take medians over repeats."""

    groups: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for path in paths:
        report = json.loads(path.read_text())
        device = "gpu" if report["physics"]["gpu_dynamics"] else "cpu"
        groups[(device, Path(report["usd"]).stem)].append(report)
    rows = []
    for (device, variant), reports in sorted(groups.items()):
        def median(path):
            values = []
            for report in reports:
                value = report
                for key in path:
                    value = value[key]
                values.append(value)
            return float(np.median(values))

        model = reports[0]["model"]
        rows.append({
            "device": device,
            "variant": variant,
            "repeats": len(reports),
            "dofs": model["dofs"],
            "bodies": model["rigid_bodies"],
            "d6": model["joints"].get("d6", 0),
            "frame_idle_ms": median(("phases", "idle", "p50_ms")),
            "frame_push_ms": median(("phases", "push", "p50_ms")),
            "frame_push_p95_ms": median(("phases", "push", "p95_ms")),
            "physics_ms": median(("phases", "physics", "p50_ms")),
            "render_static_ms": median(("phases", "paused", "p50_ms")),
            "sync_dynamic_ms": median(("breakdown_ms_p50", "sync_and_dynamic_render")),
            "fps_idle": median(("phases", "idle", "fps_p50")),
            "spread_physics_pct": float(
                100 * (max(r["phases"]["physics"]["p50_ms"] for r in reports)
                       - min(r["phases"]["physics"]["p50_ms"] for r in reports))
                / np.median([r["phases"]["physics"]["p50_ms"] for r in reports])
            ) if len(reports) > 1 else None,
            "gpu_memory_mib": median(("memory", "gpu", "memory_used_mib")),
            "rss_mib": median(("memory", "rss_mib")),
        })
    return {"rows": rows, "hardware": next(iter(groups.values()))[0]["hardware"],
            "gpu": next(iter(groups.values()))[0]["memory"]["gpu"].get("name")}


def fit(rows: list[dict]) -> dict:
    """Least squares on DOF and bodies; DOF only if a coefficient is negative.

    DOFs and bodies are nearly collinear across the optimizer variants, so
    with noisy timings (GPU) the two effects cannot be separated.
    """

    result = _fit(rows, with_bodies=True)
    if result["ms_per_dof"] < 0 or result["ms_per_body"] < 0:
        result = _fit(rows, with_bodies=False)
    return result


def _fit(rows: list[dict], *, with_bodies: bool) -> dict:
    x = np.array([[1.0, r["dofs"], r["bodies"] if with_bodies else 0.0] for r in rows])
    y = np.array([r["physics_ms"] for r in rows])
    coef, *_ = np.linalg.lstsq(x, y, rcond=None)
    pred = x @ coef
    ss_res = float(((y - pred) ** 2).sum())
    ss_tot = float(((y - y.mean()) ** 2).sum())
    loo = []
    for i in range(len(rows)):
        mask = np.arange(len(rows)) != i
        c, *_ = np.linalg.lstsq(x[mask], y[mask], rcond=None)
        loo.append(abs(float(x[i] @ c) - y[i]) / y[i] * 100)
    return {
        "intercept_ms": float(coef[0]),
        "ms_per_dof": float(coef[1]),
        "ms_per_body": float(coef[2]),
        "r2": 1 - ss_res / ss_tot if ss_tot > 0 else None,
        "loo_mean_abs_pct": float(np.mean(loo)),
        "points": len(rows),
        "terms": "dof+bodies" if with_bodies else "dof",
    }


def budget(rows: list[dict], model: dict, frame_ms: float) -> dict:
    """Largest DOF count whose predicted frame fits ``frame_ms``.

    Bodies are tied to DOFs with the variants' own ratio (fit bodies ~ DOF).
    """

    overhead = float(np.mean([r["render_static_ms"] + r["sync_dynamic_ms"] for r in rows]))
    dofs = np.array([r["dofs"] for r in rows], dtype=float)
    bodies = np.array([r["bodies"] for r in rows], dtype=float)
    slope, intercept = np.polyfit(dofs, bodies, 1)
    per_dof = model["ms_per_dof"] + model["ms_per_body"] * slope
    fixed = model["intercept_ms"] + model["ms_per_body"] * intercept + overhead
    max_dof = (frame_ms - fixed) / per_dof if per_dof > 0 else float("inf")
    return {"frame_ms": frame_ms, "overhead_ms": overhead, "max_dofs": float(max_dof),
            "max_d6": float(max_dof / 2)}


def figure(rows: list[dict], fits: dict, out: Path, hardware: dict, gpu: str) -> Path:
    _style()
    fig, (left, right) = plt.subplots(1, 2, figsize=(13, 5.0))
    for device, colour in DEVICE_COLOURS.items():
        data = sorted((r for r in rows if r["device"] == device), key=lambda r: r["dofs"])
        if not data:
            continue
        xs = [r["dofs"] for r in data]
        left.scatter(xs, [r["physics_ms"] for r in data], s=70, color=colour, edgecolor=SURFACE,
                     linewidth=2, zorder=3, label=f"{device.upper()} dynamics")
        m = fits[device]
        grid = np.linspace(min(xs), max(xs), 50)
        bodies = np.polyval(np.polyfit(xs, [r["bodies"] for r in data], 1), grid)
        left.plot(grid, m["intercept_ms"] + m["ms_per_dof"] * grid + m["ms_per_body"] * bodies,
                  color=colour, lw=1.5, alpha=0.7)
        terms = (f"{m['intercept_ms']:.1f} ms + {m['ms_per_dof'] * 1e3:.0f} µs/DOF"
                 + (f" + {m['ms_per_body'] * 1e3:.0f} µs/body" if m["terms"] == "dof+bodies" else " (DOF only)"))
        left.annotate(f"{device.upper()}: {terms}, R² {m['r2']:.2f}", (xs[-1], data[-1]["physics_ms"]), xytext=(-8, 10),
                      textcoords="offset points", ha="right", fontsize=9, color=INK)
        bottom = np.zeros(len(data))
        width = 0.38
        offset = -width / 2 if device == "gpu" else width / 2
        positions = np.arange(len(data)) + offset
        for part, alpha in (("physics_ms", 1.0), ("render_static_ms", 0.55), ("sync_dynamic_ms", 0.3)):
            values = np.array([r[part] for r in data])
            right.bar(positions, values, width=width - 0.02, bottom=bottom, color=colour, alpha=alpha,
                      edgecolor=SURFACE, linewidth=1.5)
            bottom += values
        for x, total in zip(positions, bottom):
            right.text(x, total + 1, f"{1e3 / total:.0f} FPS", ha="center", fontsize=8, color=INK)
    labels = [f"{r['d6']} D6\n{r['bodies']} bodies" for r in sorted(
        (r for r in rows if r["device"] == "gpu"), key=lambda r: r["dofs"])]
    right.set_xticks(range(len(labels)), labels, fontsize=9)
    right.set_axisbelow(True)
    left.set_axisbelow(True)
    for limit, name in ((1e3 / 60, "60 FPS"), (50.0, "20 FPS")):
        right.axhline(limit, color=INK_2, lw=1, ls="--", zorder=1)
        right.text(-0.62, limit + 1, name, ha="left", fontsize=9, color=INK_2)
    left.set_title("Physics cost per step (60 Hz, physics-only)", loc="left")
    left.set_xlabel("articulation DOFs")
    left.set_ylabel("ms per physics step (median)")
    left.set_ylim(bottom=0)
    left.legend(loc="upper left")
    right.set_title("Interactive frame breakdown (GPU | CPU)", loc="left")
    right.set_ylabel("ms per frame (median, plant at rest)")
    fig.suptitle("What the plant model costs in the interactive regime (GUI, RTX, 60 Hz)",
                 x=0.01, y=0.985, ha="left", fontsize=14, fontweight="bold", color=INK)
    fig.text(0.01, 0.925, f"{hardware.get('cpu', '')}  |  {gpu}  |  TGS 32/4 iterations, 1600x900 viewport, "
             "medians of 2 runs", ha="left", fontsize=9, color=INK_2)
    fig.text(0.01, 0.895, "Bars, dark to light: physics, static render, sync + moving geometry. "
             "Blue = GPU dynamics, orange = CPU dynamics.", ha="left", fontsize=9, color=INK_2)
    fig.tight_layout(rect=(0, 0, 1, 0.87))
    path = out / "interactive_cost.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--interactive", type=Path, nargs="+", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--figures", type=Path, required=True)
    args = parser.parse_args(argv)
    data = load(args.interactive)
    rows = data["rows"]
    fits, budgets = {}, {}
    for device in sorted({r["device"] for r in rows}):
        subset = [r for r in rows if r["device"] == device]
        fits[device] = fit(subset)
        budgets[device] = {name: budget(subset, fits[device], ms)
                           for name, ms in (("60fps", 1e3 / 60), ("20fps", 50.0))}
    args.out.mkdir(parents=True, exist_ok=True)
    args.figures.mkdir(parents=True, exist_ok=True)
    summary = {"rows": rows, "fits": fits, "budgets": budgets, "hardware": data["hardware"], "gpu": data["gpu"]}
    (args.out / "interactive_cost_day160.json").write_text(json.dumps(summary, indent=1))
    print("| device | variant | D6 | bodies | physics ms | static render ms | sync+dyn ms | idle frame ms | FPS | push p95 ms | spread % |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        spread = f"{r['spread_physics_pct']:.0f}" if r["spread_physics_pct"] is not None else "-"
        print(f"| {r['device']} | {r['variant']} | {r['d6']} | {r['bodies']} | {r['physics_ms']:.1f} | "
              f"{r['render_static_ms']:.1f} | {r['sync_dynamic_ms']:.1f} | {r['frame_idle_ms']:.1f} | "
              f"{r['fps_idle']:.0f} | {r['frame_push_p95_ms']:.1f} | {spread} |")
    for device, m in fits.items():
        print(f"fit {device} [{m['terms']}]: {m['intercept_ms']:.2f} ms + {m['ms_per_dof'] * 1e3:.1f} us/DOF + "
              f"{m['ms_per_body'] * 1e3:.1f} us/body, R2 {m['r2']:.3f}, LOO {m['loo_mean_abs_pct']:.1f}%")
        for name, b in budgets[device].items():
            reach = (f"max {b['max_dofs']:.0f} DOF ({b['max_d6']:.0f} D6)" if b["max_dofs"] > 0
                     else "not reachable at any DOF count")
            print(f"  {name}: overhead {b['overhead_ms']:.1f} ms -> {reach}")
    print(figure(rows, fits, args.figures, data["hardware"], data["gpu"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
