"""Generate reproducible report figures from the aggregate validation JSON."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc, Circle, FancyArrowPatch, Rectangle


SCRIPT_DIR = Path(__file__).resolve().parent
RESULT_PATH = SCRIPT_DIR / "results" / "cantilever_validation_results.json"
ASSET_DIR = SCRIPT_DIR / "docs" / "assets"

BLUE = "#2364AA"
GREEN = "#2A9D6F"
ORANGE = "#E07A2D"
RED = "#C43D3D"
GRAY = "#59636E"
LIGHT = "#D8DEE5"


def matching(payload: dict[str, Any], **criteria: Any) -> list[dict[str, Any]]:
    rows = []
    for row in payload["measurements"]:
        if row.get("error"):
            continue
        if all(row.get(key) == value for key, value in criteria.items()):
            rows.append(row)
    return sorted(rows, key=lambda row: (row["n_links"], row["physics_hz"]))


def configure() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 120,
            "savefig.dpi": 180,
            "font.size": 10,
            "axes.titlesize": 13,
            "axes.labelsize": 10,
            "axes.edgecolor": GRAY,
            "axes.linewidth": 0.8,
            "axes.grid": True,
            "grid.color": LIGHT,
            "grid.linewidth": 0.7,
            "grid.alpha": 0.8,
            "legend.frameon": False,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
        }
    )


def save(fig: plt.Figure, name: str) -> None:
    fig.savefig(ASSET_DIR / name, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"[asset] {ASSET_DIR / name}")


def physical_model_figure() -> None:
    fig, ax = plt.subplots(figsize=(12, 6.4))
    ax.set_xlim(-0.6, 10.8)
    ax.set_ylim(-3.1, 3.2)
    ax.axis("off")

    ax.add_patch(Rectangle((-0.35, -1.35), 0.35, 2.7, color=GRAY))
    for y in np.linspace(-1.25, 1.05, 7):
        ax.plot([-0.55, -0.35], [y - 0.2, y], color=GRAY, lw=1)
    ax.text(-0.45, 1.7, "Fixed root", ha="center", weight="bold")

    link_y = 0.25
    link_count = 6
    link_width = 1.45
    gap = 0.08
    for index in range(link_count):
        x = index * (link_width + gap)
        ax.add_patch(
            Rectangle(
                (x, link_y - 0.18),
                link_width,
                0.36,
                facecolor="#DCEAF7",
                edgecolor=BLUE,
                linewidth=1.5,
            )
        )
        ax.text(x + link_width / 2, link_y, f"rigid link {index + 1}", ha="center", va="center", fontsize=8)
        if index:
            hinge_x = x - gap / 2
            ax.add_patch(Circle((hinge_x, link_y), 0.105, facecolor="white", edgecolor=ORANGE, lw=2))
            ax.add_patch(Arc((hinge_x, link_y), 0.55, 0.55, theta1=20, theta2=320, color=ORANGE, lw=1.5))

    tip_x = link_count * (link_width + gap) - gap
    ax.plot(tip_x, link_y, marker="o", color=RED, markersize=7)
    ax.add_patch(FancyArrowPatch((tip_x, 2.5), (tip_x, 0.55), arrowstyle="-|>", mutation_scale=18, color=RED, lw=2))
    ax.text(tip_x, 2.72, "0.05 N at geometric tip", ha="center", color=RED, weight="bold")

    for x in np.linspace(0.5, tip_x - 0.5, 8):
        ax.add_patch(FancyArrowPatch((x, -0.45), (x, -1.35), arrowstyle="-|>", mutation_scale=11, color=GREEN, lw=1.2))
    ax.text(tip_x / 2, -1.7, "Self-weight: distributed load w = rho A g", ha="center", color=GREEN, weight="bold")

    ax.annotate("", xy=(tip_x, -2.35), xytext=(0, -2.35), arrowprops={"arrowstyle": "<->", "color": GRAY})
    ax.text(tip_x / 2, -2.62, "Physical length L, divided into N cells of length l = L/N", ha="center", color=GRAY)

    ax.annotate(
        "D6 bending spring\nk = EI/l (per radian)\nUSD gain = k pi/180",
        xy=(3.02, link_y),
        xytext=(3.2, 1.65),
        arrowprops={"arrowstyle": "->", "color": ORANGE},
        ha="center",
        color=ORANGE,
        weight="bold",
    )
    ax.annotate(
        "Runtime measurement\nlocal endpoint transformed\nthrough final PhysX link pose",
        xy=(tip_x, link_y),
        xytext=(7.5, -0.75),
        arrowprops={"arrowstyle": "->", "color": RED},
        ha="center",
        color=RED,
    )
    ax.text(0, 3.0, "Discrete cantilever model and quantitative measurement protocol", fontsize=17, weight="bold")
    ax.text(0, 2.65, "Schematic, not to scale. Tip force and self-weight are separate scenarios.", color=GRAY)
    save(fig, "fig01_physical_model.png")


def spatial_convergence_figure(payload: dict[str, Any]) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.4), sharex=False)
    specs = [
        ("tip_force_0p05N", 240.0, "0.05 N tip force at 240 Hz"),
        ("self_weight", 1920.0, "Self-weight at 1920 Hz"),
    ]
    for ax, (scenario, rate, title) in zip(axes, specs):
        rows = matching(
            payload,
            benchmark="synthetic_solid_40cm",
            model="new_physics",
            support="fixed",
            joint_model="d6_biaxial",
            scenario=scenario,
            physics_hz=rate,
        )
        n = [row["n_links"] for row in rows]
        measured = [row["final_deflection_mm"] for row in rows]
        discrete = [row["expected_discrete_deflection_mm"] for row in rows]
        continuum = rows[0]["expected_deflection_mm"] if rows else np.nan
        ax.plot(n, measured, "o-", color=BLUE, lw=2, label="PhysX")
        ax.plot(n, discrete, "s--", color=ORANGE, lw=1.8, label="Exact discrete chain")
        ax.axhline(continuum, color=GREEN, lw=1.8, label="Continuum limit")
        ax.set_title(title, loc="left", weight="bold")
        ax.set_xlabel("Number of rigid links, N")
        ax.set_ylabel("Tip deflection (mm)")
        ax.set_xticks(n)
        ax.legend(loc="lower right")
    fig.suptitle("Synthetic benchmark: spatial refinement separates solver and discretization error", x=0.07, ha="left", fontsize=16, weight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    save(fig, "fig02_synthetic_spatial_convergence.png")


def timestep_figure(payload: dict[str, Any]) -> None:
    rows = matching(
        payload,
        benchmark="synthetic_solid_40cm",
        model="new_physics",
        support="fixed",
        joint_model="d6_biaxial",
        n_links=20,
        scenario="tip_force_0p05N",
    )
    hz = [row["physics_hz"] for row in rows]
    measured = [row["final_deflection_mm"] for row in rows]
    passed = [row.get("validation_status") == "passed" for row in rows]
    fig, ax = plt.subplots(figsize=(10.5, 5.5))
    ax.plot(hz, measured, color=BLUE, lw=2)
    for x, y, ok in zip(hz, measured, passed):
        ax.scatter(x, y, s=70, color=GREEN if ok else RED, edgecolor="white", linewidth=0.8, zorder=3)
        ax.annotate(f"{y:.3f}", (x, y), xytext=(0, 9), textcoords="offset points", ha="center", fontsize=8)
    if rows:
        ax.axhline(rows[0]["expected_discrete_deflection_mm"], color=ORANGE, ls="--", lw=1.8, label="Exact discrete chain")
        ax.axhline(rows[0]["expected_deflection_mm"], color=GREEN, ls=":", lw=1.8, label="Continuum limit")
    ax.set_xscale("log", base=2)
    ax.set_xticks(hz, labels=[f"{value:g}" for value in hz])
    ax.set_xlabel("Physics rate (Hz, logarithmic scale)")
    ax.set_ylabel("Tip deflection (mm)")
    fig.suptitle("Synthetic N20 timestep sensitivity", x=0.07, ha="left", fontsize=16, weight="bold")
    fig.text(0.07, 0.91, "TGS 32/4; red marker denotes a non-settled or physically rejected run", color=GRAY)
    ax.legend(loc="upper right")
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    save(fig, "fig03_timestep_sensitivity.png")


def pre_post_figure(payload: dict[str, Any]) -> None:
    ns = [3, 10, 20]
    legacy = []
    new = []
    for n_links in ns:
        legacy_rows = matching(
            payload,
            benchmark="synthetic_solid_40cm",
            model="legacy_current",
            support="fixed",
            joint_model="d6_biaxial",
            n_links=n_links,
            scenario="tip_force_0p05N",
            physics_hz=1920.0,
        )
        new_rows = matching(
            payload,
            benchmark="synthetic_solid_40cm",
            model="new_physics",
            support="fixed",
            joint_model="d6_biaxial",
            n_links=n_links,
            scenario="tip_force_0p05N",
            physics_hz=1920.0,
        )
        legacy.append(legacy_rows[0]["final_deflection_mm"] if legacy_rows else np.nan)
        new.append(new_rows[0]["final_deflection_mm"] if new_rows else np.nan)
    x = np.arange(len(ns))
    width = 0.35
    fig, ax = plt.subplots(figsize=(10.5, 5.5))
    legacy_bars = ax.bar(x - width / 2, legacy, width, color=GRAY, label="legacy_current (audited L = 0.004 m)")
    bars = ax.bar(x + width / 2, new, width, color=BLUE, label="new_physics (audited L = 0.400 m)")
    for bar, value in zip(bars, new):
        if np.isfinite(value):
            ax.text(bar.get_x() + bar.get_width() / 2, value + 0.035, f"{value:.3f}", ha="center", fontsize=9)
    for bar, value in zip(legacy_bars, legacy):
        if np.isfinite(value):
            ax.text(bar.get_x() + bar.get_width() / 2, 0.02, f"{value:.3f}", ha="center", color=GRAY, fontsize=9)
    ax.set_xticks(x, [f"N{n}" for n in ns])
    ax.set_ylabel("Measured tip deflection (mm)")
    fig.suptitle("Current legacy branch versus new physics", x=0.06, ha="left", fontsize=16, weight="bold")
    fig.text(0.06, 0.91, "Behavioral comparison only: the audited physical geometries are not equivalent", color=RED, weight="bold")
    ax.legend(loc="upper left")
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    save(fig, "fig04_pre_post.png")


def gao_figure(payload: dict[str, Any]) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.4))
    for ax, scenario, title in [
        (axes[0], "tip_force_0p05N", "0.05 N geometric-tip load"),
        (axes[1], "self_weight", "Self-weight"),
    ]:
        rows = matching(
            payload,
            benchmark="tomato_gao_20cm",
            model="new_physics",
            support="fixed",
            joint_model="d6_biaxial",
            scenario=scenario,
            physics_hz=1920.0,
        )
        n = [row["n_links"] for row in rows]
        ax.plot(n, [row["final_deflection_mm"] for row in rows], "o-", color=BLUE, lw=2, label="PhysX")
        ax.plot(n, [row["expected_discrete_deflection_mm"] for row in rows], "s--", color=ORANGE, lw=1.8, label="Exact discrete chain")
        if rows:
            ax.axhline(rows[0]["expected_deflection_mm"], color=GREEN, lw=1.8, label="Continuum limit")
        ax.set_xticks(n)
        ax.set_xlabel("Number of rigid links, N")
        ax.set_ylabel("Tip deflection (mm)")
        ax.set_title(title, loc="left", weight="bold")
        ax.legend(loc="lower right")
    fig.suptitle("Gao tomato-stalk parameter benchmark at 1920 Hz", x=0.07, ha="left", fontsize=16, weight="bold")
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    save(fig, "fig05_gao_convergence.png")


def main() -> int:
    configure()
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    with RESULT_PATH.open() as handle:
        payload = json.load(handle)
    physical_model_figure()
    spatial_convergence_figure(payload)
    timestep_figure(payload)
    pre_post_figure(payload)
    gao_figure(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
