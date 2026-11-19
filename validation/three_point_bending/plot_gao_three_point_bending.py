"""Figures of the three-point bending test.

    uv run python validation/three_point_bending/plot_gao_three_point_bending.py \
        validation/three_point_bending/results/gao_three_point_bending.json validation/three_point_bending/figures \
    [output/gao/ramp_n32.json]
"""
import json, math, sys
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

out = Path(sys.argv[2]); out.mkdir(parents=True, exist_ok=True)
R = json.load(open(sys.argv[1]))
SURF, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
RAMP = {480: "#86b6ef", 1920: "#3987e5", 3840: "#1c5cab", 7680: "#0d366b"}
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "axes.edgecolor": INK2,
    "axes.labelcolor": INK, "xtick.color": INK2, "ytick.color": INK2, "axes.facecolor": SURF,
    "figure.facecolor": SURF, "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8, "lines.linewidth": 2})
GAO, GAO_SD = 50.64, 2.42

def exact_chain(n, span=0.057):
    I = R["hz1920"]["specimen"]["I_mm4"] * 1e-12; EI = GAO * 1e6 * I; a = span / 2; l = 0.08 / n
    c = sum((a - abs(-0.04 + j * l)) ** 2 / 4 / (EI / l) for j in range(1, n) if abs(-0.04 + j * l) < a)
    return span ** 3 / (48 * I * c) / 1e6

# 1. E backed out vs number of links
fig, ax = plt.subplots(figsize=(6.4, 4.0))
ax.axhspan(GAO - GAO_SD, GAO + GAO_SD, color="#ecebe7", zorder=0)
ax.axhline(GAO, color=INK2, lw=1, zorder=1)
ax.text(2.05, GAO + GAO_SD + 0.6, "Gao et al. 50.64 ± 2.42 MPa", color=INK2, fontsize=9)
ns = [2, 4, 8, 16, 32]
ax.plot(ns, [exact_chain(n) for n in ns], ls="--", color=INK2, lw=1.5, marker="o", ms=4, label="exact chain (no solver error)")
for hz in (480, 1920, 3840, 7680):
    cases = R[f"hz{hz}"]["cases"]
    pts = sorted((c["n_links"], c["young_backed_out_MPa"]) for c in cases)
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color=RAMP[hz], marker="o", ms=6,
            markeredgecolor=SURF, markeredgewidth=1.5, label=f"PhysX {hz} Hz")
ax.set_xscale("log", base=2); ax.set_xticks(ns); ax.set_xticklabels([f"{n}\n({80/n:g} mm)" for n in ns])
ax.set_xlabel("links in the 80 mm specimen (link length)"); ax.set_ylabel("E backed out with Gao's formula (MPa)")
ax.set_ylim(15, 58); ax.legend(frameon=False, fontsize=8.5, loc="lower right")
ax.set_title("Gao three-point bending in PhysX, input E = 50.64 MPa", loc="left", fontsize=11, color=INK)
fig.tight_layout(); fig.savefig(out / "gao_E_vs_links.png", dpi=200); plt.close(fig)

# 2. Force-deflection line, N=32 at 7680 Hz, with the slow ramp
case = [c for c in R["hz7680"]["cases"] if c["n_links"] == 32][0]
# Optional third argument: the --record output of gao_three_point_bending.py.
ramp = json.load(open(sys.argv[3]))["frames"] if len(sys.argv) > 3 else []
ramp = [f for f in ramp if f["t"] >= 1.0]
fig, ax = plt.subplots(figsize=(6.4, 4.0))
per_n = case["beam_theory_deflection_per_N_mm"]
for e, style in ((GAO - GAO_SD, ":"), (GAO + GAO_SD, ":")):
    ax.plot([0, 4.2 * per_n * GAO / e], [0, 4.2], color=INK2, lw=1, ls=style)
ax.plot([0, 4.2 * per_n], [0, 4.2], color=INK2, lw=1.5, label="beam theory, E = 50.64 MPa (dotted: ± 1 SD)")
if ramp:
    ax.plot([f["deflection_mm"] for f in ramp], [f["force_N"] for f in ramp], color=RAMP[1920], lw=2, label="PhysX, slow load ramp")
rows = case["rows"]
ax.plot([r["deflection_mm"] for r in rows], [r["force_N"] for r in rows], ls="none", marker="o", ms=7,
        color=RAMP[7680], markeredgecolor=SURF, markeredgewidth=1.5, label="PhysX, held loads (fit: 49.3 MPa)")
ax.set_xlabel("midpoint deflection S (mm)"); ax.set_ylabel("load F (N)")
ax.set_xlim(0, 0.5); ax.set_ylim(0, 4.3); ax.legend(frameon=False, fontsize=8.5, loc="upper left")
ax.set_title("Force-deflection, 32 links, 7680 Hz, span 57 mm", loc="left", fontsize=11, color=INK)
fig.tight_layout(); fig.savefig(out / "gao_force_deflection.png", dpi=200); plt.close(fig)

# 3. Main-scene side shoots vs physics rate
fig, ax = plt.subplots(figsize=(5.2, 3.6))
rates = [60, 240, 960]
vals = [sum(v["stiffness_N_per_m"] for v in R[f"main_scene_side_shoots_{hz}hz"].values()) / 4 for hz in rates]
ax.bar([str(r) for r in rates], vals, width=0.55, color=RAMP[1920], zorder=2)
ax.axhline(59.9, color=INK2, lw=1.5, ls="--", zorder=3)
ax.text(2.3, 62.5, "authored joints (hand calculation)", color=INK2, fontsize=8.5, ha="right")
for i, v in enumerate(vals):
    ax.text(i, v - 4, f"{v:.0f}", ha="center", color=SURF, fontsize=10, fontweight="bold", zorder=4)
ax.set_xlabel("physics rate (Hz)"); ax.set_ylabel("side-shoot tip stiffness (N/m)"); ax.set_ylim(0, 68)
ax.grid(axis="x", visible=False)
ax.set_title("Main scene side shoots, E = 50.64 MPa", loc="left", fontsize=11, color=INK)
fig.tight_layout(); fig.savefig(out / "scene_rate_stiffness.png", dpi=200); plt.close(fig)
print("ok")
