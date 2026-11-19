# Complexity study

How heavy is the plant to simulate, and what does the joint optimizer cost in
fidelity? The study keeps three levels apart and links them with measurements
instead of a single complexity score:

| Level | Question | Depends on the hardware? | Tool |
| --- | --- | --- | --- |
| A. Plant | how complex is the plant? | no | `complexity_profile.py` |
| B. Model | how heavy is the built model (joints, DOF, bodies, colliders)? | no | `complexity_profile.py` |
| C. Cost | how long does it take on a given PC? | yes | `interactive_bench.py`, `performance_benchmark.py` |

B and C are linked by a linear cost model fitted by `analyze.py`.

## Tools

| Script | What it does | Needs Isaac Sim |
| --- | --- | --- |
| `make_variants.py` | Exports the reference and the optimizer variants (budgets 120 / 60 / 40, any compliance policy). Tomatoes are visual, so the reference has 206 D6 joints and 216 bodies. | no |
| `fidelity_bench.py run` | Static test: settles a plant 5 s under gravity (480 Hz, headless) and records the position of every visual mesh. | yes |
| `fidelity_bench.py compare` | Distance of every mesh from the same mesh in the reference. Meshes are matched by a key that survives link merging. | no |
| `push_bench.py` | Dynamic test: all plants side by side, the same horizontal push for 0.2 s on the tip of the longest side branch, a truss tip and a leaflet; force = weight of the part beyond the pushed point. Peak displacement and oscillation frequency over 4 s. `--gui` shows the push. | yes |
| `interactive_bench.py` | Cost test in the GUI: RTX viewport 1600x900, physics 60 Hz, no frame cap; 600 frames at rest, pushed, paused (render only) and physics only. `--device cpu|gpu`. | yes |
| `analyze.py` | Fits `physics ms = a + b DOF + c bodies` per device and writes the cost table and figure. | no |
| `make_figures.py` | Push trajectories, cost against fidelity, and the plant drawn at each joint budget. | no |
| `performance_benchmark.py` | Steps/s and render updates/s of two stages at the same physics rates. | yes |

## Results

`results/` holds the data of the study (Isaac Sim 4.5, RTX 4080 Laptop 12 GB,
i9-13900HK). It was measured with the development version of the exporter;
the variants of this repository have the same joints and bodies
(206/216, 116/191, 60/135, 40/115), checked with `complexity_profile.py`.
The file names keep the day-160 label of the run; the plant structure is the
same at day 120.

- Articulation limit: 255 links (bodies) per articulation. 256 or more crash
  on GPU (CUDA error 700) and on CPU (segmentation fault). The exporter
  refuses such models.
- Static fidelity at 40 D6 (`results/fidelity_day160.json`, mean / max mesh
  error): load 3.0 / 10.7 mm, blend 5.1 mm, keep 7.7 / 30.6 mm, tip 9.4 mm,
  series 37.6 / 96.1 mm. Reference sag: median 15.2 mm, max 53.3 mm.
- Dynamic fidelity at 40 D6 (`results/push_day160.json`, error on peak /
  frequency): tip 18% / 11%, blend 29% / 1.3%, series 33% / 38%,
  load 43% / 14.5%, keep 70% / 58%.
- Interactive cost (`results/interactive/`, medians of 2 runs, TGS 32/4):
  CPU 53 FPS at 206 D6 and 78 FPS at 40 D6; GPU 12 and 25 FPS.
  Cost model: CPU 0.1 ms + 2.8 us/DOF + 41 us/body (R2 0.995);
  GPU 16.6 ms + 122 us/DOF (R2 0.998).

Known limits: the fidelity tests ran with GPU physics (TGS) while the
interactive scene uses CPU PGS; one plant and one PC; the compliance
policies are heuristics.

## Run

```bash
# variants (no Isaac Sim)
uv run python validation/complexity/make_variants.py --out output/complexity \
  --policies load,tip,blend,keep,series

# static fidelity
for s in ref b40_load b40_blend; do
  ~/isaacsim/python.sh validation/complexity/fidelity_bench.py run \
    --usd output/complexity/$s.usda --output output/complexity/$s.fidelity.json
done
uv run python validation/complexity/fidelity_bench.py compare \
  --reference output/complexity/ref.fidelity.json \
  b40_load=output/complexity/b40_load.fidelity.json \
  b40_blend=output/complexity/b40_blend.fidelity.json \
  --output output/complexity/fidelity_compare.json

# dynamic fidelity (first stage is the reference)
~/isaacsim/python.sh validation/complexity/push_bench.py --settle 6 \
  --stage ref=output/complexity/ref.usda \
  --stage b40_load=output/complexity/b40_load.usda \
  --stage b40_blend=output/complexity/b40_blend.usda \
  --output output/complexity/push.json

# interactive cost (TGS 32/4 as in the study): every variant, CPU and GPU,
# twice, in alternating order; analyze.py groups the reports by USD name and device
for r in 1 2; do for d in cpu gpu; do for s in ref b120_blend b60_blend b40_blend; do
  ~/isaacsim/python.sh validation/complexity/interactive_bench.py --solver TGS --device $d \
    --usd output/complexity/$s.usda --output output/complexity/interactive/${s}_${d}_r$r.json
done; done; done

# cost model and figures from the saved data (no Isaac Sim)
uv run --with matplotlib python validation/complexity/analyze.py \
  --interactive validation/complexity/results/interactive/*.json \
  --out output/complexity --figures output/complexity/figures
uv run --with matplotlib python validation/complexity/make_figures.py \
  --push validation/complexity/results/push_day160.json \
  --fidelity validation/complexity/results/fidelity_day160.json \
  --skeleton ref=output/complexity/ref.usda --skeleton b40_blend=output/complexity/b40_blend.usda \
  --out output/complexity/figures
```
