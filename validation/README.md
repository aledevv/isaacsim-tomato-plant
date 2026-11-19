# Validation

Studies that check or measure the model. They are not part of the pipeline:
`run_main.sh` and the exporter never import them.

| Folder | What it is |
| --- | --- |
| [`complexity/`](complexity/) | Complexity study: counts of the plant and of the USD model (`complexity_profile.py`), the optimizer variants, their static and dynamic fidelity against the full plant, the interactive cost on CPU and GPU, and the fitted cost model. Saved results in `complexity/results/`. See [`complexity/README.md`](complexity/README.md). |
| [`three_point_bending/`](three_point_bending/) | The three-point bending test of Gao et al. 2024 rebuilt in PhysX with the exporter's joint formula: does the test give back E = 50.64 MPa? Saved results and figures script. |
| [`branch_stiffness/`](branch_stiffness/) | Side-shoot tip stiffness of the exported plant at several physics rates, and a hand calculation of the rest bend of every D6 joint against its limit. |
| [`cantilever_bending/`](cantilever_bending/) | Cantilever bending of the articulated branch model against beam theory and the tomato stalk data of Gao et al. 2024; the evidence for the 50.64 MPa Young's modulus. Report: [`docs/CantileverValidationReport.md`](cantilever_bending/docs/CantileverValidationReport.md). |

## Run

From the repository root:

```bash
uv run pytest -q validation          # fast checks of the studies

uv run python validation/complexity/complexity_profile.py \
  output/plant.usda --skeleton input/skeleton.json

uv run python validation/complexity/performance_benchmark.py \
  --baseline before=/tmp/before.usda \
  --candidate after=/tmp/after.usda \
  --physics-hz 60,120,240,480 \
  --output /tmp/performance.json

./validation/cantilever_bending/run_paper_experiment.sh   # needs Isaac Sim
uv run python validation/branch_stiffness/rest_angles.py output/plant.usda
```

The other studies have their commands in their own README. Figures need
matplotlib, which is not a dependency of the exporter: run those scripts
with `uv run --with matplotlib python ...`.

The benchmark and the cantilever simulations start Isaac Sim (`~/isaacsim`).
Generated cantilever USD files go to `cantilever_bending/usd_models/` (not
tracked).
