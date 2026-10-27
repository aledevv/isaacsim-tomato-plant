# Validation

Studies that check or measure the model. They are not part of the pipeline:
`run_main.sh` and the exporter never import them.

| Folder | What it is |
| --- | --- |
| [`complexity/`](complexity/) | `complexity_profile.py` counts the plant (organs, segments), the authored USD model (bodies, joints, colliders, meshes) and the solver settings, so that a measured cost can be linked to counts. `performance_benchmark.py` measures render updates/s and PhysX steps/s of two USD stages at the same physics rates in Isaac Sim. |
| [`cantilever_bending/`](cantilever_bending/) | Cantilever bending of the articulated branch model against beam theory and the tomato stalk data of Gao et al. 2024; the evidence for the 50.64 MPa Young's modulus. Report: [`docs/CantileverValidationReport.md`](cantilever_bending/docs/CantileverValidationReport.md). |

## Run

From the repository root:

```bash
uv run pytest -q validation          # fast checks of both studies

uv run python validation/complexity/complexity_profile.py \
  output/plant.usda --skeleton input/skeleton.json

uv run python validation/complexity/performance_benchmark.py \
  --baseline before=/tmp/before.usda \
  --candidate after=/tmp/after.usda \
  --physics-hz 60,120,240,480 \
  --output /tmp/performance.json

./validation/cantilever_bending/run_paper_experiment.sh   # needs Isaac Sim
```

The benchmark and the cantilever simulations start Isaac Sim (`~/isaacsim`).
Generated cantilever USD files go to `cantilever_bending/usd_models/` (not
tracked).
