# Testing

Run the suite by hand before merging:

```bash
uv run pytest -q \
  src/plant_skeleton/tests \
  src/exporterV2/tests \
  src/exporterV2/core/optimizations/tests \
  src/exporterV2/core/skinning/tests
uv run pytest -q src/exporterV2/core/usd/tests
uv run pytest -q validation
```

## What the main tests cover

| Test | Covers |
| --- | --- |
| `src/plant_skeleton/tests/test_skeleton.py` | format, JSON round trip, validation messages |
| `tests/test_skeleton_branches.py` | chain rules, attachment, physics per kind, visual switches, poses |
| `tests/test_config.py` | YAML files only use known keys, bad values rejected, `global_scale: auto` |
| `tests/test_exporter.py` | small plant end to end, fruit modes, scale, joint budget, CLI, baked leaflets and droop, stem colour, branch collisions, day-160 regression |
| `tests/test_baked_leaf_assets.py` | the committed baked leaflet set |
| `tests/test_joint_budget*.py` | lock and merge on the day-160 plant |
| `tests/legacy_csv/tests`, `core/*/tests` | generic core builder, using the legacy CSV builder as a fixture |

(`tests/` in the table means `src/exporterV2/tests/`.)

The day-160 regression compares against
`src/exporterV2/tests/data/day160_reference.json`, recorded from the earlier
exporter at 30 MPa (the test pins that modulus): same bodies (216), D6 (206) and fixed (10)
joints, capsules, meshes, link poses, masses and drive gains. If a deliberate
change alters them, regenerate the reference and say why in the commit.

## USD and PhysX tests under Isaac Python

Tests that need `PhysxSchema` run with Isaac Sim's Python:

```bash
~/isaacsim/python.sh -m pytest src/exporterV2/core/usd/tests -v
```

## Manual checks in Isaac Sim

```bash
./run_main.sh                    # GUI: Shift+drag a branch, a leaf and a truss
./run_main.sh --headless         # 5 s physics run, prints the stability check
```

## Validation studies

`validation/` holds the studies that are not part of the pipeline: the
complexity profile and performance benchmark (`validation/complexity/`) and the
cantilever bending validation (`validation/cantilever_bending/`). Their fast
checks run with `uv run pytest -q validation`. See
[`validation/README.md`](../../../validation/README.md).

```bash
uv run python validation/complexity/performance_benchmark.py \
  --baseline before=/tmp/before.usda \
  --candidate after=/tmp/after.usda \
  --physics-hz 60,120,240,480 \
  --output /tmp/performance.json
```

One clean Isaac worker runs per rate; both stages load in the same process.
