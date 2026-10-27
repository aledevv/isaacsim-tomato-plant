# exporterV2

Turns a plant skeleton into an articulated OpenUSD plant for Isaac Sim:
rigid links joined by bending D6 joints, organic meshes, baked 3D leaflets and
tomatoes.

```text
input/skeleton.json ─┐
config/physics.yaml ─┼─> exporterV2 ─> output/plant.usda (+ .manifest.json) ─> Isaac Sim
config/visuals.yaml ─┘
```

## Quick start

```bash
./run_main.sh                       # build output/plant.usda and open it in Isaac Sim
./run_main.sh --generate-only       # build only, no Isaac Sim
./run_main.sh --headless            # 5 s physics check without GUI
./run_main.sh --input input/examples/small_plant.json
```

Or without the launcher:

```bash
uv run python -m exporterV2 [--input PATH] [--physics PATH] [--visuals PATH] [--output PATH]
```

It prints a one-line summary, for example
`[OK] 216 segments -> 216 rigid bodies, 206 D6 joints, 131 leaflets, 72 fruits (scale 2)`.

## What to edit

| I want to... | Edit |
| --- | --- |
| Load a different plant | `input/skeleton.json` ([format](../plant_skeleton/README.md)) |
| Convert another program's plant | write `plant_skeleton/1.0` ([guide](../plant_skeleton/README.md#converting-another-source)) |
| Change stiffness, damping, mass, joint limits, fruit physics, collisions, scale | `config/physics.yaml` |
| Change mesh density, leaflets and their droop, stem colour, decorations, pose | `config/visuals.yaml` |
| Change how a mesh is generated | the generator listed in [visuals/README.md](visuals/README.md) |

Both YAML files list every option with its default and an explanation.
Every key is optional, and unknown keys are rejected so typos are caught.

## Pipeline

`exporter.py` (`export_plant`) runs these steps:

1. **Load** the skeleton (`plant_skeleton.load_skeleton`) and both configs (`config.py`).
2. **Scale**: `global_scale` (a number, or `auto`) multiplies every length.
   PhysX needs bodies at least about 2 mm thick; the example tomato uses 2.
3. **Branches** (`skeleton_branches.py`): nodes are grouped into chains
   (trunk, side shoots, leaves, truss rachis, pedicels). Each chain becomes
   one branch with one rigid link per node, its physics block from
   `segments.<kind>`, and the leaflet and fruit records hanging from it. The
   default `v2-aesthetic` pose spreads leaflets and pedicels at fixed angles;
   `canonical` uses the skeleton frames.
4. **Joint budget** (`joint_budget.py`, optional): above about 220 D6 joints,
   `joints.optimize: lock` turns the least important joints into fixed
   joints and `full` also merges links.
5. **USD stage** (`core/usd/stage.py`): rigid bodies, capsule colliders,
   D6 joints with drives computed from Young's modulus and radius, organic
   segmented meshes, tomato spheres (visual or physical).
6. **Dressing** (`visuals/leaves.py`, `visuals/trusses.py`, `visuals/fruits.py`,
   `visuals/stems.py`): leaflet stalks and blades (with `leaves.generator:
   baked`, a reference to a baked 3D leaflet from `leaf_shapes/baked.py`,
   tilted by `leaves.droop_deg`), truss material, gravity-curved pedicels,
   calyces, stem colour.
7. **Collisions** (`collisions.py`): colliders that already overlap at rest
   are filtered pairwise (or the export stops, with `collisions.initial_overlaps: error`);
   with `collisions.branches: true`, articulation self-collision is turned on
   and filtered so that only distant branches, petioles and leaf rachides touch.
8. **Scene settings** and an **audit** that compares the authored counts with
   the plan; the manifest JSON records counts, poses, masses and settings.

`isaac_app.py` opens the USDA in Isaac Sim with the physics rates stored in
the stage and the interactive drag tools; `run_main.sh` calls it.

## Folder map

```text
exporterV2/
├── cli.py, __main__.py   command line (defaults: input/skeleton.json, config/*.yaml, output/plant.usda)
├── config.py             loads and checks config/physics.yaml and config/visuals.yaml
├── exporter.py           the pipeline above
├── skeleton_branches.py  skeleton nodes -> branch chains, poses, leaflet and fruit records
├── joint_budget*.py      optional joint reduction
├── collisions.py         initial-overlap filter, branch-branch contacts
├── visuals/              leaflets, trusses, calyces, stem colour; README maps every mesh generator
├── leaf_shapes/          baked leaflets (baked.py) and per-leaflet seeds (seeds.py)
├── core/                 generic builder: tree_config, usd/, skinning/, optimizations/
├── isaac_app.py          Isaac Sim launcher and interaction tools
├── tests/                tests; tests/legacy_csv/ is an old CSV plant builder kept as a fixture
└── docs/                 technical notes: physics, collisions, visuals, leaflets
```

Studies that are not part of the pipeline (complexity profile, performance
benchmark, cantilever bending validation) are in
[`validation/`](../../validation/README.md) at the repository root.

## Tests

Run the suite by hand before merging.

```bash
uv run pytest -q \
  src/plant_skeleton/tests \
  src/exporterV2/tests \
  src/exporterV2/core/optimizations/tests \
  src/exporterV2/core/skinning/tests
uv run pytest -q src/exporterV2/core/usd/tests
uv run pytest -q validation
```

Key tests:

- `tests/test_exporter.py`: the hand-written small plant, fruit modes,
  scale, joint budget, CLI, baked leaflets and droop, stem colour, branch
  collisions, and the **day-160 regression**: the default skeleton must give
  the same bodies, joints, poses, masses and drive gains as the reference
  export (`tests/data/day160_reference.json`).
- `tests/test_skeleton_branches.py`: chain rules, attachment, pose, config switches.
- `tests/test_config.py`: the commented YAML files equal the built-in defaults.

The Isaac Sim GUI (dragging, bending, fruit detachment) can only be checked by
hand on a machine with Isaac Sim: `./run_main.sh`.

## Known limits

- Apart from the Young's modulus (50.64 MPa, Gao et al. 2024), physics
  parameters are tuned by eye, not calibrated on a real plant.
- Physical tomatoes (`fruits.mode: physical`) are stable only on trusses of
  the main stem; side trusses sag and can drop fruit.
- Fruits are spheres only.
- One plant per scene has been tested.
