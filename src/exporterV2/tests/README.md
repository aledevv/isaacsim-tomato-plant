# exporterV2 tests

Run from the project root:

```bash
uv run pytest -q src/exporterV2/tests
```

| File | What it checks |
| --- | --- |
| `test_exporter.py` | End-to-end exports: the hand-written small plant, fruit modes, scale, joint budget, CLI, baked leaflets and droop, stem colour, branch collisions, and the day-160 regression against `data/day160_reference.json` |
| `test_config.py` | The YAML files only use keys the code knows; bad settings are rejected |
| `test_skeleton_branches.py` | Chain rules, attachment, pose, config switches |
| `test_baked_leaf_assets.py` | The committed leaflet set is complete and references at scale |
| `test_joint_budget*.py`, `test_body_merge.py` | Joint locking and link merging |
| `test_collision_geometry.py` | Collider placement |
| `legacy_csv/` | An old CSV plant builder and its GroIMP CSVs, used as a fixture by the optimizer, skinning, USD and collision tests; `legacy_csv/tests/` checks the fixture itself |

The complexity profile, the performance benchmark and the cantilever bending
study live in `validation/` at the repository root.

The Isaac Sim GUI (dragging, bending, fruit detachment) is checked by hand
with `./run_main.sh`.
