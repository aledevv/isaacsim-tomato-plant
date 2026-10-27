# exporterV2 tests

Run from the project root:

```bash
uv run pytest -q src/exporterV2/tests
```

| File | What it checks |
| --- | --- |
| `test_exporter.py` | End-to-end exports: the hand-written small plant, fruit modes, scale, joint budget, CLI, baked leaflets and droop, stem colour, branch collisions, and the day-160 regression against `data/day160_reference.json` |
| `test_config.py` | The commented YAML files equal the built-in defaults; bad settings are rejected |
| `test_skeleton_branches.py` | Chain rules, attachment, pose, config switches |
| `test_baked_leaf_assets.py` | The committed leaflet set is complete and references at scale |
| `test_joint_budget*.py`, `test_body_merge.py` | Joint locking and link merging |
| `test_collision_geometry.py` | Collider placement |
| other `test_*.py` | Diagnostic helpers used with Isaac Sim (fruit interaction, pacing, watchdog, benchmarks) |

`cantilever_bending_experiment/` is a stand-alone bending study with its own
report; its fast checks also run with pytest.

The Isaac Sim GUI (dragging, bending, fruit detachment) is checked by hand
with `./run_main.sh`.
