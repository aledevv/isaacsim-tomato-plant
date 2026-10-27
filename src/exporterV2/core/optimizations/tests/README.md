# Joint-budget optimizer tests

Run from the project root:

```bash
uv run pytest -q src/exporterV2/core/optimizations/tests
```

| Folder / file | Technique or part |
| --- | --- |
| `1_infrastructure/` | `BudgetOptimizer`, joint counting, configuration |
| `2_collision/` | AABB and sphere checks used to keep locked links apart |
| `3_geometry/` | remapping attachments after a merge |
| `4_petiole_lock/` | locking petiolule joints |
| `5_lateral_reduce/` | reducing lateral leaves |
| `8_leaf_branch_reduce/` | merging leaf branches |
| `9_integration/` | techniques combined, on CSV fixture plants |
| `10_thin_link_lock/` | locking the thinnest links |
| `11_truss_static/` | static trusses, also on an authored USD stage |
| `test_integration.py`, `test_cli_integration.py`, `test_rigid_trunk.py` | end-to-end optimizer runs |

The plants come from the CSV fixture builder in `src/exporterV2/tests/legacy_csv/`
(`data/simulation_output/dynamic_output/graphs/` inside it). The optimizer is used by the
exporter through `joints.optimize` in `config/physics.yaml`; the technique
reports are in `../docs/`.
