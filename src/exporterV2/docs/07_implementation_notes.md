# Implementation Notes

## Compatibility Rules

- Treat branch identifiers, prim paths, topology, transforms, joint frames, material bindings and collision relationships as stable output. The default day-160 plant (216 bodies, 206 D6, 10 fixed) is covered by a regression test.
- Keep generated USD/JSON artifacts out of source-only refactors.
- User-facing values belong in `config/physics.yaml` and `config/visuals.yaml` (defaults in `config.py`). `core/tree_config.py` holds the core's internal constants and receives the physics values at export time.

## Pipeline

1. `plant_skeleton.load_skeleton` reads and validates the `plant_skeleton/1.0` JSON.
2. `config.py` loads the two YAML files and rejects unknown keys.
3. `skeleton_branches.py` turns skeleton segments into branch chains, leaf visuals and terminal bodies (the `BranchPlan`).
4. `exporter.py` checks the joint budget, prepares leaf shapes, calls `core/usd/stage.build_stage`, authors leaf and truss visuals (`visuals/`), filters initial overlaps, audits the stage and writes the manifest.

## Module Responsibilities

- `stage.py` validates inputs, resolves branch transforms and coordinates authoring. The exporter always uses the skinned vegetative backend with segmented visuals.
- `branch_chains.py` owns rigid chain geometry, mass, drive gains, joints and link colors.
- `terminal_bodies.py` owns terminal authoring, curved pedicels, detachment, clearance checks and terminal filters.
- `materials.py` creates shared PreviewSurface materials and optional OmniSurface look-development presets.
- `skinning/builder.py` coordinates vegetative physics, visual axes, leaf blades and the visual mode.
- `visual_segmented.py` is the production visual; `visual_static.py`, `visual_rigid.py` and the UsdSkel code in `mesh.py` remain in the core but are not reachable from the exporter configuration. `mesh.py` also owns common radius sampling.
- `mesh_geometry.py` contains topology helpers shared by procedural tube meshes.
- `tests/legacy_csv/` is the old CSV builder, kept only as a fixture for core tests.

## Design Decisions

- Vegetative and truss systems share a stage but keep separate authoring paths.
- Detachable tomatoes are moved into USD terminal-body space only during stage authoring; the biological hierarchy remains in branch metadata.
- Leaf blades are separate visual meshes without rigid bodies, colliders or joints; leaflet mass from the skeleton is added to the host support.
- Terminal visual variation is deterministic from branch identifiers.

## Verification

Source refactors should compare the day-160 USDA and manifest before and after, then run the test suite listed in [06_testing.md](06_testing.md). Isaac-dependent checks must use `~/isaacsim/python.sh`.
