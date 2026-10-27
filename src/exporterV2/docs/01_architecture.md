# Exporter architecture

The exporter turns a `plant_skeleton/1.0` graph into an OpenUSD stage for
Isaac Sim. Its main invariant: source adaptation, physics authoring and visual
authoring stay separate. Whatever writes the skeleton knows the source,
`skeleton_branches.py` knows the skeleton, `core/` knows nothing about either.

## Pipeline

1. Any source (outside this repository) writes `input/skeleton.json`.
2. `config.py` loads `config/physics.yaml` and `config/visuals.yaml`, fills
   defaults and rejects unknown keys or bad values.
3. `exporter.resolve_global_scale` sets `tree_config.GLOBAL_SCALE` (a number,
   or `auto` from the thinnest radius) and copies the simulation settings into
   `PhysicsRuntimeConfig`.
4. `skeleton_branches.build_branch_plan` groups nodes into chains and returns
   a `BranchPlan`: branch definitions in the generic BRANCHES format (one
   link spec per node, physics per segment kind, visual profile, parent and
   attach link), leaflet records and fruit records. It also computes the
   `v2-aesthetic` pose (pedicels tilted 56° and alternating, leaflets at their
   inclination) or keeps the skeleton frames (`canonical`).
5. `joint_budget.py` optionally locks or merges joints to fit the PhysX
   articulation limit.
6. `core/usd/stage.build_stage(branch_backend="skinned",
   skinning_visual_mode="segmented")` authors rigid links, colliders, D6
   joints with drives, organic segmented meshes and tomato bodies.
7. `visuals/leaves.py` and `visuals/trusses.py` add leaflets, truss material
   and curved pedicels.
8. `collisions.auto_filter_initial_overlaps` filters colliders overlapping at
   rest; PhysX scene settings are applied; `audit_stage` compares authored
   counts with the plan and the manifest is written.

## Authoring Boundaries

- `core/tree_config.py`: runtime constants (global scale, runtime physics
  settings, colours) that the core reads.
- `core/skinning/`: vegetative rigid links (`branch_physics.py`) and organic
  visuals (`visual_segmented.py`, `mesh.py`, `terminal_fork.py`,
  `leaf_blade.py`).
- `core/usd/terminal_bodies.py`: tomato spheres, breakable attachment
  joints for physical fruits, clearance checks and terminal collision filters.
- `core/usd/materials.py`: shared leaf, stem, truss and ripeness-bucketed
  fruit materials under `/World/Looks`.
- `core/optimizations/`: joint-budget techniques. It does not render.
- `visuals/README.md` maps every mesh generator to its config switch.

## Stage Structure

- `/World` is the default prim.
- `/World/Stem` is the articulation root; it carries `autotom:skeletonSchema`,
  `autotom:fruitMode` and `autotom:globalScale`, which `isaac_app.py` reads.
- `/World/Stem/Vegetative` contains rigid links and collision capsules.
  Each link records its source node in `autotom:skeletonNodeId`.
- `/World/PlantVisual` contains visual-axis roots when the visual mode needs them.
- `/World/TerminalBodies` contains physical tomatoes, excluded from the
  articulation so they can detach.
- `/World/Looks` contains shared materials.

## Naming

Branch ids follow fixed patterns because the optimizer and the terminal-fork
code match on them: `trunk`, `Branch_<node>`, `Leaf_<base>_petiole`,
`Leaf_<base>_rachis`, `Truss_<node>_rachis`, `Truss_<node>_pedicel_NN`.
Keep these patterns when changing `skeleton_branches._branch_ids`.
