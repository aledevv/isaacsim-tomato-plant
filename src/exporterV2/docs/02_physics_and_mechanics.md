# Physics And Mechanics

The exporter represents plant axes as rigid links connected by Fixed or D6 joints. All user-facing values live in `config/physics.yaml`; `exporter.py` (`apply_physics_config`) copies them into the core's global configuration (`core/tree_config.py`) before the stage is built. Documentation does not copy the numbers; read the commented YAML file.

## Segment Mechanics

Each skeleton segment kind (`stem`, `branch`, `petiole`, `leaf_rachis`, `truss_rachis`, `pedicel`) has its own block under `segments:` with joint type, Young's modulus, damping ratio, density, bend limit and a drive stiffness multiplier. `skeleton_branches.py` groups segments into branch chains and attaches these settings.

Branch-chain authoring is centralized in `core/usd/branch_chains.py`. Link mass is derived from geometry and density. Flexible drive stiffness follows beam-style flexural rigidity and attachment compliance; damping follows the configured damping ratio and the link inertia.

Organic stem meshes, leaf blades, terminal fork dressing and junction bulges (`config/visuals.yaml`) are visual-only. They change how the plant looks, not its rigid links, joints or colliders.

## Trusses And Pedicels

Truss rachides and pedicels are articulated chains like the other axes, using the `truss_rachis` and `pedicel` blocks. They are much stiffer and more damped than the vegetative organs so the trusses stay stable under fruit load.

The gravity-curved pedicel (`pedicels.gravity_curve`) is a child mesh of the straight physical proxy. Its tip places the visual tomato and the fixed-joint frames consistently, while the proxy topology and collision shape stay unchanged.

## Tomatoes

`fruits.mode` selects how tomatoes are exported:

- `none`: no tomatoes;
- `visual` (default): tomatoes are drawn but have no mass;
- `physical` (experimental): tomatoes are rigid bodies; side trusses can be unstable.

With `physical` and `fruits.detachable: true`, a tomato:

- is authored as a rigid body outside the main articulation;
- is connected to its pedicel by a FixedJoint that breaks above `fruits.break_force_n`;
- uses `simulation.fruit_solver_position_iterations` / `fruit_solver_velocity_iterations`.

The stage does not replace the configured break force with an artificial holding value.

## Joint Budget

One articulation in Isaac Sim 4.5 handles about 220 D6 joints well and at most 255 rigid bodies. `joints.joint_target` warns above the target, `joints.joint_max` refuses the export. `joints.optimize: lock` turns the least important D6 joints into fixed joints; `full` also merges rigid links into one body (`joints.merge_stiffness` chooses how merged links keep their flexibility).

## Collision Filtering

Parent-child and sibling filtering is part of branch authoring. Detached tomatoes are filtered against their own pedicel and related rachis links. Pairs of colliders that already overlap in the rest pose are filtered or rejected according to `collisions.initial_overlaps` (see [05_collision_checks.md](05_collision_checks.md)).
