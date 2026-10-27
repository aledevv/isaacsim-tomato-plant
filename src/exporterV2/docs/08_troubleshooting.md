# Troubleshooting

## Import Errors

Run ordinary tests from the project root so `src/` imports resolve:

```bash
uv run pytest -q src/exporterV2/legacy_csv/tests src/exporterV2/core/optimizations/tests \
  src/exporterV2/tests src/exporterV2/core/skinning/tests src/plant_skeleton/tests
uv run pytest -q src/exporterV2/core/usd/tests
```

Do not collect the entire `src/exporterV2` directory with ordinary Python:
`core/usd/tests` contains standalone Isaac-oriented tests and needs Isaac Sim
Python:

```bash
~/isaacsim/python.sh -m pytest src/exporterV2/core/usd/tests -v
```

## Config Errors

Unknown keys in `config/physics.yaml` or `config/visuals.yaml` stop the export
with `[ERROR]`, so a typo never passes silently. Every key is optional; delete
a line to get the default written in the file.

## Interactive FPS vs Validation Rate

The exporter authors and validates physics at `simulation.physics_hz`
(default `480`), while the Isaac Sim GUI runs at
`simulation.interactive_physics_hz` (default `60`). `./run_main.sh` reads
both from the manifest written next to the USDA:

```bash
./run_main.sh                          # GUI at interactive_physics_hz
./run_main.sh --headless --duration 5  # finite check at physics_hz
```

Interactive alternatives are `120`, `240` and `480 Hz`. Increasing the GUI
rate adds 2, 4 or 8 PhysX substeps to every 60 Hz render update. Do not
diagnose this cost from joint count alone: compare stages at the same runtime
timestep.

The benchmark tool reports USD complexity, render updates/s and raw PhysX
steps/s for two stages:

```bash
uv run python -m exporterV2.performance_benchmark \
  --baseline before=/tmp/before.usda \
  --candidate after=/tmp/after.usda \
  --physics-hz 60,120,240,480 \
  --output /tmp/performance.json
```

On Isaac Sim 4.5, constructing `World` after opening an existing stage can
leave `SimulationManager` unaware of its already-authored `PhysicsScene`; its
rate getter then silently returns 60 Hz. `isaac_app.py` synchronizes that
registry explicitly and reapplies the selected rate after `World.reset()`.
Do not remove this shim because the USD attribute reads 480 Hz: authored and
effective runtime rates are separate checks.

## Branches Look Cylindrical

Check `quality` in `config/visuals.yaml`. `realistic` (default) uses 20 radial
samples, 5 mm axial spacing and 9 radius-transition samples on every segmented
axis; `performance` uses 12 / 12 mm / 5. Neither authors visual cylinders. A
truss containing `HistoricalTrussRachisVisual` cylinders or a non-zero
`visual_cylinders` audit count is a regression.

## Leaflet or Pedicel Angles Look Wrong

`pose: v2-aesthetic` (default) keeps the skeleton topology and dimensions but
spreads leaflets left/right at their inclination and alternates pedicels
around the truss. `pose: canonical` uses the frames written in the skeleton
as they are. The manifest stores both `source_pose` and `authored_pose`, and
mesh, collider, joint, leaf blade and tomato all follow the authored pose
together.

Leaflets and blades are rigid visuals: no rigid body, collider or joint. Their
mass from the skeleton is added to the host support. Do not make them rigid
bodies to get more bending; that quickly exhausts the joint budget.

## Truss Stiffness and Damping

Truss values are the `truss_rachis` and `pedicel` blocks under `segments:` in
`config/physics.yaml`. The defaults, chosen by eye on day 160, are rachis
`20 GPa`, pedicel `4 GPa`, damping ratio `4` for both and pedicel
`drive_stiffness_scale: 0.2`.

- Raise `damping_ratio` for persistent oscillation, never to correct static sag.
- Keep truss density at `2000 kg/m3` (ordinary tissue `1000`). Do not raise
  density to hide an unstable drive.

Pedicel length is set by the input skeleton. The example tomatoes in
`input/` have pedicels drawn 3x longer than the growth model's, so big tomatoes
do not crowd the rachis.

## Too Many Joints

The export warns above `joints.joint_target` and stops above
`joints.joint_max` or 255 articulation links. Set `joints.optimize: lock` to
turn the least important D6 joints into fixed joints, or `full` to also merge
rigid links.

## Tomatoes Fall or the Plant Will Not Settle

`fruits.mode: physical` is experimental and not supported on mature plants.
The default `visual` draws tomatoes without mass, rigid body, collider or
terminal joint.

With `physical`, tomatoes are external bodies with
`excludeFromArticulation=true` and an immediate break force
(`fruits.break_force_n`, default `6 N`). There is no soft-start, gravity ramp
or automatic equilibrium capture.

Do not infer stability from finite coordinates. The headless monitor fails an
unforced terminal body that travels more than `0.25 m` and reports its path
and joint ancestry. In the 2026-08-25 checkpoint, day 160 still failed to
settle with all colliders disabled and the break force raised to `1e9 N`, so
residual contacts are not the primary cause; the 72 external terminal bodies
and FixedJoints are the unsupported boundary.

## Tomatoes Do Not Detach

Only applies to `fruits.mode: physical`. Verify that:

- `fruits.detachable: true`;
- the tomato body is under `/World/TerminalBodies`;
- the FixedJoint has a finite `breakForce`;
- `physics:excludeFromArticulation = True`;
- the interacting object applies enough impulse to exceed the break force.

## Collision Instability Near Tomatoes

Inspect `FilteredPairs` on the tomato body. Detachable tomatoes should filter
collisions toward their pedicel and related truss rachis. Missing filters can
make contact resolution fight the FixedJoint.

## A Different Plant Reports Initial Overlaps

With `collisions.initial_overlaps: filter` (default) the exporter measures
the authored capsule, cylinder and sphere colliders and filters only each
rigid-body pair that overlaps at rest. The manifest records collider paths,
roles, IDs, contact count and maximum penetration depth.

This prevents startup impulses but permanently disables contact between that
pair. It does not change the skeleton pose and does not filter the whole stem.
Use `initial_overlaps: error` to check whether a new overlap is a modelling
problem in the input rather than an intentionally dense canopy.

Sphere checks must be order-independent. A sphere is encoded internally as a
zero-length swept segment; both point-segment orders have regression coverage.
If behavior changes after edits to the narrow phase, run the reversed
sphere-capsule and sphere-cylinder tests before adding more filters.

## Runner

The only root runner is `run_main.sh`.
