# Developer guide

A short tour of the project for a developer who has never seen it. It takes
about ten minutes to read; the detailed references are linked at the end.
Coding agents should read [`AGENTS_GUIDE.md`](AGENTS_GUIDE.md) instead.

## 1. What the project is

AutoTom Digital Twin builds an interactive tomato plant for NVIDIA Isaac Sim.
It reads a **plant skeleton** (a graph of straight segments with leaflets and
fruits) and writes an OpenUSD scene in which every segment is a rigid body,
joints bend under gravity and touch, and the plant can be pushed and dragged
with the mouse.

The skeleton can come from any source: a growth model, a reconstruction of a
real plant, or a hand-written file. The example plants in `input/` are a dwarf
tomato from a GroIMP growth simulation.

## 2. Quick start

Requirements: Linux, [uv](https://github.com/astral-sh/uv) (Python 3.14 is
installed by uv), and Isaac Sim at `~/isaacsim` to open the scene (set
`ISAACSIM_DIR` if it is elsewhere). Building the USD file does not need Isaac
Sim. To use pip, conda or Docker instead of uv, see [`INSTALL.md`](INSTALL.md).

```bash
git clone https://github.com/aledevv/isaacsim-tomato-plant
cd autotom_digital_twin
./run_main.sh                    # input/skeleton.json -> output/plant.usda -> Isaac Sim
./run_main.sh --generate-only    # only write the USD file
./run_main.sh --headless         # 5 s physics check without the GUI
```

A successful build prints a summary such as
`[OK] 216 segments -> 216 rigid bodies, 206 D6 joints, 131 leaflets, 72 fruits (scale 2)`.

## 3. How the pipeline works

```text
any plant source ──> input/skeleton.json ──exporter──> output/plant.usda ──> Isaac Sim
                     plant_skeleton/1.0    + config/physics.yaml
                                           + config/visuals.yaml
```

1. **Skeleton.** The plant as `plant_skeleton/1.0`: one node per segment, each
   with a kind (`stem`, `branch`, `petiole`, `leaf_rachis`, `truss_rachis`,
   `pedicel`), plus leaflets and fruits.
2. **Branches.** The exporter groups nodes into chains (trunk, side shoots,
   leaves, trusses). Each node becomes one rigid link with capsule colliders.
3. **Joints.** Links are connected by D6 joints whose stiffness comes from the
   segment's Young's modulus, radius and length (`config/physics.yaml`). The
   main stem is welded with fixed joints so the plant stands.
4. **Joint budget (optional).** Isaac Sim handles about 220 bending joints per
   plant; above that the optimizer can lock or merge the least important ones.
5. **Visuals.** Organic stem meshes in one dark green, baked 3D leaflets
   (one per leaflet, hanging a little below their stalk), curved pedicels,
   tomato spheres with a calyx. Visuals never change the physics.
6. **Collisions and checks.** Colliders that overlap at rest are filtered,
   distant branches, petioles and leaf rachides are allowed to touch each
   other, and a manifest JSON next to the USD file records counts, poses,
   masses and settings.

## 4. Repository map

| Path | What it is |
| --- | --- |
| `run_main.sh` | The one launcher: build, then open in Isaac Sim. |
| `input/skeleton.json` | Default plant (a 160-day dwarf tomato). `input/examples/` has a 16-segment hand-written plant and the same tomato at days 10, 25 and 50. |
| `config/physics.yaml`, `config/visuals.yaml` | All settings, each option commented with its default. |
| `src/plant_skeleton/` | The input format: data model, JSON I/O, validation. |
| `src/exporterV2/` | The exporter: pipeline, mesh generators, Isaac Sim launcher. |
| `assets/leaves/default/` | 131 baked 3D leaflets, one per leaflet of the default plant. |
| `output/` | Generated files, not tracked. |

Things you can ignore at first: `src/exporterV2/tests/legacy_csv/` (an old
CSV plant builder and its CSVs, kept only as a fixture for the joint-optimizer
tests) and `validation/` (complexity profile, performance benchmark and the
cantilever bending study, outside the pipeline).

## 5. Common tasks

**Use another plant.**

```bash
./run_main.sh --input input/examples/tomato_day50.json
```

**Load a plant from another program.** Write `plant_skeleton/1.0` directly,
or convert your data with a small script on your side. The format and a
step-by-step conversion guide are in
[`src/plant_skeleton/README.md`](../src/plant_skeleton/README.md).

**Change how the plant looks.** Edit `config/visuals.yaml` (mesh density,
leaflet generator and droop, stem colour, decorations, tomato size, calyx). To change a mesh
generator itself, find it in
[`src/exporterV2/visuals/README.md`](../src/exporterV2/visuals/README.md).

**Change the physics.** Edit `config/physics.yaml`: one block per segment kind
(joint type, Young's modulus, damping, density, bend limit), fruit mode,
solver iterations, branch collisions and `global_scale`. For a much smaller or
larger plant, start with `global_scale: auto`.

**Run the tests.** Run them by hand before merging:

```bash
uv run pytest -q \
  src/plant_skeleton/tests \
  src/exporterV2/tests \
  src/exporterV2/core/optimizations/tests \
  src/exporterV2/core/skinning/tests
uv run pytest -q src/exporterV2/core/usd/tests
uv run pytest -q validation
```

Interaction in Isaac Sim (dragging, bending, fruit detachment) is checked by
hand with `./run_main.sh`.

## 6. Known limitations

- The Young's modulus of stems and leaves is measured (50.64 MPa, Gao et al.
  2024); the other physics parameters are tuned by eye in Isaac Sim, not
  calibrated on a real plant.
- At the interactive rate of 60 Hz the side shoots bend about 20% more than
  the measured modulus predicts (a solver effect); at 240 Hz they match within
  2%.
- `global_scale: 2` is needed for the example tomato: PhysX is unstable with
  bodies thinner than about 2 mm.
- Visible stem meshes are a few millimetres thicker than their colliders, and
  leaflet stalks have no colliders, so touching parts can overlap slightly.
- Physical tomatoes (`fruits.mode: physical`) are stable only on trusses of the
  main stem. The default is `visual`: tomatoes are drawn but have no mass.
- Fruits are spheres only.
- Only one plant per scene has been tested.

## 7. Future work

- **Visual details.** Plant tips and terminal shoots can look more natural.
- **PhysX time-step bug.** PhysX seems to have a bug related to the time step
  (dt), also visible in the public PhysX source. To confirm it, reproduce it in
  a minimal scene or build PhysX with the fix.
- **Tomatoes on side branches.** They make the plant unstable. A PhysX
  TGS-on-GPU bug lowers the effective break threshold to about
  `breakForce / N` (N = solver iterations), which explains why fruits on side
  trusses fall off on their own. It does not explain the 13-24 cm sagging of the
  side branches, which also happens with the PGS solver.
- **Several plants in one scene.** Not tested in Isaac Sim yet. It should be
  possible to turn collisions and physics on or off per plant.
- **Branch generator.** There is no generator that builds a branch from a
  length and a number of joints; the skeleton must provide complete branches.

## 8. Further reading

- [`src/plant_skeleton/README.md`](../src/plant_skeleton/README.md): input format with examples, converting another source.
- [`src/exporterV2/README.md`](../src/exporterV2/README.md): pipeline steps, folder map, what to edit.
- [`src/exporterV2/visuals/README.md`](../src/exporterV2/visuals/README.md): every mesh generator.
- [`src/exporterV2/docs/`](../src/exporterV2/docs/): physics and mechanics, collisions, segmented visuals, leaflets (`11_baked_leaf_assets.md`), troubleshooting.
- [`src/exporterV2/core/optimizations/docs/`](../src/exporterV2/core/optimizations/docs/): joint-budget optimizer.
