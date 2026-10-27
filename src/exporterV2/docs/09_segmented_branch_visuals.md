# Vegetative Visual Modes

The exporter separates vegetative physics from visible organic surfaces. PhysX always owns the rigid links, joints and collision proxies; the visual layer only decides how the same resolved plant is rendered.

## Visual Modes

The exporter always uses the skinned vegetative backend with the `segmented` visual mode (`stems.generator: organic_segmented` in `config/visuals.yaml`). The other modes still exist in `core/skinning` but can no longer be selected from the runner or the config:

| Mode | USD representation | Status |
| --- | --- | --- |
| `segmented` | One local organic mesh per rigid link | Used by the exporter |
| `skinned` | Continuous tube, per-axis UsdSkel, runtime synchronization | Core only; deformation reference |
| `static` | Continuous world-space mesh | Core only; geometry-cost benchmark |
| `rigid-single` | Continuous local mesh on one-link axes; other axes use the skinned path | Core only; runtime diagnostic |

The `legacy` cylinder branch backend also remains in `core/usd/stage.py` for the legacy CSV test fixture.

Development profiling on a day-40 plant found that static and segmented organic geometry performed close to the old cylinder visuals, while full runtime UsdSkel evaluation was substantially heavier. That is why `segmented` is the production mode.

## Shared Visual Profile

`build_visual_axes()` groups resolved vegetative branches into continuous botanical axes. Each `VisualAxisData` records:

- source segments and their radii;
- physical link intervals and rest transforms;
- metric sample spacing;
- taper and radius-transition settings;
- attachment arcs used for local junction swelling.

For straight rest-pose centerline origin `p0`, axis `a`, and arc length `s`:

$$
\mathbf{c}(s)=\mathbf{p}_0+s\mathbf{a}.
$$

A ring point uses radius `r(s)` and an orthonormal frame `n`, `b`:

$$
\mathbf{v}(s,\theta)=\mathbf{c}(s)+r(s)
\left(\mathbf{n}\cos\theta+\mathbf{b}\sin\theta\right).
$$

Botanical taper, smooth radius transitions, petiolule root treatment and attachment swelling are calculated once in `mesh.py` and reused by all organic representations. `stems.junction_bulge: false` sets the swelling amplitude to zero, and `quality` (`realistic` or `performance`) sets the ring and sample density.

## Segmented Representation

For each physical link, `visual_segmented.py` selects the rings inside that link's arc interval and converts points into the link's rest-local frame. Standard USD transform inheritance then makes the mesh follow PhysX without a skeleton runtime.

```text
Rigid link
├── collision proxy
└── OrganicVisual_NN
```

A short tapered tongue extends a parent visual into the next link to hide cracks during joint rotation. The tongue is visual-only and does not add collision shapes, rigid bodies, joints, or prim-level physics.

When every link of a visual axis is joined by fixed joints (the stem with the
default `segments.stem.joint: fixed`, or branches locked by `joints.optimize`),
the axis cannot bend, so `author_segmented_visual_axis` builds one continuous
mesh over the whole axis, in the frame of its first link (`OrganicVisual_01`).
This removes the visible seams at the nodes without changing physics.

## Terminal Leaf Junctions

A vegetative petiole attached near the end of a structural lateral can be marked as its centered terminal continuation. The rendered host tip narrows toward the actual petiole contact radius, while the petiole root extends slightly inside the host. This keeps the junction visually closed without changing attachment physics.

`terminal_fork.py` can add a small secondary young shoot and leaf under the host's final rigid link (`stems.terminal_forks`, default `true`). Its azimuth is deterministic from branch identifiers. Truss and tomato terminals are excluded from this dressing.

Leaf blades are separate double-sided meshes with a shared leaf material. Their geometry and material do not alter leaf collision proxies or branch topology.

## Materials

Visual meshes bind shared materials from `core/usd/materials.py`:

- `/World/Looks/TomatoStem` for vegetative organic surfaces;
- `/World/Looks/TomatoLeaf` for leaf blades;
- maturation buckets below `/World/Looks/TomatoFruit` for tomatoes.

Realtime generation uses `UsdPreviewSurface`. Optional OmniSurface presets remain in the code for isolated look development; the exporter does not use them.

## Implementation Map

- `builder.py`: visual-mode selection and orchestration.
- `adapter.py`: vegetative graph resolution and centered-terminal classification.
- `axis.py`: continuous visual-axis construction.
- `mesh.py`: shared profile, tube samples, plain mesh authoring, and `skinned` mode.
- `visual_segmented.py`: per-link organic meshes and overlap tongues.
- `visual_static.py`: static continuous benchmark.
- `visual_rigid.py`: one-link direct attachment mode.
- `visual_modes.py`: compatibility re-exports.
- `terminal_fork.py`: terminal leaf-only dressing.
- `leaf_blade.py`: procedural double-sided leaf blades.
- `runtime.py`: PhysX-to-UsdSkel synchronization.

## Running

```bash
./run_main.sh                  # segmented visuals, settings from config/visuals.yaml
./run_main.sh --generate-only  # only write output/plant.usda
```

The invariant across visual modes is physical equivalence: visual implementation changes must not redefine rigid bodies, joints, collision proxies, masses, detachment or filters.
