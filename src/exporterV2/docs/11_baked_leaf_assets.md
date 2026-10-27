# Baked Leaflet Assets

Realistic leaflets are generated **offline**, once, into a fixed set of USD assets.
The exporter and Isaac Sim only load those files: nothing is generated at
startup, and Isaac needs no extra Python packages.

Each asset is a complete 3D leaflet from the tomato leaflet generator
[tomato-leaf-gen](https://github.com/aledevv/tomato-leaf-gen), learned from the
Chitwood et al. tomato leaflet photos (outline and veins) and Pheno4D scans
(bend):

| Part | Model |
| --- | --- |
| 2D outline | RAE-64 + flow (`rae_flow`), subset `field` |
| 3D bend | G1-GMM-NORM-12: PCA-12 height field + Gaussian mixture fitted on Pheno4D, temperature 0.9 |
| Veins | VeinFlow draws the veins inside the outline; the `bridge_loopcut` repair cuts loops and joins detached pieces to the main vein system; the drawing becomes a drainage tree from the base |
| Relief | `pillow` profile, lamina 0.40 mm |
| Mesh | one closed solid; top and bottom meet on the outline with zero thickness |

## The default set

`assets/leaves/default/` holds 131 leaflets baked with seed 42 and VeinFlow + repair veins,
one for every leaflet of the default plant, with 512 px textures:

```text
assets/leaves/default/
  manifest.json                     settings, per-leaf provenance, sizes, timings
  leaf_000.usda ... leaf_130.usda   one leaflet each (~4k vertices, ~8k triangles)
  textures/leaf_NNN_{top,bottom}_{albedo,normal}.png
```

The whole folder is about 160 MB. Any plant with up to 131 leaflets gets one unique leaf
per leaflet from it; a larger plant reuses them (see below).

Asset frame, the same for every file:

- metres, `upAxis = Z`
- attachment (biological base) at the origin
- tip towards +Y, at `manifest["length_m"]` (0.06 m) from the origin
- upper side towards +Z (the bend lifts the blade in +Z)
- prim layout: `/<leaf_NNN>/Blade` (Mesh) and `/<leaf_NNN>/Looks/{Top,Bottom}` (UsdPreviewSurface)

The blade has two `GeomSubset`s, `Top` and `Bottom`, each bound to its own material
(albedo plus tangent-space normal map; the bottom is paler with whitish veins). The
mesh carries the bend and the coarse relief. The textures carry the fine relief and the
vein colours.

## How the exporter uses them

`leaves.generator: baked` in `config/visuals.yaml` (the default) replaces every
procedural blade with a reference to a baked leaflet. The code is
`leaf_shapes/baked.py` (`prepare_baked_leaves`, `reference_baked_leaf`; it
imports nothing beyond `pxr`) and the `baked` branch of `visuals/leaves.py`.

- **Which asset.** With `leaves.one_per_leaflet: true` every leaflet gets its own
  asset (below). Otherwise, or when the plant has more leaflets than the set,
  the leaflet identity (`leaflet_seed`: global seed, plant ID, structural ID,
  role) is hashed modulo the size of the set. Either way
  the same skeleton and seed always give the same assets, and a different
  `leaves.seed` reshuffles them. `leaves.baked_set: <folder>` uses another
  baked set instead.
- **Placement.** The asset origin sits at the petiolule tip, +Y along the
  petiolule and +Z towards the sky, then the blade is tipped down by
  `leaves.droop_deg` (22 degrees, plus or minus `droop_jitter_deg`) and rolled
  around its axis by up to `roll_jitter_deg`, both jittered per leaflet. It is
  scaled uniformly to the blade length the procedural blade would have
  (`visuals/leaves.py: blade_length`). The bend is in the mesh, so no
  procedural arch, sag or fold is added.
- **Prims.** `<petiolule>/LeafBlade` is an instanceable `Xform` that references
  the asset and carries `autotom:leafAsset`, `autotom:leafAssetSeed`,
  `autotom:leafShapeStructuralId`, `autotom:leafletRole` and
  `autotom:leafDroopDeg`. References are relative to the exported stage, so
  keep `output/` and `assets/leaves/` in the same relative place when moving
  exports (or flatten the stage).
- **Physics.** The blade stays visual only. Its mass is added to the host body
  at the same centroid as before (`BLADE_CENTROID_FRACTION`).

The shapes are role-agnostic: the outline model does not condition on left, right or
terminal. Any asset can go on any leaflet. Do not mirror them per role.

## One leaflet per leaflet (`leaves.one_per_leaflet: true`)

When the default set was baked with `leaves.seed` (42) and holds at least as many
leaflets as the plant, every blade gets its own leaflet. Blades are ordered by their
leaflet seed and blade `k` gets `leaf_k`, so the result does not depend on the
skeleton's node order. A plant with more leaflets, or another seed, reuses the set
across blades and prints a warning.

## Making a new set

The set was made with the tomato-leaf-gen models (RAE-64 + flow outline, VeinFlow
veins, bend prior), 131 leaflets with seed 42, 60 mm base-tip length, 0.8 mm mesh
spacing and 512 px textures. To use another set, bake leaflets in the same asset
frame and layout (above) into a folder with a `manifest.json` listing them, and point
`leaves.baked_set` at it, or replace `assets/leaves/default`. Check a set with:

```bash
uv run pytest -q src/exporterV2/tests/test_baked_leaf_assets.py
```
