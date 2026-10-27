# Cantilever Bending Validation

This directory contains the reproducible validation protocol for articulated
branch physics. This experiments considers data from the [paper](https://www.mdpi.com/2077-0472/14/4/531) by Gao et al. "Discrete Element Model Building and Optimization of Tomato Stalks at Harvest" (2024) as groundtruth.
It keeps three questions separate:

1. Does PhysX reproduce the mechanics of the generated rigid-link chain?
2. Does that chain converge toward continuum beam theory as `N` increases?
3. How does the current `legacy_physics=True` branch differ from
   `new_physics`, and are they physically comparable?

The canonical English report is:

```text
docs/CantileverValidationReport.md
```

Machine-readable evidence and the flat measurement table are in `results/`.
Generated USD artifacts are in `data/usd_models/physics_tests/`.

## Requirements

Run commands from the repository root. Pure Python commands use `uv run`.
Generation and simulation must use Isaac Sim's Python interpreter:

```text
uv run python ...
~/isaacsim/python.sh ...
```

The accepted principal configuration is CPU TGS with `32` position and `4`
velocity iterations, fixed root, biaxial D6 bending, disabled collisions, and
force applied at the true geometric tip.

## Full Reproduction

The full evidence run is intentionally explicit and can take several minutes:

```bash
./src/exporterV2/tests/cantilever_bending_experiment/run_paper_experiment.sh
```

It performs, in order:

1. Formula and protocol tests.
2. Synthetic spatial convergence under a 0.05 N tip load.
3. Synthetic N20 timestep convergence.
4. Synthetic self-weight spatial convergence.
5. A 960 Hz synthetic self-weight negative/settling control.
6. Current legacy/new behavior comparison.
7. Gao tomato-stalk spatial convergence under both loads.
8. Gao N20 timestep checks.
9. Acceptance recomputation, Markdown report generation, and PNG figures.

The first simulation starts a fresh aggregate dataset. Later commands use
`--append-results`; the complete experimental key replaces matching records,
so rerunning a phase does not duplicate measurements. Every Isaac invocation
also writes `results/cantilever_validation_last_run.json` before aggregate
merging, which preserves the latest run if a later phase fails.

## Quick Diagnostic

For a shorter synthetic D6 check:

```bash
./src/exporterV2/tests/cantilever_bending_experiment/run_experiment.sh
```

This is useful during development but is not the complete evidence matrix.

## Individual Commands

Pure formula check and tests:

```bash
UV_CACHE_DIR=/tmp/uv-cache uv run python \
  src/exporterV2/tests/cantilever_bending_experiment/cantilever_validation.py formula-check

UV_CACHE_DIR=/tmp/uv-cache uv run python -m pytest \
  src/exporterV2/tests/cantilever_bending_experiment/test_validation_protocol.py -q
```

Generate and simulate one controlled case:

```bash
~/isaacsim/python.sh \
  src/exporterV2/tests/cantilever_bending_experiment/cantilever_validation.py all \
  --benchmarks synthetic_solid_40cm --models new_physics \
  --supports fixed --joint-models d6_biaxial --n-links 20 \
  --scenarios tip_force_0p05N --force-point geometric_tip \
  --backend cpu --physics-hz 960,1920 --max-seconds 30 \
  --solver-position-iterations 32 --solver-velocity-iterations 4
```

Audit existing USD files without starting Isaac Sim:

```bash
UV_CACHE_DIR=/tmp/uv-cache uv run python \
  src/exporterV2/tests/cantilever_bending_experiment/cantilever_validation.py audit \
  --benchmarks synthetic_solid_40cm --models new_physics \
  --supports fixed --joint-models d6_biaxial --n-links 3,5,10,15,20 \
  --backend cpu
```

Regenerate the report and figures from an existing aggregate JSON, without
rerunning Isaac Sim:

```bash
UV_CACHE_DIR=/tmp/uv-cache uv run python \
  src/exporterV2/tests/cantilever_bending_experiment/cantilever_validation.py report

MPLCONFIGDIR=/tmp/matplotlib UV_CACHE_DIR=/tmp/uv-cache uv run python \
  src/exporterV2/tests/cantilever_bending_experiment/generate_report_assets.py
```

## Benchmarks

### `synthetic_solid_40cm`

- Solid cylinder: `L=0.40 m`, `r=0.010 m`.
- `E=100 MPa`, `rho=1000 kg/m3`.
- Continuum self-weight deflection: `12.5568 mm`.
- Continuum 0.05 N tip-force deflection: `1.3581 mm`.

### `tomato_gao_20cm`

- Hollow harvested tomato-stalk approximation from Gao et al. (2024).
- `L=0.20 m`, `d_o=11.1 mm`, `d_i=3.82 mm`.
- `E=50.64 MPa`, `rho=769.96 kg/m3`.
- Continuum self-weight deflection: `3.4637 mm`.
- Continuum 0.05 N tip-force deflection: `3.5836 mm`.

The Gao case is a literature-parameter plausibility benchmark, not direct
ground truth for a living greenhouse plant.

## Model and Reference Logic

For a hollow circular section:

```text
A = pi (r_o^2 - r_i^2)
I = (pi / 4) (r_o^4 - r_i^4)
EI = E I
```

Each link has length `l=L/N` and mass `rho A l`. Internal rotational spring
stiffness is `EI/l` per radian, converted to the USD angular-drive convention:

```text
k_USD = (EI/l) (pi/180)
```

The runner computes two references:

- Exact small-angle response of the generated rigid-link and hinge topology.
- Euler-Bernoulli continuum response.

PhysX is verified against the first. Spatial convergence is assessed against
the second. A fixed-root coarse chain is expected to be too stiff because its
first rigid cell cannot bend.

## Measurement Rules

- The undeformed baseline is recorded before applying gravity or tip force.
- The measured point is local `(0, 0, link_height)` transformed by the final
  runtime PhysX link pose, not a default-time USD transform or body COM.
- Tip force is applied at that geometric endpoint, including its moment arm.
- Settling is based on the deflection range over a declared time window.
- Timeout produces `not_settled`; it is never silently converted to a pass.
- A settled but inaccurate equilibrium is `settled_wrong_equilibrium`.
- Static USD audit checks units, physical length, mass, stiffness, drives,
  support, collision state, solver configuration, and fingerprint.

## Supported Diagnostics

`--joint-models` accepts `d6_biaxial`, single-axis `d6_planar`,
`revolute_planar`, and `fixed_chain`. `--supports` accepts the principal
`fixed` support and an explicitly modelled `half_cell` support with stiffness
`2EI/l`. These alternatives are diagnostic models and must not be mixed into
one convergence series.

Visual comparison is available with:

```bash
~/isaacsim/python.sh \
  src/exporterV2/tests/cantilever_bending_experiment/compare_cantilevers.py \
  --benchmark tomato_gao_20cm --n 10
```

Legacy script names remain wrappers. `cantilever_validation.py` is the source
of truth for quantitative evidence.
