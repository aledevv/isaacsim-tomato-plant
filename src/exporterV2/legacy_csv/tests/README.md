# Legacy CSV Builder Tests

`src/exporterV2/legacy_csv/` is an old builder that reads GroIMP CSV graph
exports. The exporter no longer reads CSV; it reads a
`plant_skeleton/1.0` JSON. The builder is kept only as a fixture: several core
tests (`core/optimizations`, `core/skinning`, `core/usd`, collision geometry)
use the branch dictionaries it produces. These tests check that the fixture
still behaves as those core tests expect.

## Run

From the project root:

```bash
uv run pytest -q src/exporterV2/legacy_csv/tests
```

`test_generation_config.py` and `test_parser_truss_integration.py` read
`data/simulation_output/dynamic_output/graphs/graph_day_<N>.csv` (days 50 and
80).

## Files

| File | Covers |
| --- | --- |
| `test_truss_builder.py` | Truss branch generation without `pxr`: rachis orientation, alternating lateral pedicels, coaxial terminal pedicel, default pedicel length, radius clamping to `MIN_LINK_RADIUS_WORLD`, tomato mass, even fruit counts |
| `test_parser_truss_integration.py` | CSV parsing into trunk trusses and terminal tomato bodies (day 80), filtering of zero-size organs (day 50), and the joint optimizer meeting the D6 budget |
| `test_generation_config.py` | Resolution caps, rigid-trunk joint type, organ switches in the legacy `tree_config` profile, single CSV read, and the JSON metadata of the effective configuration |

## Truss Model In The Fixture

- **Rachis**: articulated chain of links.
- **Pedicels**: lateral links alternating ±90°, plus one coaxial terminal pedicel.
- **Tomatoes**: spheres attached with a FixedJoint, outside the articulation.
- **Drives**: Euler-Bernoulli beam stiffness per joint:

```
I = π r⁴ / 4      second moment of area   [m⁴]
K = E I / L       spring constant         [N·m/rad]
D = 2ζ √(K J)     damping coefficient     [N·m·s/rad], J = link inertia
```

The fixture takes its constants from `core/tree_config.py`. These are not the
production exporter settings, which live in `config/physics.yaml`.
