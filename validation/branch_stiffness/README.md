# Branch stiffness in the plant

Two checks on the exported plant itself.

| Script | What it does | Needs Isaac Sim |
| --- | --- | --- |
| `branch_stiffness_probe.py` | Settles the plant, pushes the tip of each side shoot (and three leaf rachides) with a small horizontal force, and reports the tip stiffness (force / displacement) and the free oscillation. Run it at several physics rates to see how much softer the solver makes the branches. | yes |
| `rest_angles.py` | Hand calculation of the bend of every D6 joint at rest: moment of the weight downstream of the joint divided by the joint stiffness. Checks that the joints stay far from their ±30° (pedicels ±25°) limits. | no |

## Results

Side-shoot tip stiffness, 0.05 N push (stored in
`../three_point_bending/results/gao_three_point_bending.json`, measured with the
development version of the exporter):

| Physics rate | Tip stiffness (N/m) |
| --- | ---: |
| hand calculation from the authored joints | 59.8 to 60.2 |
| 60 Hz (interactive default) | 45.5 to 48.2 |
| 240 Hz | 58.9 to 59.6 |
| 960 Hz | 58.8 to 59.5 |

At 60 Hz the side shoots are about 20% softer than authored; at 240 Hz they
match within 1-2%.

Rest angles of the default plant of this repository (`rest_angles.py`):

| Organ | Joints | Median (deg) | Max (deg) |
| --- | ---: | ---: | ---: |
| branch | 16 | 3.27 | 3.88 |
| leaf | 55 | 0.44 | 1.98 |
| pedicel | 72 | 0.05 | 0.08 |
| truss rachis | 63 | 0.05 | 1.42 |

## Run

```bash
./run_main.sh                     # or: uv run python -m exporterV2 (writes output/plant.usda)
uv run python validation/branch_stiffness/rest_angles.py output/plant.usda

~/isaacsim/python.sh validation/branch_stiffness/branch_stiffness_probe.py \
  --usd output/plant.usda --hz 60 --out output/probe_60hz.json
~/isaacsim/python.sh validation/branch_stiffness/branch_stiffness_probe.py \
  --usd output/plant.usda --hz 240 --out output/probe_240hz.json
```
