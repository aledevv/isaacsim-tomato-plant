# Gao et al. three-point bending, rebuilt in PhysX

Question: if we run the stiffness test of Gao et al. on our branch model with
E = 50.64 MPa, do we get 50.64 MPa back?

Answer: yes, with fine links and a high physics rate (49.3 MPa, 2.7% low).
At coarse links or low physics rates the model reads softer.

## The paper

Gao et al. (2024), *Agriculture* 14(4):531. Stalks of greenhouse cherry
tomato at harvest, 80 mm pieces from the middle of the stalk, three-point
bending at 1 mm/min. E from beam theory:

    E = F L1^3 / (48 S I),   I = pi (d^4 - (d - 2w)^4) / 64

with d = 11.1 mm outer diameter and w = 3.64 mm bast thickness (Table 1).
They report E = 50.64 MPa (SD 2.42). The density (769.96 kg/m3) comes from
earlier studies; the shear modulus is computed with an assumed Poisson
ratio of 0.2.

Not in the paper, so not comparable:

- the support distance L1 (about 57 mm from their figure; we also run 50 and 65 mm);
- any force-deflection curve or raw data: only E can be compared.

## The test

`gao_three_point_bending.py` builds the 80 mm specimen from N rigid links.
The joint stiffness comes from the exporter's own `calculate_physics_params`
(k = E I / l), the same code used for the plant. The beam rests on two
frictionless static cylinders with gravity off; loads of 0.5 to 4 N are
applied at the middle; E is backed out from the slope of force against
midpoint deflection. The specimen is small and light, so its density is
raised 100 times (`--density-scale`): with gravity off this does not change
the static result. Solver: CPU TGS, 32 position iterations.

## Results

E backed out of the test (MPa), input 50.64 MPa, span 57 mm
(`results/gao_three_point_bending.json`):

| Links (length) | Exact chain | 480 Hz | 1920 Hz | 3840 Hz | 7680 Hz |
| --- | ---: | ---: | ---: | ---: | ---: |
| 2 (40 mm) | 24.1 | | 24.1 | | |
| 4 (20 mm) | 40.8 | | 42.0 | | |
| 8 (10 mm) | 47.6 | 40.3 | 45.1 | 45.4 | 45.2 |
| 16 (5 mm) | 49.9 | 34.1 | 45.0 | 47.3 | 46.8 |
| 32 (2.5 mm) | 50.4 | 23.7 | 40.9 | 45.7 | 49.3 |

"Exact chain" is the hand calculation for the same links and springs, with
no solver error. At 32 links and 7680 Hz the span does not matter: 49.5 MPa
at 50 mm, 49.3 at 57 mm, 50.0 at 65 mm.

The same file also holds the side-shoot tip stiffness of the plant at 60,
240 and 960 Hz, measured with `../branch_stiffness/branch_stiffness_probe.py`.

## Run

```bash
# needs Isaac Sim (~/isaacsim)
~/isaacsim/python.sh validation/three_point_bending/gao_three_point_bending.py \
  --links 2,4,8,16,32 --hz 1920 --out output/gao/span57_1920hz.json
~/isaacsim/python.sh validation/three_point_bending/gao_three_point_bending.py \
  --links 8,16,32 --hz 7680 --span-mm 50 --out output/gao/span50_7680hz.json

# figures from the saved results (no Isaac Sim)
uv run --with matplotlib python validation/three_point_bending/plot_gao_three_point_bending.py \
  validation/three_point_bending/results/gao_three_point_bending.json output/gao/figures
```

`results/gao_three_point_bending.json` was measured with the development
version of the exporter (Isaac Sim 4.5, RTX 4080 Laptop, i9-13900HK). The
specimen does not depend on the plant, only on the joint formula, which is
the same in this repository.
