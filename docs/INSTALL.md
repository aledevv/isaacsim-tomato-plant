# Installation

AutoTom has two parts with separate environments:

| Part | Python | What it does | Needs a GPU |
| --- | --- | --- | --- |
| Exporter (`src/exporterV2`) | 3.14 | `input/skeleton.json` -> `output/plant.usda` | No |
| Isaac Sim | its own (`python.sh`) | Opens and simulates the USD file | Yes (RTX) |

The leaflets are plain USD files in `assets/leaves/default/`, so they need no
extra environment.

All four routes below install the same pinned versions: `uv.lock` is the source
of truth, and `requirements.txt` is exported from it. The tested Isaac Sim
version is 4.5.

```bash
git clone https://github.com/aledevv/isaacsim-tomato-plant
cd autotom_digital_twin
```

## Choose a route

| Route | Use it when |
| --- | --- |
| [uv](#uv-recommended) | Day-to-day development (recommended) |
| [venv + pip](#venv--pip) | No uv allowed, Python 3.14 already installed |
| [conda](#conda) | Lab machines that already use conda |
| [Docker](#docker) | Exact reproduction, papers, other machines |

## uv (recommended)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh    # once
./run_main.sh --generate-only
```

`uv run` creates `.venv` from `uv.lock` on first use and installs Python 3.14
if needed. After the first run, `UV_OFFLINE=1 ./run_main.sh` works without
network.

## venv + pip

Needs Python 3.14 (for example from
[deadsnakes](https://launchpad.net/~deadsnakes/+archive/ubuntu/ppa) or
`pyenv`).

```bash
python3.14 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt     # pinned versions
pip install --no-deps -e .

./run_main.sh --generate-only
```

Without uv in `PATH`, `run_main.sh` uses the active `python3`.

## conda

```bash
conda env create -f environment.yml    # autotom: Python 3.14 + requirements.txt + this repo
conda activate autotom
./run_main.sh --generate-only
```

Python comes from conda-forge; packages are installed with pip from the same
pinned files as the venv route.

## Docker

The image pins the operating system, Python and every package, so the same
commit gives the same USD file on any machine.

Exporter only (CPU, any machine):

```bash
docker build -t autotom .
docker run --rm -v "$PWD/output:/app/output" autotom                 # writes output/plant.usda
docker run --rm autotom uv run pytest -q src/plant_skeleton/tests src/exporterV2/tests   # test suite
```

Its `plant.usda` is byte-identical to a local `./run_main.sh --generate-only`.

Exporter plus Isaac Sim (NVIDIA GPU, driver and
[NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html);
the base image is `nvcr.io/nvidia/isaac-sim:4.5.0`):

```bash
docker build --target isaac -t autotom-isaac .
docker run --rm --gpus all -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y autotom-isaac    # headless physics check
```

The Isaac image runs `./run_main.sh --headless` by default. The GUI inside a
container needs Isaac Sim's livestream client or X forwarding; see NVIDIA's
[container installation guide](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_container.html).
Change the Isaac Sim version with `--build-arg ISAAC_IMAGE=...`.

## Isaac Sim outside Docker

Install Isaac Sim 4.5 following NVIDIA's workstation guide. `run_main.sh`
looks for `~/isaacsim/python.sh`; set `ISAACSIM_DIR` if it is elsewhere.
Nothing from this repository is installed into Isaac Sim's Python.

## Checking an installation

```bash
./run_main.sh --generate-only
sha256sum output/plant.usda
```

Exports are deterministic: the same commit and configuration give a
byte-identical `plant.usda` with uv, pip, conda or Docker. Comparing the hash
with a colleague's is a quick reproducibility check.

## Updating dependencies

1. Edit `pyproject.toml`.
2. `uv lock`.
3. Regenerate the pinned file:

```bash
uv export --frozen --no-hashes --emit-index-url --no-dev --no-emit-project -o requirements.txt
```

A module whose dependencies do not fit Python 3.14 should run in its own
environment and talk to the exporter through files (for example JSON in, JSON
out), with one more `requirements-<module>.txt` and a stage in the `Dockerfile`.
