# Reproducible AutoTom Digital Twin images. See docs/INSTALL.md.
#
# exporter (default target, CPU only): skeleton JSON -> output/plant.usda, plus the test suite
#   docker build -t autotom .
#   docker run --rm -v "$PWD/output:/app/output" autotom
#
# isaac (NVIDIA GPU + NVIDIA Container Toolkit): same image on top of Isaac Sim
#   docker build --target isaac -t autotom-isaac .
#   docker run --rm --gpus all -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y autotom-isaac

ARG PYTHON_IMAGE=python:3.14-slim-bookworm
ARG ISAAC_IMAGE=nvcr.io/nvidia/isaac-sim:4.5.0
ARG UV_VERSION=0.11.32

# --- uv (build tool shared by both images) -----------------------------------
FROM ${PYTHON_IMAGE} AS uv
ARG UV_VERSION
RUN pip install --no-cache-dir "uv==${UV_VERSION}"

# --- isaac -----------------------------------------------------------------
FROM ${ISAAC_IMAGE} AS isaac
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python \
    UV_LINK_MODE=copy \
    UV_FROZEN=1 \
    ISAACSIM_DIR=/isaac-sim
# uv brings its own Python 3.14 for the exporter; Isaac Sim keeps its own Python.
COPY --from=uv /usr/local/bin/uv /usr/local/bin/uv
WORKDIR /app
COPY pyproject.toml uv.lock .python-version README.md ./
RUN uv sync --no-install-project
COPY . .
RUN uv sync
ENTRYPOINT []
CMD ["./run_main.sh", "--headless"]

# --- exporter --------------------------------------------------------------
FROM ${PYTHON_IMAGE} AS exporter
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python \
    UV_LINK_MODE=copy \
    UV_FROZEN=1
COPY --from=uv /usr/local/bin/uv /usr/local/bin/uv
WORKDIR /app

# Dependencies first so code edits do not reinstall them.
COPY pyproject.toml uv.lock .python-version README.md ./
RUN uv sync --no-install-project

COPY . .
RUN uv sync
ENV PATH=/app/.venv/bin:$PATH
CMD ["./run_main.sh", "--generate-only"]
