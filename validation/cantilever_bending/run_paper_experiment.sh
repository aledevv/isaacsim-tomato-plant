#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
RUNNER="$SCRIPT_DIR/cantilever_validation.py"
ISAAC_PYTHON="$HOME/isaacsim/python.sh"

cd "$REPO_ROOT"

echo "[paper 1/10] Pure-Python protocol tests and formula audit"
UV_CACHE_DIR=/tmp/uv-cache uv run python -m pytest "$SCRIPT_DIR/test_validation_protocol.py" -q
UV_CACHE_DIR=/tmp/uv-cache uv run python "$RUNNER" formula-check

echo "[paper 2/10] Start dataset: synthetic spatial tip-load series at 240 Hz"
"$ISAAC_PYTHON" "$RUNNER" all \
  --benchmarks synthetic_solid_40cm --models new_physics \
  --supports fixed --joint-models d6_biaxial --n-links 3,5,10,15,20 \
  --scenarios tip_force_0p05N --force-point geometric_tip \
  --backend cpu --physics-hz 240 --max-seconds 30 \
  --solver-position-iterations 32 --solver-velocity-iterations 4

echo "[paper 3/10] Synthetic N20 tip-load timestep series"
"$ISAAC_PYTHON" "$RUNNER" simulate \
  --benchmarks synthetic_solid_40cm --models new_physics \
  --supports fixed --joint-models d6_biaxial --n-links 20 \
  --scenarios tip_force_0p05N --force-point geometric_tip \
  --backend cpu --physics-hz 120,480,960,1920 --max-seconds 30 \
  --solver-position-iterations 32 --solver-velocity-iterations 4 --append-results

echo "[paper 4/10] Synthetic self-weight spatial series at 1920 Hz"
"$ISAAC_PYTHON" "$RUNNER" simulate \
  --benchmarks synthetic_solid_40cm --models new_physics \
  --supports fixed --joint-models d6_biaxial --n-links 3,5,10,15,20 \
  --scenarios self_weight --force-point geometric_tip \
  --backend cpu --physics-hz 1920 --max-seconds 30 \
  --solver-position-iterations 32 --solver-velocity-iterations 4 --append-results

echo "[paper 5/10] Synthetic N20 self-weight check at 960 Hz"
"$ISAAC_PYTHON" "$RUNNER" simulate \
  --benchmarks synthetic_solid_40cm --models new_physics \
  --supports fixed --joint-models d6_biaxial --n-links 20 \
  --scenarios self_weight --force-point geometric_tip \
  --backend cpu --physics-hz 960 --max-seconds 30 \
  --solver-position-iterations 32 --solver-velocity-iterations 4 --append-results

echo "[paper 6/10] Current legacy branch and new-physics comparison"
"$ISAAC_PYTHON" "$RUNNER" all \
  --benchmarks synthetic_solid_40cm --models legacy_current,new_physics \
  --supports fixed --joint-models d6_biaxial --n-links 3,10,20 \
  --scenarios tip_force_0p05N --force-point geometric_tip \
  --backend cpu --physics-hz 1920 --max-seconds 30 \
  --solver-position-iterations 32 --solver-velocity-iterations 4 --append-results

echo "[paper 7/10] Gao spatial series at 1920 Hz"
"$ISAAC_PYTHON" "$RUNNER" all \
  --benchmarks tomato_gao_20cm --models new_physics \
  --supports fixed --joint-models d6_biaxial --n-links 3,5,10,15,20 \
  --scenarios tip_force_0p05N,self_weight --force-point geometric_tip \
  --backend cpu --physics-hz 1920 --max-seconds 60 \
  --solver-position-iterations 32 --solver-velocity-iterations 4 --append-results

echo "[paper 8/10] Gao N20 timestep check at 960 Hz"
"$ISAAC_PYTHON" "$RUNNER" simulate \
  --benchmarks tomato_gao_20cm --models new_physics \
  --supports fixed --joint-models d6_biaxial --n-links 20 \
  --scenarios tip_force_0p05N,self_weight --force-point geometric_tip \
  --backend cpu --physics-hz 960 --max-seconds 60 \
  --solver-position-iterations 32 --solver-velocity-iterations 4 --append-results

echo "[paper 9/10] Recompute acceptance and scientific Markdown report"
UV_CACHE_DIR=/tmp/uv-cache uv run python "$RUNNER" report

echo "[paper 10/10] Generate deterministic report figures"
MPLCONFIGDIR=/tmp/matplotlib UV_CACHE_DIR=/tmp/uv-cache \
  uv run python "$SCRIPT_DIR/generate_report_assets.py"

echo "[paper] Complete"
echo "[paper] Report: $SCRIPT_DIR/docs/CantileverValidationReport.md"
echo "[paper] Evidence: $SCRIPT_DIR/results/cantilever_validation_results.json"
echo "[paper] Figures: $SCRIPT_DIR/docs/assets/"
