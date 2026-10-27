#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_DIR="$HOME/isaacsim"

echo "=================================================="
echo " Cantilever Bending Experiment"
echo "=================================================="
echo "Phase 1 isolates the current D6 model against the"
echo "matching rigid-link analytical reference."
echo ""

echo "[1/2] Checking analytical formulas..."
UV_CACHE_DIR=/tmp/uv-cache uv run python $SCRIPT_DIR/cantilever_validation.py formula-check >/dev/null

echo "[2/2] Running the controlled synthetic D6 diagnostic..."
$ISAACSIM_DIR/python.sh $SCRIPT_DIR/cantilever_validation.py all \
  --benchmarks synthetic_solid_40cm \
  --models new_physics \
  --supports fixed \
  --joint-models d6_biaxial \
  --n-links 3,5,10,15,20 \
  --scenarios tip_force_0p05N \
  --force-point geometric_tip \
  --backend cpu \
  --physics-hz 120,240,480

echo "=================================================="
echo "Experiment finished!"
echo "Generated USD files: data/usd_models/physics_tests/"
echo "Results: $SCRIPT_DIR/results/"
echo ""
echo "To view the simulation in Isaac Sim UI, run:"
echo "  $ISAACSIM_DIR/python.sh $SCRIPT_DIR/simulate_cantilever_headless.py --gui 10"
echo ""
echo "To visually compare legacy_current vs new_physics side-by-side, run:"
echo "  $ISAACSIM_DIR/python.sh $SCRIPT_DIR/compare_cantilevers.py --benchmark synthetic_solid_40cm --n 10"
echo "(You can replace 10 with 3 or 20 to view other resolutions)"
echo "=================================================="
