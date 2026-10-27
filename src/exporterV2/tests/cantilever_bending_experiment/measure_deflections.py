"""
Compatibility wrapper for quantitative runtime measurements.

Run with Isaac Sim:
    ~/isaacsim/python.sh src/exporterV2/tests/cantilever_bending_experiment/measure_deflections.py
"""

from __future__ import annotations

from cantilever_validation import main


if __name__ == "__main__":
    raise SystemExit(main(["simulate"]))
