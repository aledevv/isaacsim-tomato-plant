"""
Compatibility wrapper for the quantitative cantilever generator.

Run with Isaac Sim:
    ~/isaacsim/python.sh validation/cantilever_bending/test_cantilever_convergence.py
"""

from __future__ import annotations

from cantilever_validation import main


if __name__ == "__main__":
    raise SystemExit(main(["generate"]))
