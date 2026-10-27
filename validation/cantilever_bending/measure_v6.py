"""
Deprecated compatibility wrapper.

The old v6 script used a hardcoded initial Z and is not valid for quantitative
claims. This entrypoint now delegates to the validated measurement runner.
"""

from __future__ import annotations

from cantilever_validation import main


if __name__ == "__main__":
    raise SystemExit(main(["simulate"]))
