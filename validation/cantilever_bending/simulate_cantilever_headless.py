"""
Compatibility wrapper for runtime simulation.

The previous version read authored USD transforms at Default time and was only
safe for visual inspection. This wrapper uses cantilever_validation.py, which
measures runtime PhysX body poses and the actual final tip point.
"""

from __future__ import annotations

import argparse

from cantilever_validation import main


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run cantilever validation simulation")
    parser.add_argument("--gui", type=int, choices=[3, 5, 10, 20], help="Render a new_physics N-link run")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    argv = ["simulate"]
    if args.gui is not None:
        argv.extend(["--n-links", str(args.gui), "--models", "new_physics", "--gui"])
    raise SystemExit(main(argv))
