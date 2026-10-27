"""Command line: skeleton + configs -> USDA. Every argument has a default.

    uv run python -m exporterV2            # input/skeleton.json -> output/plant.usda
    uv run python -m exporterV2 --input other.json --output output/other.usda
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from plant_skeleton import SkeletonFormatError, SkeletonValidationError, load_skeleton

from .config import DEFAULT_PHYSICS_PATH, DEFAULT_VISUALS_PATH, PROJECT_ROOT, ConfigError, load_physics_config, load_visuals_config


DEFAULT_INPUT_PATH = PROJECT_ROOT / "input" / "skeleton.json"
DEFAULT_OUTPUT_PATH = PROJECT_ROOT / "output" / "plant.usda"


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build an articulated Isaac Sim USDA from a plant skeleton.")
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT_PATH, help="plant_skeleton/1.0 JSON (default: input/skeleton.json)")
    parser.add_argument("--physics", type=Path, default=DEFAULT_PHYSICS_PATH, help="default: config/physics.yaml")
    parser.add_argument("--visuals", type=Path, default=DEFAULT_VISUALS_PATH, help="default: config/visuals.yaml")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_PATH, help="default: output/plant.usda")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        from .exporter import ExportError, export_plant

        skeleton = load_skeleton(args.input)
        physics = load_physics_config(args.physics)
        visuals = load_visuals_config(args.visuals)
        result = export_plant(skeleton, args.output, physics, visuals)
    except FileNotFoundError as exc:
        print(f"[ERROR] file not found: {exc.filename}", file=sys.stderr)
        return 1
    except (SkeletonFormatError, SkeletonValidationError, ConfigError, ExportError, ValueError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    authored = result.manifest["authored"]
    print(
        f"[OK] {len(skeleton.nodes)} segments -> {authored['rigid_bodies']} rigid bodies, "
        f"{authored['d6_joints']} D6 joints, {authored['leaflets']} leaflets, "
        f"{authored['fruit_spheres']} fruits (scale {result.manifest['metadata']['global_scale']:g})"
    )
    print(f"[OK] USDA: {result.usd_path}")
    print(f"[OK] Manifest: {result.manifest_path}")
    return 0
