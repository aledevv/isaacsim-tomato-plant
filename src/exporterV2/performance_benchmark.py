"""Repeatable, apples-to-apples performance comparison for two V2 USD stages.

Run with Isaac Sim Python, for example::

    ~/isaacsim/python.sh src/exporterV2/performance_benchmark.py \
      --baseline legacy=/tmp/tree_legacy.usda \
      --candidate plantstate=/tmp/tree_candidate.usda \
      --physics-hz 60,120,240,480 \
      --output /tmp/v2_performance.json
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any

SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from exporterV2.isaac_app import (
    _authored_physics_hz,
    _open_stage_and_wait,
    _register_runtime_physics_scene,
)


SCHEMA_VERSION = "exporter_v2_performance_comparison/1.0"
SUPPORTED_PHYSICS_HZ = (60, 120, 240, 480)


class PerformanceBenchmarkError(ValueError):
    """Raised for invalid benchmark inputs or inconsistent Isaac timing."""


def _labeled_stage(value: str) -> tuple[str, Path]:
    label, separator, raw_path = value.partition("=")
    if not separator or not label or not raw_path:
        raise argparse.ArgumentTypeError("expected LABEL=PATH")
    if not label.replace("-", "").replace("_", "").isalnum():
        raise argparse.ArgumentTypeError(
            "stage label may contain only letters, numbers, '-' and '_'"
        )
    return label, Path(raw_path).expanduser().resolve()


def _physics_hz_values(value: str) -> tuple[int, ...]:
    try:
        result = tuple(int(item.strip()) for item in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("physics rates must be integers") from exc
    if not result or any(item not in SUPPORTED_PHYSICS_HZ for item in result):
        raise argparse.ArgumentTypeError(
            "physics rates must be selected from 60,120,240,480"
        )
    if len(set(result)) != len(result):
        raise argparse.ArgumentTypeError("physics rates must be unique")
    return result


def _positive_int(value: str) -> int:
    result = int(value)
    if result < 1:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Benchmark legacy and candidate ExporterV2 USD stages identically."
    )
    parser.add_argument("--baseline", type=_labeled_stage, required=True)
    parser.add_argument("--candidate", type=_labeled_stage, required=True)
    parser.add_argument(
        "--physics-hz",
        type=_physics_hz_values,
        default=SUPPORTED_PHYSICS_HZ,
        help="Comma-separated subset of 60,120,240,480.",
    )
    parser.add_argument("--render-hz", type=_positive_int, default=60)
    parser.add_argument("--render-frames", type=_positive_int, default=60)
    parser.add_argument("--physics-steps", type=_positive_int, default=480)
    parser.add_argument("--warmup-render-frames", type=_positive_int, default=10)
    parser.add_argument("--warmup-physics-steps", type=_positive_int, default=20)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--isaac-python",
        type=Path,
        default=Path(os.environ.get("ISAACSIM_DIR", "~/isaacsim")).expanduser()
        / "python.sh",
        help="Isaac launcher used to isolate each rate in a fresh process.",
    )
    parser.add_argument(
        "--require-candidate-faster",
        action="store_true",
        help="Exit non-zero unless candidate is faster in both measurements at every rate.",
    )
    parser.add_argument("--_worker", action="store_true", help=argparse.SUPPRESS)
    return parser


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def collect_stage_statistics(path: str | Path) -> dict[str, Any]:
    """Return deterministic OpenUSD complexity and physics counts."""

    from pxr import Usd, UsdGeom, UsdPhysics

    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    stage = Usd.Stage.Open(str(source))
    if stage is None:
        raise PerformanceBenchmarkError(f"cannot open USD stage {source}")

    prims = list(stage.Traverse())
    types = Counter(prim.GetTypeName() or "<typeless>" for prim in prims)
    mesh_points = 0
    mesh_faces = 0
    mesh_triangles = 0
    material_targets = set()
    for prim in prims:
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            mesh_points += len(mesh.GetPointsAttr().Get() or ())
            counts = list(mesh.GetFaceVertexCountsAttr().Get() or ())
            mesh_faces += len(counts)
            mesh_triangles += sum(max(0, int(count) - 2) for count in counts)
        binding = prim.GetRelationship("material:binding")
        if binding:
            material_targets.update(str(target) for target in binding.GetTargets())

    return {
        "path": str(source),
        "sha256": _sha256(source),
        "file_bytes": source.stat().st_size,
        "authored_physics_hz": _authored_physics_hz(stage),
        "total_prims": len(prims),
        "prim_types": dict(sorted(types.items())),
        "rigid_bodies": sum(
            prim.HasAPI(UsdPhysics.RigidBodyAPI) for prim in prims
        ),
        "collision_shapes": sum(
            prim.HasAPI(UsdPhysics.CollisionAPI) for prim in prims
        ),
        "d6_joints": types["PhysicsJoint"],
        "fixed_joints": types["PhysicsFixedJoint"],
        "mesh_points": mesh_points,
        "mesh_faces": mesh_faces,
        "mesh_triangles": mesh_triangles,
        "material_count": len(material_targets),
    }


def build_comparison(
    baseline_runs: list[dict[str, Any]],
    candidate_runs: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Pair runs by rate and calculate candidate/baseline speed ratios."""

    baseline_by_hz = {run["physics_hz"]: run for run in baseline_runs}
    candidate_by_hz = {run["physics_hz"]: run for run in candidate_runs}
    if baseline_by_hz.keys() != candidate_by_hz.keys():
        raise PerformanceBenchmarkError("baseline/candidate physics rates differ")
    result = []
    for physics_hz in sorted(baseline_by_hz):
        baseline = baseline_by_hz[physics_hz]
        candidate = candidate_by_hz[physics_hz]
        render_ratio = (
            candidate["render_updates_per_second"]
            / baseline["render_updates_per_second"]
        )
        physics_ratio = (
            candidate["physics_steps_per_second"]
            / baseline["physics_steps_per_second"]
        )
        result.append(
            {
                "physics_hz": physics_hz,
                "candidate_to_baseline_render_ratio": render_ratio,
                "candidate_to_baseline_physics_ratio": physics_ratio,
                "candidate_render_faster": render_ratio > 1.0,
                "candidate_physics_faster": physics_ratio > 1.0,
            }
        )
    return result


def save_report(report: dict[str, Any], path: str | Path) -> Path:
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return destination


def _merge_rate_reports(
    reports: list[dict[str, Any]], physics_hz: tuple[int, ...]
) -> dict[str, Any]:
    if not reports:
        raise PerformanceBenchmarkError("no worker reports to merge")
    by_role: dict[str, dict[str, Any]] = {}
    comparisons = []
    for report in reports:
        if report.get("schema_version") != SCHEMA_VERSION:
            raise PerformanceBenchmarkError("worker report schema mismatch")
        comparisons.extend(report["comparison"])
        worker_hz = report["configuration"]["physics_hz"]
        if len(worker_hz) != 1:
            raise PerformanceBenchmarkError("worker report must contain one rate")
        rate = int(worker_hz[0])
        for stage in report["stages"]:
            role = stage["role"]
            merged = by_role.get(role)
            if merged is None:
                merged = {
                    "role": role,
                    "label": stage["label"],
                    "statistics": stage["statistics"],
                    "open_seconds_by_hz": {},
                    "runs": [],
                }
                by_role[role] = merged
            elif (
                merged["label"] != stage["label"]
                or merged["statistics"] != stage["statistics"]
            ):
                raise PerformanceBenchmarkError(
                    f"inconsistent worker stage metadata for {role}"
                )
            merged["open_seconds_by_hz"][str(rate)] = stage["open_seconds"]
            merged["runs"].extend(stage["runs"])

    configuration = dict(reports[0]["configuration"])
    configuration["physics_hz"] = list(physics_hz)
    stages = [by_role[role] for role in ("baseline", "candidate")]
    for stage in stages:
        stage["runs"].sort(key=lambda run: run["physics_hz"])
    comparisons.sort(key=lambda item: item["physics_hz"])
    return {
        "schema_version": SCHEMA_VERSION,
        "configuration": configuration,
        "stages": stages,
        "comparison": comparisons,
    }


def _run_isolated_rates(args: argparse.Namespace, kit_args: list[str]) -> dict[str, Any]:
    launcher = args.isaac_python.expanduser().resolve()
    if not launcher.is_file():
        raise FileNotFoundError(launcher)
    baseline_label, baseline_path = args.baseline
    candidate_label, candidate_path = args.candidate
    reports = []
    with tempfile.TemporaryDirectory(prefix="autotom-v2-benchmark-", dir="/tmp") as raw:
        temporary = Path(raw)
        for physics_hz in args.physics_hz:
            partial = temporary / f"rate_{physics_hz}.json"
            command = [
                str(launcher),
                str(Path(__file__).resolve()),
                "--baseline",
                f"{baseline_label}={baseline_path}",
                "--candidate",
                f"{candidate_label}={candidate_path}",
                "--physics-hz",
                str(physics_hz),
                "--render-hz",
                str(args.render_hz),
                "--render-frames",
                str(args.render_frames),
                "--physics-steps",
                str(args.physics_steps),
                "--warmup-render-frames",
                str(args.warmup_render_frames),
                "--warmup-physics-steps",
                str(args.warmup_physics_steps),
                "--output",
                str(partial),
                "--_worker",
                *kit_args,
            ]
            completed = subprocess.run(command, check=False)
            if completed.returncode != 0:
                raise PerformanceBenchmarkError(
                    f"Isaac benchmark worker {physics_hz} Hz failed with "
                    f"exit code {completed.returncode}"
                )
            reports.append(json.loads(partial.read_text(encoding="utf-8")))
    return _merge_rate_reports(reports, args.physics_hz)


def _benchmark_rate(
    *,
    world,
    physics_hz: int,
    render_hz: int,
    render_frames: int,
    physics_steps: int,
    warmup_render_frames: int,
    warmup_physics_steps: int,
) -> dict[str, Any]:
    reset_started = time.perf_counter()
    world.reset()
    reset_seconds = time.perf_counter() - reset_started
    # World.reset() reapplies Kit's default 60 Hz loop on an opened stage.  The
    # runtime cadence must therefore be set *after* every reset; this is the
    # same distinction the benchmark is intended to measure.
    world.set_simulation_dt(
        physics_dt=1.0 / physics_hz,
        rendering_dt=1.0 / render_hz,
    )
    authored_runtime_hz = _authored_physics_hz(world.stage)
    actual_render_hz = int(round(1.0 / world.get_rendering_dt()))
    if (authored_runtime_hz, actual_render_hz) != (physics_hz, render_hz):
        raise PerformanceBenchmarkError(
            "Isaac timing mismatch: "
            f"requested {physics_hz}/{render_hz}, "
            f"authored {authored_runtime_hz}/{actual_render_hz}"
        )

    for _ in range(warmup_render_frames):
        world.step(render=True)
    actual_physics_hz = int(round(1.0 / world.get_physics_dt()))
    if actual_physics_hz != physics_hz:
        raise PerformanceBenchmarkError(
            "Isaac runtime timing mismatch after warmup: requested "
            f"{physics_hz}, got {actual_physics_hz}"
        )
    simulated_before = float(world.current_time)
    render_started = time.perf_counter()
    for _ in range(render_frames):
        world.step(render=True)
    render_seconds = time.perf_counter() - render_started
    render_simulated_seconds = float(world.current_time) - simulated_before

    world.reset()
    world.set_simulation_dt(
        physics_dt=1.0 / physics_hz,
        rendering_dt=1.0 / render_hz,
    )
    for _ in range(warmup_physics_steps):
        world.step(render=False)
    physics_started = time.perf_counter()
    for _ in range(physics_steps):
        world.step(render=False)
    physics_seconds = time.perf_counter() - physics_started
    return {
        "physics_hz": physics_hz,
        "render_hz": render_hz,
        "physics_substeps_per_render": physics_hz // render_hz,
        "reset_seconds": reset_seconds,
        "render_update_count": render_frames,
        "render_physics_step_count": render_frames * (physics_hz // render_hz),
        "render_wall_seconds": render_seconds,
        "render_simulated_seconds": render_simulated_seconds,
        "render_updates_per_second": render_frames / render_seconds,
        "physics_step_count": physics_steps,
        "physics_wall_seconds": physics_seconds,
        "physics_steps_per_second": physics_steps / physics_seconds,
    }


def main(argv: list[str] | None = None) -> int:
    args, kit_args = build_argument_parser().parse_known_args(argv)
    baseline_label, baseline_path = args.baseline
    candidate_label, candidate_path = args.candidate
    if baseline_label == candidate_label:
        raise PerformanceBenchmarkError("baseline and candidate labels must differ")
    for path in (baseline_path, candidate_path):
        if not path.is_file():
            raise FileNotFoundError(path)
    if any(rate % args.render_hz for rate in args.physics_hz):
        raise PerformanceBenchmarkError(
            "each physics rate must be divisible by --render-hz"
        )

    # The ordinary uv entry point never has Isaac's Python environment, even
    # for a single requested rate. Always isolate master/worker execution; the
    # worker alone imports SimulationApp.
    if not args._worker:
        report = _run_isolated_rates(args, kit_args)
        destination = save_report(report, args.output)
        print(f"[OK] Performance report: {destination}", flush=True)
        if args.require_candidate_faster and not all(
            item["candidate_render_faster"]
            and item["candidate_physics_faster"]
            for item in report["comparison"]
        ):
            print(
                "[ERROR] Candidate is not faster at every requested rate",
                file=sys.stderr,
            )
            return 1
        return 0

    # SimulationApp owns any remaining Kit arguments.
    sys.argv = [sys.argv[0], *kit_args]
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        print("[BENCH] Isaac startup complete", flush=True)
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import is_stage_loading

        context = omni.usd.get_context()
        stages = []
        for role, label, path in (
            ("baseline", baseline_label, baseline_path),
            ("candidate", candidate_label, candidate_path),
        ):
            print(f"[BENCH] Opening {role} stage: {path}", flush=True)
            World.clear_instance()
            open_started = time.perf_counter()
            opened = _open_stage_and_wait(
                context, app, path, is_stage_loading
            )
            open_seconds = time.perf_counter() - open_started
            if opened != path:
                raise PerformanceBenchmarkError(
                    f"opened {opened!s} instead of {path!s}"
                )
            # Capture immutable source statistics before World/runtime timing
            # authors a session-layer override for the selected cadence.
            statistics = collect_stage_statistics(path)
            first_hz = args.physics_hz[0]
            world = World(
                stage_units_in_meters=1.0,
                physics_dt=1.0 / first_hz,
                rendering_dt=1.0 / args.render_hz,
            )
            _register_runtime_physics_scene(context.get_stage())
            print(f"[BENCH] Created World for {role}", flush=True)
            runs = []
            for physics_hz in args.physics_hz:
                run = _benchmark_rate(
                    world=world,
                    physics_hz=physics_hz,
                    render_hz=args.render_hz,
                    render_frames=args.render_frames,
                    physics_steps=args.physics_steps,
                    warmup_render_frames=args.warmup_render_frames,
                    warmup_physics_steps=args.warmup_physics_steps,
                )
                runs.append(run)
                print(
                    f"[BENCH] {label} {physics_hz}Hz: "
                    f"render={run['render_updates_per_second']:.3f}/s "
                    f"physics={run['physics_steps_per_second']:.3f}/s",
                    flush=True,
                )
            stages.append(
                {
                    "role": role,
                    "label": label,
                    "open_seconds": open_seconds,
                    "statistics": statistics,
                    "runs": runs,
                }
            )

        comparison = build_comparison(stages[0]["runs"], stages[1]["runs"])
        report = {
            "schema_version": SCHEMA_VERSION,
            "configuration": {
                "physics_hz": list(args.physics_hz),
                "render_hz": args.render_hz,
                "render_frames": args.render_frames,
                "physics_steps": args.physics_steps,
                "warmup_render_frames": args.warmup_render_frames,
                "warmup_physics_steps": args.warmup_physics_steps,
                "headless": True,
            },
            "stages": stages,
            "comparison": comparison,
        }
        destination = save_report(report, args.output)
        print(f"[OK] Performance report: {destination}", flush=True)
        if args.require_candidate_faster and not all(
            item["candidate_render_faster"]
            and item["candidate_physics_faster"]
            for item in comparison
        ):
            print("[ERROR] Candidate is not faster at every requested rate", file=sys.stderr)
            return 1
        return 0
    except BaseException as exc:
        # Kit occasionally normalizes an internal SystemExit to status zero.  A
        # benchmark must never look successful without producing its report.
        import traceback

        print(
            f"[ERROR] Performance worker aborted: {type(exc).__name__}: {exc}",
            file=sys.stderr,
            flush=True,
        )
        traceback.print_exc()
        return 1
    finally:
        app.close(wait_for_replicator=False)


if __name__ == "__main__":
    exit_code = main()
    if "--_worker" in sys.argv:
        # Kit can normalize SystemExit to zero.  Workers must preserve failures
        # so the outer orchestrator never accepts a missing/partial report.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(exit_code)
    raise SystemExit(exit_code)
