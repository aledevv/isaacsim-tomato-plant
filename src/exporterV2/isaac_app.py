"""Open and exercise an already generated canonical ExporterV2 stage in Isaac Sim."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import signal
from pathlib import Path
import sys
import time
from typing import Callable


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--gui-until-close", action="store_true")
    parser.add_argument("--disable-native-observer", action="store_true")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--physics-preset", choices=("locked", "flexible"), required=True)
    parser.add_argument("--physics-hz", type=int, choices=(60, 120, 240, 480, 960), default=480)
    parser.add_argument(
        "--interactive-physics-hz",
        type=int,
        choices=(60, 120, 240, 480, 960),
        default=60,
        help="GUI runtime rate; headless validation continues to use --physics-hz.",
    )
    parser.add_argument("--report", type=Path)
    parser.add_argument("--diagnostic-monitor", action="store_true")
    parser.add_argument("--fruit-experiment", type=Path,
                        help="Opt-in configuration for shared headless/GUI fruit diagnostics")
    args, kit_args = parser.parse_known_args()
    if args.gui_until_close and (args.headless or not args.fruit_experiment):
        parser.error("--gui-until-close requires GUI and --fruit-experiment")
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    # Kit also owns --usd; prevent a bootstrap open followed by a second open.
    sys.argv = [sys.argv[0], *kit_args]
    return args


def _authored_physics_hz(stage) -> int | None:
    """Read the source USD rate before ``World`` applies its runtime override."""

    scenes = [
        prim
        for prim in stage.Traverse()
        if prim.GetTypeName() == "PhysicsScene"
    ]
    if len(scenes) != 1:
        return None
    value = scenes[0].GetAttribute("physxScene:timeStepsPerSecond").Get()
    return None if value is None else int(value)


def _register_runtime_physics_scene(stage) -> str:
    """Synchronize Isaac 4.5's runtime registry with an open USD scene.

    SimulationManager tracks PhysicsScene addition events.  When a ``World``
    is constructed after ``open_stage`` (the safe legacy V2 order), that event
    may already be over and ``get_physics_dt()`` silently falls back to 60 Hz.
    """

    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import PhysxSchema, UsdPhysics

    scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    if len(scenes) != 1:
        raise RuntimeError(
            f"expected exactly one PhysicsScene, found {len(scenes)}"
        )
    prim = scenes[0]
    api = (
        PhysxSchema.PhysxSceneAPI(prim)
        if prim.HasAPI(PhysxSchema.PhysxSceneAPI)
        else PhysxSchema.PhysxSceneAPI.Apply(prim)
    )
    path = str(prim.GetPath())
    # Isaac Sim 4.5 exposes no public refresh for this registry.  Keep the
    # compatibility shim isolated here so a future runtime can replace it.
    SimulationManager._physics_scene_apis.clear()
    SimulationManager._physics_scene_apis[path] = api
    return path


def _timing_metrics(
    *,
    authored_physics_hz: int | None,
    runtime_physics_hz: int,
    render_hz: int,
    render_update_count: int,
    physics_step_count: int,
    simulated_seconds: float,
    wall_seconds: float,
) -> dict:
    """Return consistent GUI/headless timing telemetry."""

    return {
        # Compatibility field retained for existing report consumers.
        "physics_hz": runtime_physics_hz,
        "authored_physics_hz": authored_physics_hz,
        "runtime_physics_hz": runtime_physics_hz,
        "render_hz": render_hz,
        "physics_substeps_per_render": runtime_physics_hz // render_hz,
        "render_update_count": render_update_count,
        "physics_step_count": physics_step_count,
        "simulated_seconds": simulated_seconds,
        "wall_seconds": wall_seconds,
        "render_updates_per_second": (
            render_update_count / wall_seconds if wall_seconds > 0.0 else None
        ),
        "physics_steps_per_second": (
            physics_step_count / wall_seconds if wall_seconds > 0.0 else None
        ),
        "simulation_realtime_ratio": (
            simulated_seconds / wall_seconds if wall_seconds > 0.0 else None
        ),
    }


def _open_stage_and_wait(
    context,
    app,
    usd_path: Path,
    is_stage_loading: Callable[[], bool],
) -> Path | None:
    context.open_stage(str(usd_path))
    while is_stage_loading():
        app.update()
    app.update()
    stage = context.get_stage()
    if stage is None or not stage.GetRootLayer().realPath:
        return None
    return Path(stage.GetRootLayer().realPath).resolve()


def _entity_paths(stage) -> tuple[list[str], str]:
    paths = []
    root_path = ""
    for prim in stage.Traverse():
        attribute = prim.GetAttribute("autotom:entityKind")
        kind = attribute.Get() if attribute else None
        if kind in {"physical_link", "terminal_body"}:
            path = str(prim.GetPath())
            paths.append(path)
            if kind == "physical_link" and prim.GetChild("RootFixedJoint"):
                root_path = path
    paths.sort()
    if not paths or not root_path:
        raise RuntimeError("stage has no canonical physical bodies or fixed root")
    return paths, root_path


def _diagnostic_metadata(stage) -> dict:
    plants = [
        prim
        for prim in stage.Traverse()
        if prim.GetAttribute("autotom:skeletonSchema").Get()
    ]
    if len(plants) != 1:
        return {"fruit_mode": "unknown"}
    plant = plants[0]

    def value(name, default=None):
        attribute = plant.GetAttribute(name)
        result = attribute.Get() if attribute else None
        return default if result is None else result

    return {
        "fruit_mode": str(value("autotom:fruitMode", "physical")),
        "colliders_enabled": bool(value("autotom:collidersEnabled", True)),
        "drives_enabled": bool(value("autotom:drivesEnabled", True)),
        "articulation_enabled": bool(value("autotom:articulationEnabled", True)),
        "terminal_bodies_physical": bool(
            value("autotom:terminalBodiesPhysical", True)
        ),
    }


def _body_metadata(stage, body_paths: list[str]) -> dict[str, dict]:
    result = {}
    for path in body_paths:
        prim = stage.GetPrimAtPath(path)

        def value(name, default=None):
            attribute = prim.GetAttribute(name)
            item = attribute.Get() if attribute else None
            return default if item is None else item

        result[path] = {
            "role": str(value("autotom:role", "terminal_body")),
            "joint_type": str(value("autotom:jointType", "terminal")),
            "length": float(value("autotom:sourceLength", 0.0)),
            "branch_id": str(value("autotom:branchId", "")),
            "branch_kind": str(value("autotom:branchKind", "")),
            "branch_link_index": int(value("autotom:branchLinkIndex", 0)),
            "branch_link_count": int(value("autotom:branchLinkCount", 0)),
        }
    return result


def _authored_body_geometry(stage) -> tuple[dict[str, tuple[float, float, float]], dict[str, tuple[float, float, float]]]:
    """Capture authored starts/endpoints before ``World.reset()`` starts PhysX."""

    from pxr import Gf, UsdGeom

    paths, _root_path = _entity_paths(stage)
    metadata = _body_metadata(stage, paths)
    starts = {}
    endpoints = {}
    for path in paths:
        matrix = UsdGeom.Xformable(
            stage.GetPrimAtPath(path)
        ).ComputeLocalToWorldTransform(0)
        start = matrix.ExtractTranslation()
        direction = matrix.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)).GetNormalized()
        endpoint = start + direction * metadata[path]["length"]
        starts[path] = tuple(float(value) for value in start)
        endpoints[path] = tuple(float(value) for value in endpoint)
    return starts, endpoints


def _suspend_gravity_for_reset(stage):
    """Avoid counting World.reset's mandatory internal step as pose snapping."""

    from pxr import UsdPhysics

    scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    if len(scenes) != 1:
        raise RuntimeError(
            f"expected exactly one PhysicsScene, found {len(scenes)}"
        )
    scene = UsdPhysics.Scene(scenes[0])
    attribute = scene.GetGravityMagnitudeAttr()
    magnitude = attribute.Get()
    if magnitude is None:
        magnitude = 9.81
        attribute = scene.CreateGravityMagnitudeAttr()
    attribute.Set(0.0)

    def restore() -> None:
        attribute.Set(float(magnitude))

    return restore


def _world_endpoints(positions, orientations, paths, metadata):
    """Return body endpoints from scalar-first world quaternions."""

    import numpy as np

    position_values = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
    quaternion_values = np.asarray(orientations, dtype=np.float64).reshape(-1, 4)
    result = {}
    for path, position, quaternion in zip(
        paths, position_values, quaternion_values, strict=True
    ):
        length = metadata[path]["length"]
        local = np.asarray((0.0, 0.0, length), dtype=np.float64)
        vector = quaternion[1:]
        rotated = local + quaternion[0] * (2.0 * np.cross(vector, local))
        rotated += np.cross(vector, 2.0 * np.cross(vector, local))
        result[path] = position + rotated
    return result


def _configure_mouse_interaction(app, stage, *, grab_with_force=False, force_coefficient=10.0,
                                 allow_experimental_coefficient=False) -> dict:
    """Restore the exact mouse-grab setup used by the interactive legacy V2."""

    maximum = 100.0 if allow_experimental_coefficient else 10.0
    if not 0 < force_coefficient <= maximum:
        raise ValueError(f"mouse force coefficient must be in (0, {maximum}]; native Physics Settings UI range is (0, 10], larger values require explicit experimental configuration")

    import carb.settings

    source_root = Path(__file__).resolve().parents[1]
    if str(source_root) not in sys.path:
        sys.path.insert(0, str(source_root))
    from exporterV2.core.skinning.runtime import configure_physx_mouse_interaction

    settings = carb.settings.get_settings()
    # Enabling the grab UI calls app.update(). After reset the timeline is
    # playing, so that UI update must not integrate an unmonitored physics step.
    playing = settings.get("/app/player/playSimulations")
    settings.set("/app/player/playSimulations", False)
    try:
        configure_physx_mouse_interaction(app)
    finally:
        settings.set("/app/player/playSimulations", playing if playing is not None else True)
    settings.set("/physics/forceGrab", grab_with_force)
    settings.set("/physics/pickingForce", force_coefficient)
    interactive = 0
    for prim in stage.Traverse():
        kind = prim.GetAttribute("autotom:entityKind").Get()
        joint_type = prim.GetAttribute("autotom:jointType").Get()
        if kind == "terminal_body" or (
            kind == "physical_link" and joint_type == "d6"
        ):
            interactive += 1
    return {
        "mouse_interaction_enabled": bool(
            settings.get("/physics/mouseInteractionEnabled")
        ),
        "mouse_grab_enabled": bool(settings.get("/physics/mouseGrab")),
        "mouse_grab_invisible_colliders": not bool(
            settings.get("/physics/mouseGrabIgnoreInvisible")
        ),
        "interactive_body_count": interactive,
    }


def _body_parent_paths(stage, body_paths: list[str]) -> dict[str, str | None]:
    canonical_path = {}
    parent_ids = {}
    for path in body_paths:
        prim = stage.GetPrimAtPath(path)
        primitive_id = prim.GetAttribute("autotom:canonicalPrimitiveId").Get()
        canonical_path[str(primitive_id)] = path
        parent = prim.GetAttribute("autotom:physicalParentId").Get()
        if parent is None:
            parent = prim.GetAttribute("autotom:hostLinkId").Get()
        parent_ids[path] = None if parent is None else str(parent)
    return {
        path: canonical_path.get(parent_id)
        for path, parent_id in parent_ids.items()
    }


def _ancestor_chain(path: str | None, parents: dict[str, str | None]) -> list[str]:
    result = []
    seen = set()
    while path is not None and path not in seen:
        result.append(path)
        seen.add(path)
        path = parents.get(path)
    return result


def _write_report(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _run_stability(stage, world, args: argparse.Namespace, authored_geometry) -> dict:
    import numpy as np
    from isaacsim.core.prims import RigidPrim

    paths, root_path = _entity_paths(stage)
    parents = _body_parent_paths(stage, paths)
    metadata = _diagnostic_metadata(stage)
    body_metadata = _body_metadata(stage, paths)
    bodies = RigidPrim(
        paths,
        name="canonical_v2_stability_bodies",
        reset_xform_properties=False,
        prepare_contact_sensors=False,
    )
    bodies.initialize()
    resolved_paths = list(bodies.prim_paths)
    if set(resolved_paths) != set(paths):
        raise RuntimeError("batch rigid-body view does not cover the authored bodies")

    def poses_by_path():
        positions, orientations = bodies.get_world_poses()
        values = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
        endpoints = _world_endpoints(
            positions, orientations, resolved_paths, body_metadata
        )
        return dict(zip(resolved_paths, values, strict=True)), endpoints

    initial, initial_endpoints = poses_by_path()
    authored_starts, authored_endpoints = authored_geometry
    reset_projection = {
        path: float(np.linalg.norm(initial[path] - authored_starts[path]))
        for path in paths
    }
    reset_endpoint_projection = {
        path: float(
            np.linalg.norm(initial_endpoints[path] - authored_endpoints[path])
        )
        for path in paths
    }
    max_reset_projection = max(reset_projection.values(), default=0.0)
    max_reset_projection_body = max(
        reset_projection, key=reset_projection.get, default=None
    )
    max_reset_endpoint_projection = max(
        reset_endpoint_projection.values(), default=0.0
    )
    max_reset_endpoint_projection_body = max(
        reset_endpoint_projection, key=reset_endpoint_projection.get, default=None
    )
    latest = dict(initial)
    latest_endpoints = dict(initial_endpoints)
    initial_extent = max(float(np.linalg.norm(value)) for value in initial.values())
    explosion_limit = max(5.0, initial_extent * 10.0 + 1.0)
    dt = 1.0 / args.runtime_physics_hz
    steps = max(1, int(math.ceil(args.duration * args.runtime_physics_hz)))
    # A batched full-body sample once per simulated second is sufficient for
    # explosion/detachment checks and avoids making Python telemetry dominate
    # mature 200+ link plants.
    sample_interval = (
        1
        if args.diagnostic_monitor
        else max(1, int(round(1.0 * args.runtime_physics_hz)))
    )
    log_interval = (
        max(1, int(round(0.1 * args.runtime_physics_hz)))
        if args.diagnostic_monitor
        else sample_interval
    )
    samples = []
    errors = []
    reset_projection_limit = 1e-6
    fixed_endpoint_projection_limit = 2e-6
    if max_reset_projection > reset_projection_limit:
        errors.append(
            f"World.reset moved a body by {max_reset_projection:.6g} m: "
            f"{max_reset_projection_body}"
        )
    endpoint_projection_failures = {
        path: error
        for path, error in reset_endpoint_projection.items()
        if error
        > (
            fixed_endpoint_projection_limit
            if body_metadata[path]["joint_type"] == "fixed"
            else reset_projection_limit
        )
    }
    if endpoint_projection_failures:
        failure_body = max(
            endpoint_projection_failures,
            key=endpoint_projection_failures.get,
        )
        errors.append(
            "World.reset moved a body endpoint by "
            f"{endpoint_projection_failures[failure_body]:.6g} m: "
            f"{failure_body}"
        )
    max_displacement = 0.0
    max_root_displacement = 0.0
    max_displacement_body = None
    max_endpoint_displacement = 0.0
    max_endpoint_displacement_body = None
    max_dynamic_sag_ratio = 0.0
    max_dynamic_sag_body = None
    dynamic_sag_by_role = {
        role: {"ratio": 0.0, "body": None}
        for role in ("internode", "petiole", "leaf_rachis", "truss_rachis", "pedicel")
    }
    max_structural_endpoint_drift = 0.0
    first_nonfinite_body = None
    first_failure_body = None
    first_failure_time = None
    spontaneous_terminal_detachment_limit = 0.25
    steps_completed = 0
    simulation_started = float(world.current_time)
    wall_started = time.perf_counter()

    for step in range(steps):
        world.step(render=False)
        steps_completed = step + 1
        if step % sample_interval and step != steps - 1:
            continue
        current, current_endpoints = poses_by_path()
        finite_current = {}
        for path, value in current.items():
            if not np.isfinite(value).all():
                errors.append(f"non-finite body pose: {path}")
                if first_nonfinite_body is None:
                    first_nonfinite_body = path
                    first_failure_body = path
                    first_failure_time = max(
                        0.0, float(world.current_time) - simulation_started
                    )
                continue
            if float(np.linalg.norm(value)) > explosion_limit:
                errors.append(f"body exceeded explosion limit: {path}")
            finite_current[path] = value
            displacement = float(np.linalg.norm(value - initial[path]))
            if displacement > max_displacement:
                max_displacement = displacement
                max_displacement_body = path
            if (
                body_metadata[path]["role"] == "fruit"
                and displacement > spontaneous_terminal_detachment_limit
            ):
                errors.append(
                    "terminal body detached without an applied interaction: "
                    f"{path} moved {displacement:.6g} m"
                )
                if first_failure_body is None:
                    first_failure_body = path
                    first_failure_time = max(
                        0.0, float(world.current_time) - simulation_started
                    )
            endpoint_displacement = float(
                np.linalg.norm(current_endpoints[path] - initial_endpoints[path])
            )
            if endpoint_displacement > max_endpoint_displacement:
                max_endpoint_displacement = endpoint_displacement
                max_endpoint_displacement_body = path
            if body_metadata[path]["joint_type"] == "d6":
                length = body_metadata[path]["length"]
                sag_ratio = (
                    max(0.0, initial_endpoints[path][2] - current_endpoints[path][2])
                    / length
                    if length > 0.0
                    else 0.0
                )
                if sag_ratio > max_dynamic_sag_ratio:
                    max_dynamic_sag_ratio = sag_ratio
                    max_dynamic_sag_body = path
                role_sag = dynamic_sag_by_role.setdefault(
                    body_metadata[path]["role"], {"ratio": 0.0, "body": None}
                )
                if sag_ratio > role_sag["ratio"]:
                    role_sag["ratio"] = sag_ratio
                    role_sag["body"] = path
            elif body_metadata[path]["joint_type"] == "fixed":
                max_structural_endpoint_drift = max(
                    max_structural_endpoint_drift, endpoint_displacement
                )
        current = finite_current
        if root_path in current:
            max_root_displacement = max(
                max_root_displacement,
                float(np.linalg.norm(current[root_path] - initial[root_path])),
            )
        if len(current) != len(paths):
            errors.append("one or more rigid bodies became unreadable")
        elapsed = max(0.0, float(world.current_time) - simulation_started)
        if samples:
            sample_dt = max(elapsed - samples[-1][0], dt)
            max_speed = max(
                float(np.linalg.norm(current[path] - latest[path]) / sample_dt)
                for path in current.keys() & latest.keys()
            )
        else:
            max_speed = 0.0
        samples.append((elapsed, max_speed))
        latest = current
        latest_endpoints = current_endpoints
        if errors or (step + 1) % log_interval == 0 or step == steps - 1:
            print(
                f"[SIM] {elapsed:.1f}/{args.duration:.1f}s "
                f"bodies={len(current)} max_displacement={max_displacement:.6g}m",
                flush=True,
            )
        if errors:
            break

    tail_start = args.duration * 0.8
    tail_speeds = [speed for time, speed in samples if time >= tail_start]
    tail_max_speed = max(tail_speeds, default=0.0)
    root_drift_limit = 1e-3
    locked_projection_limit = 1e-2
    if max_root_displacement > root_drift_limit:
        errors.append(
            f"fixed root drifted by {max_root_displacement:.6g} m"
        )
    if args.physics_preset == "locked" and max_displacement > locked_projection_limit:
        errors.append(
            f"locked structure moved by {max_displacement:.6g} m"
        )
    if args.duration >= 30.0 and tail_max_speed > 0.05:
        errors.append(
            f"persistent oscillation: tail speed {tail_max_speed:.6g} m/s"
        )
    lateral_lengths = {}
    lateral_tip_paths = {}
    for path, item in body_metadata.items():
        if item["branch_kind"] != "lateral_branch":
            continue
        branch_id = item["branch_id"]
        lateral_lengths[branch_id] = lateral_lengths.get(branch_id, 0.0) + item["length"]
        if item["branch_link_index"] == item["branch_link_count"]:
            lateral_tip_paths[branch_id] = path
    lateral_tip_ratios = {}
    for branch_id, path in lateral_tip_paths.items():
        length = lateral_lengths.get(branch_id, 0.0)
        if length > 0.0 and path in latest_endpoints:
            lateral_tip_ratios[branch_id] = float(
                np.linalg.norm(latest_endpoints[path] - initial_endpoints[path]) / length
            )
    max_lateral_tip_ratio = max(lateral_tip_ratios.values(), default=0.0)
    max_lateral_tip_branch = max(
        lateral_tip_ratios, key=lateral_tip_ratios.get, default=None
    )
    lateral_tip_limit = 0.20
    if (
        metadata.get("fruit_mode") == "physical"
        and max_lateral_tip_ratio > lateral_tip_limit
    ):
        errors.append(
            "lateral tip displacement exceeds 20%: "
            f"{max_lateral_tip_branch}={max_lateral_tip_ratio:.6g}"
        )
    wall_seconds = time.perf_counter() - wall_started
    simulated_seconds = samples[-1][0] if samples else 0.0
    timing = _timing_metrics(
        authored_physics_hz=args.authored_physics_hz,
        runtime_physics_hz=args.runtime_physics_hz,
        render_hz=args.render_hz,
        render_update_count=0,
        physics_step_count=steps_completed,
        simulated_seconds=simulated_seconds,
        wall_seconds=wall_seconds,
    )
    return {
        "schema_version": "exporter_v2_stability/1.0",
        "status": "passed" if not errors else "failed",
        "usd": str(args.usd),
        "physics_preset": args.physics_preset,
        **timing,
        **metadata,
        "duration_seconds": args.duration,
        "steps_requested": steps,
        "samples": len(samples),
        "rigid_body_count": len(paths),
        "max_displacement_m": max_displacement,
        "max_displacement_body": max_displacement_body,
        "max_endpoint_displacement_m": max_endpoint_displacement,
        "max_endpoint_displacement_body": max_endpoint_displacement_body,
        "max_dynamic_sag_ratio": max_dynamic_sag_ratio,
        "max_dynamic_sag_body": max_dynamic_sag_body,
        "dynamic_sag_by_role": dynamic_sag_by_role,
        "max_structural_endpoint_drift_m": max_structural_endpoint_drift,
        "max_root_displacement_m": max_root_displacement,
        "reset_projection_limit_m": reset_projection_limit,
        "fixed_endpoint_reset_projection_limit_m": fixed_endpoint_projection_limit,
        "max_reset_projection_m": max_reset_projection,
        "max_reset_projection_body": max_reset_projection_body,
        "max_reset_endpoint_projection_m": max_reset_endpoint_projection,
        "max_reset_endpoint_projection_body": max_reset_endpoint_projection_body,
        "tail_max_speed_mps": tail_max_speed,
        "lateral_tip_displacement_ratios": lateral_tip_ratios,
        "max_lateral_tip_displacement_ratio": max_lateral_tip_ratio,
        "max_lateral_tip_displacement_branch": max_lateral_tip_branch,
        "lateral_tip_displacement_limit": lateral_tip_limit,
        "explosion_limit_m": explosion_limit,
        "root_drift_limit_m": root_drift_limit,
        "locked_initial_projection_limit_m": locked_projection_limit,
        "first_nonfinite_body": first_nonfinite_body,
        "first_failure_body": first_failure_body,
        "first_failure_time_seconds": first_failure_time,
        "first_failure_ancestor_chain": _ancestor_chain(
            first_failure_body, parents
        ),
        "spontaneous_terminal_detachment_limit_m": (
            spontaneous_terminal_detachment_limit
        ),
        "errors": errors,
    }

def _run_gui_monitor(
    stage,
    world,
    app,
    args: argparse.Namespace,
    mouse_interaction: dict,
    authored_geometry,
) -> dict:
    """Render until the user closes Isaac, freezing on the first invalid pose."""

    import numpy as np
    from isaacsim.core.prims import RigidPrim

    paths, root_path = _entity_paths(stage)
    parents = _body_parent_paths(stage, paths)
    body_metadata = _body_metadata(stage, paths)
    bodies = RigidPrim(
        paths,
        name="canonical_v2_gui_monitor_bodies",
        reset_xform_properties=False,
        prepare_contact_sensors=False,
    )
    bodies.initialize()
    resolved_paths = list(bodies.prim_paths)
    positions, orientations = bodies.get_world_poses()
    initial = dict(
        zip(resolved_paths, np.asarray(positions, dtype=np.float64).reshape(-1, 3), strict=True)
    )
    initial_endpoints = _world_endpoints(
        positions, orientations, resolved_paths, body_metadata
    )
    authored_starts, authored_endpoints = authored_geometry
    reset_projection = max(
        float(np.linalg.norm(initial[path] - authored_starts[path]))
        for path in resolved_paths
    )
    reset_endpoint_projection = max(
        float(np.linalg.norm(initial_endpoints[path] - authored_endpoints[path]))
        for path in resolved_paths
    )
    started = time.perf_counter()
    simulation_started = float(world.current_time)
    step = 0
    max_displacement = 0.0
    max_displacement_body = None
    max_endpoint_displacement = 0.0
    max_endpoint_displacement_body = None
    first_nonfinite = None
    failure_time = None
    frozen = False
    while app.app.is_running() and not app.is_exiting():
        if frozen:
            app.update()
            continue
        world.step(render=True)
        step += 1
        simulated_seconds = max(
            0.0, float(world.current_time) - simulation_started
        )
        positions, orientations = bodies.get_world_poses()
        endpoints = _world_endpoints(
            positions, orientations, resolved_paths, body_metadata
        )
        for path, value in zip(
            resolved_paths,
            np.asarray(positions, dtype=np.float64).reshape(-1, 3),
            strict=True,
        ):
            if not np.isfinite(value).all():
                first_nonfinite = path
                failure_time = simulated_seconds
                frozen = True
                print(
                    f"[ERROR] GUI monitor froze at {failure_time:.6f}s: "
                    f"non-finite body {path}",
                    flush=True,
                )
                break
            displacement = float(np.linalg.norm(value - initial[path]))
            if displacement > max_displacement:
                max_displacement = displacement
                max_displacement_body = path
            endpoint_displacement = float(
                np.linalg.norm(endpoints[path] - initial_endpoints[path])
            )
            if endpoint_displacement > max_endpoint_displacement:
                max_endpoint_displacement = endpoint_displacement
                max_endpoint_displacement_body = path
    wall_seconds = time.perf_counter() - started
    simulated_seconds = max(
        0.0, float(world.current_time) - simulation_started
    )
    physics_steps = int(round(simulated_seconds * args.runtime_physics_hz))
    timing = _timing_metrics(
        authored_physics_hz=args.authored_physics_hz,
        runtime_physics_hz=args.runtime_physics_hz,
        render_hz=args.render_hz,
        render_update_count=step,
        physics_step_count=physics_steps,
        simulated_seconds=simulated_seconds,
        wall_seconds=wall_seconds,
    )
    return {
        "schema_version": "exporter_v2_stability/1.0",
        "status": "failed" if first_nonfinite else "closed_by_user",
        "usd": str(args.usd),
        "mode": "gui",
        "physics_preset": args.physics_preset,
        **timing,
        **_diagnostic_metadata(stage),
        **mouse_interaction,
        "rigid_body_count": len(paths),
        "root_body": root_path,
        "max_displacement_m": max_displacement,
        "max_displacement_body": max_displacement_body,
        "max_endpoint_displacement_m": max_endpoint_displacement,
        "max_endpoint_displacement_body": max_endpoint_displacement_body,
        "max_reset_projection_m": reset_projection,
        "max_reset_endpoint_projection_m": reset_endpoint_projection,
        "first_nonfinite_body": first_nonfinite,
        "first_failure_time_seconds": failure_time,
        "first_failure_ancestor_chain": _ancestor_chain(first_nonfinite, parents),
        "errors": [] if first_nonfinite is None else [f"non-finite body pose: {first_nonfinite}"],
    }


def main() -> int:
    args = _arguments()
    usd_path = args.usd.expanduser().resolve()
    args.usd = usd_path
    if not usd_path.is_file():
        print(f"[ERROR] USD stage does not exist: {usd_path}", file=sys.stderr)
        return 2
    experiment_config = None
    if args.fruit_experiment:
        experiment_config = json.loads(args.fruit_experiment.read_text())
        if experiment_config.get('experimental_truss_preset') and args.fruit_experiment.name.endswith('.standard.json'):
            import tempfile
            evidence_root = usd_path.parent / (usd_path.stem + '-runs')
            evidence_root.mkdir(parents=True, exist_ok=True)
            experiment_config['run_dir'] = tempfile.mkdtemp(prefix='run-', dir=evidence_root)
            experiment_config['duration'] = args.duration
            Path(experiment_config['run_dir'], 'config.json').write_text(json.dumps(experiment_config, indent=2)+'\n')
        if hashlib.sha256(usd_path.read_bytes()).hexdigest() != experiment_config["scene_sha256"]:
            raise ValueError("fruit experiment USD does not match its recorded SHA-256")
        previous_report = Path(experiment_config["run_dir"]) / ("report.json" if args.headless else "gui-report.json")
        if previous_report.exists():
            raise ValueError(f"refusing to overwrite previous fruit experiment: {previous_report}")

    args.gui_watchdog = None
    if args.gui_until_close:
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from exporterV2.gui_watchdog import GuiWatchdog
        args.gui_watchdog = GuiWatchdog(experiment_config["run_dir"])
    if args.disable_native_observer and experiment_config:
        experiment_config["record_native_mouse"] = False
    from isaacsim import SimulationApp

    if args.fruit_experiment:
        # Small plant scenes suffer from dispatch overhead with the machine-wide
        # worker count. Apply the same bounded pool to every experimental case.
        sys.argv.append("--/plugins/carb.tasking.plugin/threadCount=4")
    app = SimulationApp({"headless": args.headless, "fast_shutdown": not bool(args.fruit_experiment)})
    if args.gui_until_close:
        args.gui_interrupt_requested = False
        def request_interrupt(signum, frame):
            args.gui_interrupt_requested = True
        signal.signal(signal.SIGINT, request_interrupt)
    try:
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import is_stage_loading

        if args.fruit_experiment:
            import carb.settings
            from omni.physx.bindings._physx import SETTING_NUM_THREADS
            carb.settings.get_settings().set(SETTING_NUM_THREADS, 4)

        context = omni.usd.get_context()
        requested_runtime_hz = (
            args.physics_hz if args.headless else args.interactive_physics_hz
        )
        load_started = time.perf_counter()
        opened_path = _open_stage_and_wait(context, app, usd_path, is_stage_loading)
        if opened_path != usd_path:
            raise RuntimeError(f"opened {opened_path!s} instead of {usd_path}")
        stage = context.get_stage()
        authored_geometry = _authored_body_geometry(stage)
        args.fruit_authored_geometry = authored_geometry
        args.authored_physics_hz = _authored_physics_hz(stage)
        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / requested_runtime_hz,
            rendering_dt=1.0 / 60.0,
        )
        if args.fruit_experiment:
            # World/PhysicsContext defaults overwrite an authored PGS scene
            # with TGS. Restore the requested solver before PhysX is cooked.
            world.get_physics_context().set_solver_type(experiment_config["solver"])
            if "diagnostic_gravity_magnitude" in experiment_config:
                world.get_physics_context().set_gravity(-float(experiment_config["diagnostic_gravity_magnitude"]))
        _register_runtime_physics_scene(stage)
        mouse_interaction = {}
        if not args.headless:
            # Match the old V2 ordering: configure after stage open and again
            # after reset because Kit/PhysX can recreate UI state at reset.
            _configure_mouse_interaction(app, stage)
        restore_gravity = _suspend_gravity_for_reset(stage)
        try:
            world.reset()
        finally:
            restore_gravity()
        # Reset reapplies the Kit loop default (60 Hz) on opened stages.  Set
        # the requested runtime cadence afterwards so headless validation is
        # genuinely 480 Hz while the interactive default remains 60 Hz.
        world.set_simulation_dt(
            physics_dt=1.0 / requested_runtime_hz,
            rendering_dt=1.0 / 60.0,
        )
        args.runtime_physics_hz = int(round(1.0 / world.get_physics_dt()))
        args.render_hz = int(round(1.0 / world.get_rendering_dt()))
        authored_runtime_hz = _authored_physics_hz(stage)
        if (
            authored_runtime_hz != requested_runtime_hz
            or args.runtime_physics_hz != requested_runtime_hz
        ):
            raise RuntimeError(
                "Isaac runtime physics rate mismatch after reset: requested "
                f"{requested_runtime_hz}, authored {authored_runtime_hz}, "
                f"effective {args.runtime_physics_hz}"
            )
        if not args.headless:
            mouse_interaction = _configure_mouse_interaction(
                app, stage,
                grab_with_force=bool(experiment_config and experiment_config.get("mouse_grab_mode") == "force"),
                force_coefficient=experiment_config.get("mouse_force_coefficient", 10.0) if experiment_config else 10.0,
                allow_experimental_coefficient=bool(experiment_config and experiment_config.get("allow_experimental_native_coefficient")),
            )
        load_seconds = time.perf_counter() - load_started
        print(f"[OK] Isaac Sim opened canonical V2 stage: {usd_path}", flush=True)
        print(
            f"[TIMING] authored={args.authored_physics_hz}Hz "
            f"runtime={args.runtime_physics_hz}Hz render={args.render_hz}Hz "
            f"substeps/render={args.runtime_physics_hz // args.render_hz}",
            flush=True,
        )
        report_path = None
        if (args.headless or args.diagnostic_monitor) and not args.fruit_experiment:
            report_path = (
                args.report.expanduser().resolve()
                if args.report is not None
                else usd_path.with_suffix(
                    ".stability.json" if args.headless else ".gui-stability.json"
                )
            )
            _write_report(
                report_path,
                {
                    "schema_version": "exporter_v2_stability/1.0",
                    "status": "running",
                    "mode": "headless" if args.headless else "gui",
                    "usd": str(usd_path),
                    "load_seconds": load_seconds,
                    "authored_physics_hz": args.authored_physics_hz,
                    "runtime_physics_hz": args.runtime_physics_hz,
                    "render_hz": args.render_hz,
                    "physics_substeps_per_render": (
                        args.runtime_physics_hz // args.render_hz
                    ),
                    **_diagnostic_metadata(stage),
                    **mouse_interaction,
                },
            )
        if args.fruit_experiment:
            source_root = str(Path(__file__).resolve().parents[1])
            if source_root not in sys.path:
                sys.path.insert(0, source_root)
            from exporterV2.fruit_diagnostics import run as run_fruit_diagnostics
            config = experiment_config
            if config["hz"] != args.runtime_physics_hz:
                raise ValueError("fruit experiment and runtime frequencies differ")
            report = run_fruit_diagnostics(stage, world, app, args, config)
            return 1 if report["status"] == "failed" else 0
        if args.headless:
            report = _run_stability(stage, world, args, authored_geometry)
            report["load_seconds"] = load_seconds
            _write_report(report_path, report)
            print(
                f"[{'OK' if report['status'] == 'passed' else 'ERROR'}] "
                f"stability={report['status']} report={report_path}",
                flush=True,
            )
            return 0 if report["status"] == "passed" else 1
        if args.diagnostic_monitor:
            report = _run_gui_monitor(
                stage, world, app, args, mouse_interaction, authored_geometry
            )
            report["load_seconds"] = load_seconds
            _write_report(report_path, report)
            print(
                f"[INFO] GUI diagnostic report: {report_path}", flush=True
            )
            return 0 if report["status"] != "failed" else 1
        while app.app.is_running() and not app.is_exiting():
            world.step(render=True)
        return 0
    except KeyboardInterrupt:
        print("[INFO] Isaac Sim interrupted by user.", flush=True)
        return 0
    except Exception as exc:
        import traceback

        print(f"[ERROR] Isaac Sim V2 loader failed: {exc}", file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        if args.gui_watchdog:
            args.gui_watchdog.mark("cleanup.app_close", mode="cleanup")
        app.close()
        if args.gui_watchdog:
            args.gui_watchdog.close()


if __name__ == "__main__":
    exit_code = main()
    # Kit can normalize SystemExit to zero during shutdown.  A headless
    # stability failure must remain observable by shell/CI callers.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)
