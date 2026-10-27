"""Per-physics-step fruit monitoring shared by headless and interactive runs."""
from __future__ import annotations

from collections import deque
import json
import hashlib
import math
import itertools
from pathlib import Path
import time

import numpy as np


def json_finite(value):
    """Represent PhysX's intentional infinite limits without invalid JSON."""
    if isinstance(value, np.generic):
        return json_finite(value.item())
    if isinstance(value, dict):
        return {k: json_finite(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_finite(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


def rotate(q, v):
    """Rotate vectors by scalar-first unit quaternions, with broadcasting."""
    return v + 2 * np.cross(q[..., 1:], np.cross(q[..., 1:], v) + q[..., :1] * v)


def multiply(a, b):
    return np.concatenate((a[..., :1] * b[..., :1] - np.sum(a[..., 1:] * b[..., 1:], axis=-1, keepdims=True),
                           a[..., :1] * b[..., 1:] + b[..., :1] * a[..., 1:] + np.cross(a[..., 1:], b[..., 1:])), axis=-1)


def motion_rates(previous_pos, previous_quat, pos, quat, local_com, dt):
    """Independent COM/rotation finite differences to check solver telemetry."""
    q0, q1 = np.asarray(previous_quat, dtype=float), np.asarray(quat, dtype=float)
    q0 = q0 / np.linalg.norm(q0, axis=1, keepdims=True)
    q1 = q1 / np.linalg.norm(q1, axis=1, keepdims=True)
    # Isaac 4.5 can return COM tensors with an extra singleton dimension
    # (N, 1, 3). Normalize it before broadcasting against (N, 3) body poses.
    local_com = np.asarray(local_com, dtype=float).reshape(len(q0), 3)
    c0 = np.asarray(previous_pos, dtype=float) + rotate(q0, local_com)
    c1 = np.asarray(pos, dtype=float) + rotate(q1, local_com)
    inverse = q0.copy()
    inverse[:, 1:] *= -1
    delta = multiply(q1, inverse)
    return (np.linalg.norm(c1 - c0, axis=1) / dt,
            2 * np.arctan2(np.linalg.norm(delta[:, 1:], axis=1), np.abs(delta[:, 0])) / dt)


def attachment_errors(positions, orientations, parents, children, local0, local1, rot0, rot1):
    p0 = positions[parents] + rotate(orientations[parents], local0)
    p1 = positions[children] + rotate(orientations[children], local1)
    q0, q1 = multiply(orientations[parents], rot0), multiply(orientations[children], rot1)
    dots = np.abs(np.sum(q0 * q1, axis=1) / (np.linalg.norm(q0, axis=1) * np.linalg.norm(q1, axis=1)))
    return np.linalg.norm(p0 - p1, axis=1), 2 * np.arccos(np.clip(dots, 0, 1))


def evaluate_tail(positions, linear, angular, joint_position, joint_angle, completed):
    """Engineering gates, independent of fruit's displacement from its rest pose."""
    # Bounding-box diagonal is a conservative bound on positional excursion.
    excursion = float(np.linalg.norm(np.ptp(positions, axis=0), axis=1).max()) if len(positions) else 0.0
    metrics = {"linear_speed_mps": float(np.max(linear, initial=0)),
               "angular_speed_radps": float(np.max(angular, initial=0)),
               "position_excursion_m": excursion,
               "joint_position_error_m": float(np.max(joint_position, initial=0)),
               "joint_angle_error_rad": float(np.max(joint_angle, initial=0))}
    limits = {"linear_speed_mps": .005, "angular_speed_radps": .05,
              "position_excursion_m": .001, "joint_position_error_m": .0005,
              "joint_angle_error_rad": math.radians(.5)}
    errors = [f"tail {key}={val:.6g} exceeds {limits[key]}" for key, val in metrics.items() if val > limits[key]]
    if not completed:
        errors.append("requested duration not completed")
    return metrics, limits, errors


def unexpected_breaks(broken, target, current_time, force_start, headless):
    """A target breaking before stimulation is still a spontaneous failure."""
    if target is not None:
        return broken - {target} if current_time > force_start else set(broken)
    return set(broken) if headless else set()


def performance_summary(steps, simulated_seconds, wall_seconds, frames):
    """Distinguish physics throughput from measured rendered frame cadence."""
    result = {"physics_steps_per_wall_second": steps / wall_seconds if wall_seconds else None,
              "real_time_factor": simulated_seconds / wall_seconds if wall_seconds else None,
              "loop_wall_seconds": wall_seconds, "rendered_frames": len(frames),
              "gui_fps": None, "gui_steady_fps": None, "gui_steady_p05_fps": None}
    if frames:
        values = np.asarray(frames)
        result["gui_fps"] = float(1 / values[:, 2].mean())
        steady = values[values[:, 0] >= 5]
        if len(steady):
            result["gui_steady_fps"] = float(1 / steady[:, 2].mean())
            result["gui_steady_p05_fps"] = float(1 / np.quantile(steady[:, 2], .95))
    return result


def run(stage, world, app, args, config):
    from pxr import PhysicsSchemaTools, UsdPhysics
    from isaacsim.core.prims import RigidPrim
    from omni.physx import get_physx_interface
    from omni.physx.bindings._physx import SimulationEvent
    from omni.physx.bindings._physx import SETTING_NUM_THREADS
    import carb.settings
    from exporterV2.fruit_experiments import attachment_records, bodies_and_joints, value
    from exporterV2.isaac_app import _ancestor_chain, _write_report

    output = Path(config["run_dir"])
    config = dict(config)
    config["duration"] = args.duration
    manual = getattr(args, "gui_until_close", False)
    watchdog = getattr(args, "gui_watchdog", None)
    def mark(phase, **values):
        if watchdog:
            watchdog.mark(phase, **values)
    config["gui_until_close"] = manual
    termination_reason = "duration_limit"
    last_save = time.monotonic()
    previous_timeline = None
    timeline_invalidated = False
    config["simulation_threads"] = carb.settings.get_settings().get(SETTING_NUM_THREADS)
    config["task_threads"] = carb.settings.get_settings().get("/plugins/carb.tasking.plugin/threadCount")
    config["render_settings"] = {key: app.config.get(key) for key in
                                 ("renderer", "width", "height", "window_width", "window_height")}
    config["effective_mouse_settings"] = {name: carb.settings.get_settings().get(name) for name in
                                           ("/physics/mouseGrab", "/physics/forceGrab", "/physics/pickingForce",
                                            "/physics/mouseGrabIgnoreInvisible")}
    if not args.headless:
        from omni.kit.viewport.utility import get_active_viewport
        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError("GUI measurement requires an active viewport")
        config["viewport_resolution_at_start"] = list(viewport.resolution)
        config["viewport_camera_at_start"] = str(viewport.camera_path)
        if config.get("camera_eye"):
            from isaacsim.core.utils.viewports import set_camera_view
            set_camera_view(eye=np.asarray(config["camera_eye"]), target=np.asarray(config["camera_target"]))
    config["executed_implementation_sha256"] = {
        relative: hashlib.sha256((Path(__file__).resolve().parents[2] / relative).read_bytes()).hexdigest()
        for relative in config.get("implementation_sha256", {})
    }
    output.mkdir(parents=True, exist_ok=True)
    report_path = output / ("gui-report.json" if not args.headless else "report.json")
    prefix = "gui" if not args.headless else "headless"
    paths = sorted(bodies_and_joints(stage)[0])
    view = RigidPrim(paths, name="fruit_diagnostics", reset_xform_properties=False, prepare_contact_sensors=False)
    view.initialize()
    paths = list(view.prim_paths)
    indices = {p: i for i, p in enumerate(paths)}
    records = attachment_records(stage)
    parent_map = {}
    for joint in bodies_and_joints(stage)[1]:
        a, b = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        if b:
            parent_map[str(b[0])] = str(a[0]) if a else None
    roles = [value(stage.GetPrimAtPath(p), "autotom:role", "unknown") for p in paths]
    roots = [i for i, p in enumerate(paths) if stage.GetPrimAtPath(p).GetChild("RootFixedJoint")]
    parents = np.array([indices[r["parent"]] for r in records], dtype=int)
    children = np.array([indices[r["fruit"]] for r in records], dtype=int)
    locals_pos, locals_rot = [], []
    for side in (0, 1):
        joints = [UsdPhysics.Joint(stage.GetPrimAtPath(r["joint"])) for r in records]
        locals_pos.append(np.array([tuple(getattr(j, f"GetLocalPos{side}Attr")().Get()) for j in joints]).reshape(-1, 3))
        locals_rot.append(np.array([(float(q.GetReal()), *q.GetImaginary()) for q in
                                    [getattr(j, f"GetLocalRot{side}Attr")().Get() for j in joints]]).reshape(-1, 4))
    masses = np.asarray(view.get_masses()).copy()
    inertias = np.asarray(view.get_inertias()).copy()
    com_pos, com_rot = view.get_coms()
    mass_errors = []
    scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
    if "diagnostic_gravity_magnitude" in config:
        actual_gravity = float(UsdPhysics.Scene(scene_prim).GetGravityMagnitudeAttr().Get())
        if not np.isclose(actual_gravity, config["diagnostic_gravity_magnitude"], atol=1e-7):
            raise RuntimeError(f"diagnostic gravity mismatch: {actual_gravity}")
    for name, expected in (("physxScene:solverType", config["solver"]),
                           ("physxScene:enableGPUDynamics", config["gpu"]),
                           ("physxScene:timeStepsPerSecond", config["hz"])):
        if value(scene_prim, name) != expected:
            raise RuntimeError(f"runtime configuration mismatch: {name}={value(scene_prim, name)}, expected {expected}")
    articulation_view = world.physics_sim_view.create_articulation_view("/World/Stem")
    if articulation_view.count != 1:
        raise RuntimeError(f"expected one articulation, got {articulation_view.count}")
    metatype = articulation_view.get_metatype(0)
    articulation_info = {"fixed_base": bool(metatype.fixed_base), "links": int(metatype.link_count),
                         "dofs": int(metatype.dof_count), "link_names": list(metatype.link_names)}
    if config.get("expected_articulation_dofs") is not None and metatype.dof_count != config["expected_articulation_dofs"]:
        mass_errors.append(f"unexpected articulation DOFs: {metatype.dof_count}, expected {config['expected_articulation_dofs']}")
    for path, expected in config.get("expected_body_properties", {}).items():
        i = indices[path]
        actual = dict(mass_kg=masses[i], inertia=inertias[i], com=np.asarray(com_pos)[i],
                      com_orientation=np.asarray(com_rot)[i])
        for key, target in expected.items():
            if not np.allclose(np.asarray(actual[key]).reshape(-1), np.asarray(target).reshape(-1), rtol=1e-5, atol=1e-12):
                mass_errors.append(f"unexpected runtime {key}: {path}")
    external_supports = config.get("diagnostic_external_kinematic_supports", [])
    if len(set(external_supports)) != len(external_supports):
        raise ValueError("duplicate diagnostic external supports")
    for path in external_supports:
        prim = stage.GetPrimAtPath(path)
        if (path not in indices or value(prim, "autotom:role") != "diagnostic_support"
                or not value(prim, "physics:kinematicEnabled")
                or prim.GetName() in metatype.link_names):
            raise ValueError(f"invalid diagnostic external kinematic support: {path}")
    expected_links = len(paths) - (len(records) if config["attachment"] == "external" else 0) - len(external_supports)
    if not metatype.fixed_base or metatype.link_count != expected_links:
        mass_errors.append(f"unexpected native articulation topology: {articulation_info['links']} links, expected {expected_links}")
    eigenvalues = np.linalg.eigvalsh(inertias.reshape(-1, 3, 3))
    if not np.isfinite(masses).all() or np.any(masses <= 0) or not np.isfinite(eigenvalues).all() or np.any(eigenvalues <= 0):
        mass_errors.append("invalid effective masses or inertias")
    if not np.isfinite(com_pos).all() or not np.isfinite(com_rot).all():
        mass_errors.append("nonfinite effective center-of-mass frame")
    if not config["collisions"]:
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
        # Keep the cooked mass properties identical to the contact-enabled source.
        view.set_masses(masses)
        view.set_inertias(inertias)
    effective = {"bodies": [{"path": p, "role": roles[i], "mass_kg": float(masses[i]),
                            "inertia": inertias[i].tolist(), "com": np.asarray(com_pos)[i].tolist(),
                            "com_orientation": np.asarray(com_rot)[i].tolist()} for i, p in enumerate(paths)],
                 "scene": {a.GetName(): a.Get() for a in stage.GetPrimAtPath("/World/PhysicsScene").GetAttributes()
                           if a.GetName().startswith("physxScene:") and isinstance(a.Get(), (str, int, float, bool))},
                 "gravity_direction": list(UsdPhysics.Scene(scene_prim).GetGravityDirectionAttr().Get()),
                 "gravity_magnitude_mps2": float(UsdPhysics.Scene(scene_prim).GetGravityMagnitudeAttr().Get()),
                 "solver_iterations": {p: {a.GetName(): a.Get() for a in stage.GetPrimAtPath(p).GetAttributes()
                                            if "solver" in a.GetName().lower() and a.Get() is not None}
                                       for p in ["/World/Stem", *[r["fruit"] for r in records]]},
                 "attachments": [{**record,
                     "break_force_n": value(stage.GetPrimAtPath(record["joint"]), "physics:breakForce"),
                     "break_torque_nm": value(stage.GetPrimAtPath(record["joint"]), "physics:breakTorque"),
                     "excluded": value(stage.GetPrimAtPath(record["joint"]), "physics:excludeFromArticulation"),
                     "effective_mass_ratio": float(masses[indices[record["fruit"]]] / masses[indices[record["parent"]]])}
                     for record in records],
                 "collision_enabled": {str(p.GetPath()): bool(UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get())
                                       for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)},
                 "filtered_pairs": {str(p.GetPath()): [str(t) for t in UsdPhysics.FilteredPairsAPI(p).GetFilteredPairsRel().GetTargets()]
                                    for p in stage.Traverse() if p.HasAPI(UsdPhysics.FilteredPairsAPI)},
                 "articulation": articulation_info, "errors": mass_errors}
    _write_report(output / f"{prefix}-effective.json", json_finite(effective))
    events, errors = [], list(mass_errors)
    observed_break_errors = []
    broken = set()
    start_time = float(world.current_time)
    first_failure = None
    current_time = 0.0
    target_choice = config.get("force_target")
    target_record = None
    gui_interaction = None
    if target_choice:
        order = {"min": 0, "median": len(records) // 2, "max": len(records) - 1}
        if not records:
            raise ValueError("force test requires a fruit")
        if target_choice in order:
            target_record = records[order[target_choice]]
        else:
            target_record = next((r for r in records if r["fruit"] == target_choice), None)
            if target_record is None:
                raise ValueError(f"force target is not an attached fruit: {target_choice}")
    def on_event(event):
        if event.type == int(SimulationEvent.JOINT_BREAK):
            raw = event.payload["jointPath"]
            path = str(PhysicsSchemaTools.decodeSdfPath(raw[0], raw[1]))
            event_time = float(world.current_time) - start_time
            if manual and (timeline_invalidated or event_time < current_time - .5 / hz):
                with (output / "gui-events-after-reset.jsonl").open("a") as reset_log:
                    reset_log.write(json.dumps(dict(kind="joint_break", joint=path,
                        time_s=event_time, monotonic_s=time.monotonic(), classification="after_reset_unvalidated")) + "\n")
                return
            broken.add(path)
            user_target = gui_interaction.handle_break(path, event_time) if gui_interaction else False
            if gui_interaction and not user_target:
                destination = (observed_break_errors if not args.headless and config.get("observe_spontaneous_breaks")
                               else errors)
                destination.append(f"joint broke outside the selected fruit drag: {path}")
            events.append({"time_s": event_time, "joint": path, "user_target": user_target,
                           "kind": "joint_break", "applied_force_n":
                           None if ((target_record and config.get("interaction") == "native")
                                    or (not args.headless and config.get("mouse_grab_mode") != "bounded")) else applied_force})
            print(f"[FRUIT] joint_break t={current_time:.6f} {path}", flush=True)
    # Deliver events on the Python simulation thread after the step. A push
    # subscriber can run on a PhysX worker while the main thread holds the GIL.
    event_stream = get_physx_interface().get_simulation_event_stream_v2()
    subscription = event_stream.create_subscription_to_pop(on_event)
    initial_pos, initial_rot = view.get_world_poses()
    initial_pos = np.asarray(initial_pos).copy()
    previous_pos, previous_quat = initial_pos.copy(), np.asarray(initial_rot).copy()
    previous_velocity = np.asarray(view.get_velocities()).copy()
    captured_events = 0
    if not np.isfinite(initial_pos).all() or not np.isfinite(initial_rot).all():
        errors.append("nonfinite initial pose")
    authored, _ = args.fruit_authored_geometry
    reset_errors = np.linalg.norm(initial_pos - np.array([authored[p] for p in paths]), axis=1)
    reset_projection_limit = float(config.get("diagnostic_reset_projection_limit_m", 1e-6))
    if not math.isfinite(reset_projection_limit) or not 1e-6 <= reset_projection_limit <= .01:
        raise ValueError("diagnostic reset projection limit must be between 1 um and 10 mm")
    if reset_errors.max() > reset_projection_limit:
        errors.append(f"reset projection: {paths[int(reset_errors.argmax())]} {reset_errors.max():.6g} m")
    if reset_projection_limit > 1e-6 and np.max(reset_errors[roots], initial=0.) > 1e-6:
        errors.append("fixed root reset projection exceeds 1 um")
    hz = args.runtime_physics_hz
    tail = deque(maxlen=10 * hz)
    trace, summaries = [], []
    dof_trace = []
    if config.get('record_articulation_dofs'):
        _write_report(output / f'{prefix}-dofs.json', dict(
            paths=articulation_view.dof_paths,
            limits=np.asarray(articulation_view.get_dof_limits()).tolist(),
            initial=np.asarray(articulation_view.get_dof_positions()).tolist(),
            joint_names=list(metatype.joint_names),
            joint_types=[str(t) for t in metatype.joint_types],
            stiffnesses=np.asarray(articulation_view.get_dof_stiffnesses()).tolist(),
            dampings=np.asarray(articulation_view.get_dof_dampings()).tolist(),
            angular_units='radians'))
    chunk = 0
    applied_force = 0.0
    peak_lin, peak_ang = np.zeros(len(paths)), np.zeros(len(paths))
    first_threshold = None
    max_anchor_error = 0.0
    wall_start = time.perf_counter()
    from exporterV2.realtime_pacing import RealtimePacer
    pacer = RealtimePacer(1 / hz) if not args.headless and config.get('gui_real_time', False) else None
    timing = {"physics_step_s": 0.0, "tensor_read_s": 0.0, "analysis_s": 0.0, "render_s": 0.0}
    timing['pacing_wait_s'] = 0.0
    frames = []
    last_frame_saved = 0
    last_frame_wall = wall_start
    _write_report(report_path, {"status": "running", "config": config, "paths": paths})
    def flush_trace():
        nonlocal chunk
        if trace:
            if dof_trace:
                np.savez_compressed(output / f'{prefix}-dofs-{chunk:04d}.npz',
                                    positions=np.stack(dof_trace))
                dof_trace.clear()
            np.savez_compressed(output / f"{prefix}-trace-{chunk:04d}.npz", paths=np.array(paths),
                                state=np.stack(trace), columns=np.array(["x", "y", "z", "qw", "qx", "qy", "qz", "vx", "vy", "vz", "wx", "wy", "wz"]))
            trace.clear()
            chunk += 1
    steps = int(math.ceil(args.duration * hz))
    interaction = None
    if target_record:
        from exporterV2.fruit_interaction import InteractionReplay
        interaction = InteractionReplay(config, view, indices[target_record["fruit"]], target_record,
                                        output / f"{prefix}-interaction.jsonl",
                                        np.asarray(effective["gravity_direction"]) * effective["gravity_magnitude_mps2"])
    elif not args.headless and config.get("mouse_grab_mode") == "bounded":
        from exporterV2.fruit_interaction import GuiDragBridge
        gui_interaction = GuiDragBridge(stage, view, paths, records, broken, output / "gui-interaction.jsonl", config,
                                       np.asarray(effective["gravity_direction"]) * effective["gravity_magnitude_mps2"])
    elif not args.headless and config.get("record_native_mouse"):
        from exporterV2.native_drag_observer import NativeDragObserver
        gui_interaction = NativeDragObserver(records, output / "gui-native-interaction.jsonl", watchdog=watchdog)
    gate = None
    if config.get("arm_after_settle"):
        from exporterV2.settling_gate import SettlingGate
        if not config["breakable"] or not records:
            raise ValueError("Settling gate requires detachable fruit")
        gate = SettlingGate()
        gate_indices = [i for i, role in enumerate(roles) if role in ("truss_rachis", "pedicel")]
        if not gate_indices:
            raise ValueError("No support bodies to monitor for arming")
        print("[FRUIT] Detachment disabled until supports settle; drag remains native.", flush=True)
    if errors:
        first_failure = {"time_s": 0.0, "error": errors[0]}
        steps = 0
    try:
        for step in (itertools.count() if manual and not errors else range(steps)):
            if manual and getattr(args, "gui_interrupt_requested", False):
                termination_reason = "ctrl_c"
                break
            if not args.headless and (not app.app.is_running() or app.is_exiting()):
                termination_reason = "window_close"
                break
            timeline_state = "playing" if world.is_playing() else ("stopped" if world.is_stopped() else "paused")
            if timeline_state != previous_timeline:
                mark("timeline." + timeline_state, timeline=timeline_state, mode="running" if world.is_playing() else "paused")
                previous_timeline = timeline_state
            if manual and (not world.is_playing() or timeline_invalidated):
                if pacer:
                    pacer.reset()
                mark("render.paused", mode="paused")
                app.update()
                mark("idle.paused", mode="paused")
                continue
            if not world.is_playing():
                errors.append("timeline stopped before completion")
                break
            if pacer:
                mark('pacing', mode='running')
                timing['pacing_wait_s'] += pacer.wait(current_time)
            applied_force = interaction.before_step(current_time, 1 / hz, broken) if interaction else 0.0
            if gui_interaction:
                applied_force = gui_interaction.before_step(current_time, 1 / hz, broken)
            step_started = time.perf_counter()
            mark("physics", mode="running", simulation_s=current_time, frame=len(frames))
            world.step(render=False)
            mark("event_pump")
            physics_done = time.perf_counter()
            next_time = float(world.current_time) - start_time
            if manual and next_time < current_time:
                if app.is_exiting() or not app.app.is_running():
                    termination_reason = "window_close"
                    break
                timeline_invalidated = True
                mark("timeline.reset", mode="paused", timeline="reset", simulation_s=current_time)
                print("[FRUIT] Timeline reset: physical recording stopped. Close the window explicitly; use a fresh run for further testing.", flush=True)
                continue
            current_time = next_time
            event_stream.pump()
            mark("pose_read", simulation_s=current_time)
            pos, quat = view.get_world_poses()
            pos, quat, velocities = np.asarray(pos), np.asarray(quat), np.asarray(view.get_velocities())
            state = np.concatenate((pos, quat, velocities), axis=1).astype(np.float32)
            mark("analysis")
            reads_done = time.perf_counter()
            timing["physics_step_s"] += physics_done - step_started
            timing["tensor_read_s"] += reads_done - physics_done
            finite = np.isfinite(state).all(axis=1) & (np.linalg.norm(quat, axis=1) > 1e-8)
            if not finite.all():
                errors.append(f"nonfinite state: {paths[int(np.flatnonzero(~finite)[0])]}")
                first_failure = {"time_s": current_time, "body": paths[int(np.flatnonzero(~finite)[0])]}
                np.savez_compressed(output / f"{prefix}-failure-state.npz", time_s=current_time,
                                    paths=np.asarray(paths), state=state, finite=finite)
                break
            speed, angular = np.linalg.norm(velocities[:, :3], axis=1), np.linalg.norm(velocities[:, 3:], axis=1)
            # Optional diagnostic bound includes detached bodies: native picking
            # may remain active on them after the attachment breaks.
            speed_bound = config.get("diagnostic_all_body_speed_limit_m_s")
            # Unlimited GUI sessions can accumulate arbitrarily large, valid
            # free-fall speeds in scenes without a floor. Allow gravity's full
            # elapsed-time contribution while retaining the runaway guard.
            if speed_bound is not None:
                speed_bound += abs(effective['gravity_magnitude_mps2']) * current_time
            if speed_bound is not None and speed.max() > speed_bound:
                body_index = int(np.argmax(speed))
                errors.append(f"all-body diagnostic speed limit exceeded: {paths[body_index]}")
                first_failure = {"time_s": current_time, "body": paths[body_index],
                                 "speed_m_s": float(speed[body_index]), "limit_m_s": speed_bound}
                np.savez_compressed(output / f"{prefix}-failure-state.npz", time_s=current_time,
                                    paths=np.asarray(paths), state=state)
                break
            motion_linear, motion_angular = motion_rates(previous_pos, previous_quat, pos, quat, np.asarray(com_pos), 1 / hz)
            if gate and gate.update(current_time, 1 / hz,
                    float(motion_linear[gate_indices].max()), float(motion_angular[gate_indices].max()),
                    dragging=bool(gui_interaction and getattr(gui_interaction, "active", None))):
                mark("arm_detachment")
                for record in records:
                    UsdPhysics.Joint(stage.GetPrimAtPath(record["joint"])).GetBreakForceAttr().Set(config["break_force"])
                arming = dict(event="detachment_armed", simulation_s=current_time,
                              wall_s=time.perf_counter()-wall_start, break_force_n=config["break_force"],
                              joint_paths=[r["joint"] for r in records], **gate.summary())
                _write_report(output / f"{prefix}-arming.json", arming)
                print(f"[FRUIT] Detachment enabled at {current_time:.3f}s: {config['break_force']:g} N", flush=True)
            for event in events[captured_events:]:
                record = next((r for r in records if r["joint"] == event["joint"]), None)
                if record:
                    index = indices[record["fruit"]]
                    event.update(fruit=record["fruit"], position_before=previous_pos[index].tolist(),
                                 position_after=pos[index].tolist(), velocity_before=previous_velocity[index].tolist(),
                                 velocity_after=velocities[index].tolist(),
                                 pose_step_m=float(np.linalg.norm(pos[index] - previous_pos[index])))
                    # A conservative continuity check: one step's maximum
                    # endpoint speed plus the 1 mm positional screening scale.
                    travel_bound = max(np.linalg.norm(previous_velocity[index, :3]),
                                       np.linalg.norm(velocities[index, :3])) / hz + .001
                    event["continuity_travel_bound_m"] = float(travel_bound)
                    event["continuity_passed"] = bool(event["pose_step_m"] <= travel_bound)
                    if ((target_record and event["joint"] == target_record["joint"]) or event.get("user_target")) and not event["continuity_passed"]:
                        errors.append("target fruit pose discontinuity at joint break")
            captured_events = len(events)
            previous_pos, previous_quat, previous_velocity = pos.copy(), quat.copy(), velocities.copy()
            peak_lin, peak_ang = np.maximum(peak_lin, speed), np.maximum(peak_ang, angular)
            jp, ja = attachment_errors(pos, quat, parents, children, *locals_pos, *locals_rot)
            active = np.array([r["joint"] not in broken for r in records], dtype=bool)
            live_bodies = np.ones(len(paths), dtype=bool)
            for r in records:
                if r["joint"] in broken:
                    live_bodies[indices[r["fruit"]]] = False
            p_error = float(np.max(jp[active], initial=0))
            a_error = float(np.max(ja[active], initial=0))
            if first_threshold is None and (p_error > .0005 or a_error > math.radians(.5)):
                worst = int(np.argmax(np.where(active, jp / .0005 + ja / math.radians(.5), -1)))
                first_threshold = {"time_s": current_time, **records[worst], "position_error_m": float(jp[worst]), "angle_error_rad": float(ja[worst])}
            max_anchor_error = max(max_anchor_error, p_error)
            unexpected = unexpected_breaks(broken, target_record["joint"] if target_record else None,
                                           current_time, config["force_start"], args.headless)
            if unexpected:
                errors.append("spontaneous joint break: " + sorted(unexpected)[0])
            if target_record and config.get("drag_profile") != "hold" and config.get("interaction", "com") in ("com", "surface") and current_time >= config["force_start"] + 5 and target_record["joint"] not in broken:
                errors.append("target fruit did not detach within the 0-12 N, five-second ramp")
            if np.linalg.norm(pos[roots] - initial_pos[roots], axis=1).max() > .001:
                errors.append("fixed root drift exceeds 1 mm")
            if np.linalg.norm(pos[live_bodies] - initial_pos[live_bodies], axis=1).max() > 5:
                errors.append("attached structure displacement exceeds 5 m")
            if step == 0:
                if not np.allclose(view.get_masses(), masses, rtol=1e-6, atol=0) or not np.allclose(view.get_inertias(), inertias, rtol=1e-5, atol=1e-16):
                    errors.append("effective mass/inertia changed after collision switch")
                current_com, current_com_rot = view.get_coms()
                if not np.allclose(current_com, com_pos, rtol=1e-6, atol=1e-9) or not np.allclose(current_com_rot, com_rot, rtol=1e-6, atol=1e-9):
                    errors.append("effective COM frame changed after collision switch")
            # Remove detached fruit from settling gates; its free motion remains in traces.
            tail.append((pos.copy(), speed.copy(), angular.copy(), p_error, a_error,
                         motion_linear.copy(), motion_angular.copy()))
            trace.append(state)
            if config.get('record_articulation_dofs'):
                dof_trace.append(np.asarray(articulation_view.get_dof_positions()).copy())
            summaries.append([current_time, float(speed.max()), float(angular.max()), p_error, a_error, applied_force,
                              float(motion_linear.max()), float(motion_angular.max())])
            timing["analysis_s"] += time.perf_counter() - reads_done
            if (manual and time.monotonic() - last_save >= 1) or (not manual and (step + 1) % (5 * hz) == 0) or errors:
                mark("persistence")
                flush_trace()
                if manual:
                    np.savez_compressed(output / f"gui-frames-part-{chunk:04d}.npz", samples=np.asarray(frames[last_frame_saved:]))
                    last_frame_saved = len(frames)
                    _write_report(output / "gui-partial.json", json_finite(dict(simulation_s=current_time,
                        wall_s=time.perf_counter()-wall_start, events=events, first_failure=first_failure,
                        errors=errors, last_metrics=summaries[-1], frames=len(frames),
                        settling_gate=gate.summary() if gate else None)))
                last_save = time.monotonic()
                mark("analysis")
            if step == 0 or (step + 1) % hz == 0 or errors:
                fps_note = ""
                if frames:
                    recent_frames = frames[-60:]
                    fps_note = f" gui_recent_fps={len(recent_frames) / sum(frame[2] for frame in recent_frames):.2f}"
                print(f"[FRUIT] t={current_time:.2f}/{'until-close' if manual else args.duration} v={speed.max():.5g} w={angular.max():.5g} gap={p_error:.5g} angle_deg={math.degrees(a_error):.5g} breaks={len(broken)}{fps_note}", flush=True)
                if step + 1 == hz:
                    print(f"[FRUIT] profiling={timing}", flush=True)
            if errors:
                if first_failure is None:
                    first_failure = {"time_s": current_time, "error": errors[-1]}
                break
            if not args.headless and (step + 1) % max(1, hz // 60) == 0:
                if gui_interaction:
                    gui_interaction.time_s = current_time
                render_start = time.perf_counter()
                mark("render")
                world.render()
                mark("render_complete")
                frame_wall = time.perf_counter()
                timing["render_s"] += frame_wall - render_start
                frames.append([current_time, frame_wall - wall_start, frame_wall - last_frame_wall,
                               frame_wall - render_start])
                last_frame_wall = frame_wall
    except KeyboardInterrupt:
        termination_reason = "ctrl_c"
        if not manual:
            errors.append("experiment interrupted before completion")
    finally:
        if errors and termination_reason != "timeline_reset":
            termination_reason = "physical_or_validation_error"
        mark("cleanup.monitor", mode="cleanup", termination_reason=termination_reason)
        if interaction:
            interaction.close(current_time)
        if gui_interaction:
            gui_interaction.close(current_time)
        flush_trace()
        subscription = None
    loop_wall_seconds = time.perf_counter() - wall_start
    errors.extend(observed_break_errors)
    if observed_break_errors and first_failure is None:
        first_failure = {"time_s": next(e["time_s"] for e in events if not e.get("user_target")),
                         "error": observed_break_errors[0]}
    if not args.headless and not app.is_exiting():
        config["viewport_resolution_at_end"] = list(viewport.resolution)
        config["viewport_camera_at_end"] = str(viewport.camera_path)
    completed = (manual and termination_reason in ("window_close", "ctrl_c")) or current_time >= args.duration - .5 / hz
    if tail:
        tail_values = [np.array(v) for v in zip(*tail)]
        final_attached = np.ones(len(paths), dtype=bool)
        for record in records:
            if record["joint"] in broken:
                final_attached[indices[record["fruit"]]] = False
        tail_values[0] = tail_values[0][:, final_attached, :]
        for column in (1, 2, 5, 6):
            tail_values[column] = tail_values[column][:, final_attached]
        metrics, limits, gate_errors = evaluate_tail(*tail_values[:5], completed)
        for key, column, limit in (("kinematic_linear_speed_mps", 5, .005), ("kinematic_angular_speed_radps", 6, .05)):
            metrics[key] = float(tail_values[column].max())
            limits[key] = limit
            if metrics[key] > limit:
                gate_errors.append(f"tail {key}={metrics[key]:.6g} exceeds {limit}")
        if first_failure is None and args.headless:
            anomalies = []
            for key, column, limit in (("linear_speed_mps", 1, .005), ("angular_speed_radps", 2, .05),
                                       ("kinematic_linear_speed_mps", 5, .005), ("kinematic_angular_speed_radps", 6, .05)):
                exceeded = np.argwhere(tail_values[column] > limit)
                if exceeded.size:
                    sample, body = exceeded[0]
                    anomalies.append({"time_s": current_time - (len(tail) - 1 - int(sample)) / hz,
                                      "body": np.asarray(paths)[final_attached][body].item(), "metric": key,
                                      "value": float(tail_values[column][sample, body]), "limit": limit})
            if anomalies:
                first_failure = min(anomalies, key=lambda event: event["time_s"])
    else:
        metrics, limits, gate_errors = {}, {}, ["no valid samples"]
    expect_attached = config.get("drag_profile") in ("early-release", "hold")
    if target_record and not expect_attached and target_record["joint"] not in broken:
        errors.append("target fruit did not detach during the requested interaction")
    if target_record and expect_attached and broken:
        errors.append(f"unexpected joint break in {config['drag_profile']} trial")
    if args.headless and config.get("acceptance", "strict") == "strict":
        errors.extend(gate_errors)
    if not completed:
        errors.append(f"simulation ended before the requested {args.duration:g}-second validation completed")
    per_role = {role: {"peak_linear_mps": float(peak_lin[np.array(roles) == role].max()),
                       "peak_angular_radps": float(peak_ang[np.array(roles) == role].max())} for role in sorted(set(roles))}
    if first_failure:
        for record in records:
            if record["joint"] in first_failure.get("error", ""):
                first_failure["body"] = record["fruit"]
                break
        first_failure["support_chain"] = _ancestor_chain(first_failure.get("body"), parent_map)
    if first_threshold:
        first_threshold["support_chain"] = _ancestor_chain(first_threshold["fruit"], parent_map)
    for event in events:
        event["support_chain"] = _ancestor_chain(event.get("fruit"), parent_map)
    np.savez_compressed(output / f"{prefix}-metrics.npz", samples=np.array(summaries),
                        columns=np.array(["time_s", "max_linear_mps", "max_angular_radps", "joint_gap_m", "joint_angle_rad", "applied_force_n",
                                          "kinematic_linear_mps", "kinematic_angular_radps"]))
    if frames:
        np.savez_compressed(output / "gui-frames.npz", samples=np.asarray(frames),
                            columns=np.array(["simulation_s", "wall_s", "frame_interval_s", "render_call_s"]))
    if gate and args.headless and gate.armed_at is None:
        errors.append("supports did not settle: detachment was never enabled")
    performance = performance_summary(len(summaries), current_time, loop_wall_seconds, frames)
    if not args.headless and config.get("acceptance") == "functional":
        if performance["gui_steady_fps"] is None or performance["gui_steady_fps"] < 20:
            errors.append("GUI steady frame rate is unavailable or below the required 20 FPS")
        if not manual and gui_interaction and not any(e.get("user_target") for e in events):
            errors.append("no selected fruit detached during the manual drag review")
    report = {"schema_version": "exporter_v2_fruit_diagnostics/1.1", "status": "failed" if errors else ("passed" if args.headless else "awaiting_user_review"),
              "settling_gate": gate.summary() if gate else None,
              "final_break_forces_n": {r["joint"]: value(stage.GetPrimAtPath(r["joint"]), "physics:breakForce") for r in records},
              "termination_reason": termination_reason,
              "timeline_invalidated": timeline_invalidated,
              "native_break_attribution": "observed" if gui_interaction else "unclassified",
              "config": config, "errors": list(dict.fromkeys(errors)), "events": events,
              "simulated_seconds": current_time, "wall_seconds": time.perf_counter() - wall_start,
              "timing": timing,
              "performance": performance,
              "numerical_gate_errors": gate_errors,
              "acceptance_policy": config.get("acceptance", "strict"),
              "final_attachments": [{**record, "broken": record["joint"] in broken,
                                     "position_error_m": float(jp[i]), "angle_error_rad": float(ja[i])}
                                    for i, record in enumerate(records)] if summaries else [],
              "runtime_physics_hz": hz, "samples": len(summaries), "body_count": len(paths), "fruit_count": len(records),
              "tail": metrics, "limits": limits, "gui_tail_advisories": gate_errors if not args.headless else [],
              "first_failure": first_failure, "first_attachment_threshold": first_threshold,
              "body_parent_map": parent_map,
              "max_attachment_error_m": max_anchor_error, "per_role": per_role,
              "per_body": [{"path": p, "role": roles[i], "parent": parent_map.get(p),
                            "peak_linear_mps": float(peak_lin[i]), "peak_angular_radps": float(peak_ang[i])}
                           for i, p in enumerate(paths)],
              "max_reset_projection_m": float(reset_errors.max()),
              "reset_projection_limit_m": reset_projection_limit,
              "peak_linear_body": paths[int(peak_lin.argmax())],
              "peak_angular_body": paths[int(peak_ang.argmax())], "force_target": target_record,
              "interaction": interaction.summary if interaction else (gui_interaction.summary if gui_interaction else None),
              "user_acceptance": None}
    _write_report(report_path, json_finite(report))
    print(f"[FRUIT] status={report['status']} report={report_path}", flush=True)
    return report
