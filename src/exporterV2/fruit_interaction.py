"""Reproducible native mouse events and controlled force comparisons.

Native picking is queried independently; a ray hit is not a measurement of
the force applied internally by the mouse interactor.
"""
from __future__ import annotations

import json
import numpy as np


def unit(vector):
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid interaction ray")
    return vector / norm


def pick_ray(center, body, query):
    """Find an unobstructed horizontal view of the requested rigid body."""
    center = np.asarray(center, dtype=float)
    attempts = []
    for angle in np.linspace(0, 2 * np.pi, 24, endpoint=False):
        eye = center + .5 * np.array([np.cos(angle), np.sin(angle), 0.])
        direction = unit(center - eye)
        hit = query(tuple(eye), tuple(direction), 1.0)
        attempts.append({"origin": eye.tolist(), "direction": direction.tolist(),
                         "rigid_body": str(hit.get("rigidBody", "")),
                         "collider": str(hit.get("collision", "")), "hit": bool(hit.get("hit"))})
        if hit.get("hit") and str(hit.get("rigidBody")) == body:
            return eye, direction, np.asarray(hit["position"], dtype=float), attempts
    raise ValueError(f"no unobstructed fruit ray for {body}: {attempts}")


class LimitedDrag:
    """A COM spring with a vector force cap and vector slew limit in SI."""
    stiffness = 60.0
    maximum_force = 12.0
    slew_rate = 2.4

    def __init__(self, slew_rate=2.4, damping=0.):
        if not np.isfinite(slew_rate) or slew_rate <= 0:
            raise ValueError("force slew rate must be finite and positive")
        self.slew_rate = float(slew_rate)
        if not np.isfinite(damping) or damping < 0:
            raise ValueError("drag damping must be finite and nonnegative")
        self.damping = float(damping)
        self.active = False
        self.force = np.zeros(3)

    def begin(self, center, hit, normal):
        self.center = np.asarray(center, dtype=float).copy()
        self.previous_center = self.center.copy()
        self.hit = np.asarray(hit, dtype=float).copy()
        self.normal = unit(normal)
        self.target = self.center.copy()
        self.force = np.zeros(3)
        self.active = True

    def move(self, origin, direction):
        direction = unit(direction)
        denominator = np.dot(direction, self.normal)
        if abs(denominator) < 1e-8:
            self.end()
            return
        distance = np.dot(self.hit - origin, self.normal) / denominator
        if distance <= 0 or not np.isfinite(distance):
            self.end()
            return
        point = np.asarray(origin) + distance * direction
        self.target = self.center + point - self.hit

    def step(self, current_center, dt):
        if not self.active:
            return np.zeros(3)
        current_center = np.asarray(current_center, dtype=float)
        velocity = (current_center - self.previous_center) / dt
        self.previous_center = current_center.copy()
        desired = self.stiffness * (self.target - current_center) - self.damping * velocity
        length = np.linalg.norm(desired)
        if length > self.maximum_force:
            desired *= self.maximum_force / length
        delta = desired - self.force
        length = np.linalg.norm(delta)
        if length > self.slew_rate * dt:
            delta *= self.slew_rate * dt / length
        self.force += delta
        return self.force.copy()

    def end(self):
        self.active = False
        self.force = np.zeros(3)


class InteractionReplay:
    def __init__(self, config, view, index, target, output, gravity=(0., 0., -9.81)):
        from omni.physx import get_physx_interface, get_physx_scene_query_interface
        from omni.physx.bindings._physx import PhysicsInteractionEvent
        import carb.settings

        self.config, self.view, self.index, self.target = config, view, index, target
        self.kind = config.get("interaction", "com")
        self.native = get_physx_interface()
        self.query = get_physx_scene_query_interface().raycast_closest
        self.events = PhysicsInteractionEvent
        self.started, self.released = False, False
        self.replay_cursor = 0
        self.replay_translation = None
        self.local_hit = self.eye = self.hit = None
        self.direction = None
        self.drag = LimitedDrag(config.get("drag_slew_rate", 2.4), config.get("drag_damping", 0.))
        from exporterV2.free_fruit_grip import FreeFruitGrip
        self.free_grip, self.free_phase = FreeFruitGrip(), False
        self.retain_grip = config.get("retain_fruit_grip", False)
        self.gravity = np.asarray(gravity, dtype=float)
        self.mass = float(np.asarray(view.get_masses()).reshape(-1)[index])
        self.local_com = np.asarray(view.get_coms()[0])[index].reshape(3)
        self.log = output.open("w")
        self.summary = {"kind": self.kind, "body": target["fruit"],
                        "native_force_newtons": None, "selection": None}
        if self.kind == "bounded":
            self.summary["vector_slew_nps"] = self.drag.slew_rate
            self.summary["drag_damping_ns_per_m"] = self.drag.damping
            self.summary["retain_after_break"] = self.retain_grip
            if self.retain_grip:
                self.summary["free_grip"] = self.free_grip.settings()
        if self.kind == "native":
            settings = carb.settings.get_settings()
            self._native_settings = settings
            for name, value in (("mouseInteractionEnabled", True), ("mouseGrab", True),
                                ("mouseGrabIgnoreInvisible", False),
                                ("forceGrab", config.get("mouse_grab_mode", "force") == "force"),
                                ("pickingForce", config.get("mouse_force_coefficient", 10.0))):
                settings.set("/physics/" + name, value)
            self.summary["native_settings"] = {name: settings.get("/physics/" + name) for name in
                                                ("mouseGrab", "forceGrab", "pickingForce", "mouseGrabIgnoreInvisible")}

    def write(self, value):
        self.log.write(json.dumps(value, allow_nan=False) + "\n")
        self.log.flush()

    def release(self, time_s, reason):
        if self.started and not self.released:
            if self.kind == "native":
                self.native.update_interaction(tuple(self.eye), tuple(self.direction), self.events.MOUSE_DRAG_ENDED)
            self.released = True
            self.drag.end()
            self.write({"time_s": time_s, "event": "end", "reason": reason})

    def before_step(self, time_s, dt, broken):
        from exporterV2.fruit_diagnostics import rotate
        start = self.config["force_start"]
        native_hold = self.kind == "native" and self.config.get("native_replay_hold_after_break", False)
        if self.target["joint"] in broken and not native_hold:
            if not self.retain_grip or self.kind != "bounded":
                self.release(time_s, "joint_break")
                return 0.0
            if not self.free_phase and not self.released:
                self.free_phase = True
                self.write({"time_s": time_s, "event": "free_grip", "body": self.target["fruit"]})
        if time_s + dt < start or self.released:
            return 0.0
        if self.kind == 'native' and self.config.get('native_recording'):
            return self.recorded_native_step(time_s, dt)
        hold = self.config.get("drag_profile") == "hold"
        release_after = self.config.get("hold_seconds", 10.) if hold else (.5 if self.config.get("drag_profile") == "early-release" else 5)
        if (self.kind in ("native", "bounded") or hold) and time_s >= start + release_after:
            self.release(time_s, "gesture_complete")
            return 0.0
        pos, quat = self.view.get_world_poses(indices=np.array([self.index]))
        pos, quat = np.asarray(pos)[0], np.asarray(quat)[0]
        if not self.started:
            self.eye, self.direction, self.hit, attempts = pick_ray(pos, self.target["fruit"], self.query)
            inverse = quat.copy()
            inverse[1:] *= -1
            self.local_hit = rotate(inverse, self.hit - pos)
            self.summary["selection"] = attempts[-1] | {"point": self.hit.tolist(), "attempts": attempts}
            self.write({"time_s": time_s, "event": "begin", **self.summary["selection"]})
            if self.kind == "native":
                self.native.update_interaction(tuple(self.eye), tuple(self.direction), self.events.MOUSE_DRAG_BEGAN)
            elif self.kind == "bounded":
                normal = self.config.get("drag_plane_normal")
                self.drag.begin(pos + rotate(quat, self.local_com), self.hit,
                                -self.direction if normal is None else normal)
            self.started = True
        duration = .2 if self.config.get("drag_profile") == "rapid" else (1.25 if hold else 5)
        fraction = float(np.clip((time_s + dt - start) / duration, 0, 1))
        force_direction = np.asarray(self.config.get("force_direction", [0., 0., -1.]))
        if self.kind in ("native", "bounded"):
            target_point = self.hit + force_direction * self.config.get("drag_distance", .2) * fraction
            self.direction = unit(target_point - self.eye)
            if self.kind == "native":
                self.native.update_interaction(tuple(self.eye), tuple(self.direction), self.events.MOUSE_DRAG_CHANGED)
                command = None
            else:
                self.drag.move(self.eye, self.direction)
                center = pos + rotate(quat, self.local_com)
                if self.free_phase:
                    velocity = np.asarray(self.view.get_velocities(indices=np.array([self.index])))[0, :3]
                    command = self.free_grip.force(center, velocity, self.drag.target, self.mass, self.gravity)
                else:
                    command = self.drag.step(center, dt)
                self.view.apply_forces_and_torques_at_pos(forces=np.asarray([command], dtype=np.float32),
                                                        indices=np.array([self.index], dtype=np.int32), is_global=True)
            self.write({"time_s": time_s, "event": "move", "origin": self.eye.tolist(),
                        "direction": self.direction.tolist(), "target_point": target_point.tolist(),
                        "phase": "free" if self.free_phase else "attached",
                        "target_com": self.drag.target.tolist() if self.kind == "bounded" else None,
                        "com_position": center.tolist() if self.kind == "bounded" else None,
                        "command_force_n": command.tolist() if command is not None else None})
            return float(np.linalg.norm(command)) if command is not None else 0.0
        force = (self.config.get("hold_force", 3.) if hold else 12) * fraction
        position = pos + rotate(quat, self.local_hit) if self.kind == "surface" else None
        self.view.apply_forces_and_torques_at_pos(
            forces=np.asarray([force_direction * force], dtype=np.float32),
            positions=np.asarray([position], dtype=np.float32) if position is not None else None,
            indices=np.array([self.index], dtype=np.int32), is_global=True)
        self.write({"time_s": time_s, "event": "force", "force_n": (force_direction * force).tolist(),
                    "position": position.tolist() if position is not None else "center_of_mass"})
        return force

    def recorded_native_step(self, time_s, dt):
        """Send recorded input rays to PhysX; never applies forces or edits joints."""
        recording = self.config['native_recording']
        events = recording['events']
        if self.replay_translation is None:
            pos, _ = self.view.get_world_poses(indices=np.array([self.index]))
            self.replay_translation = np.asarray(pos)[0] - np.asarray(recording['reference_body_position'])
        elapsed = time_s + dt - self.config['force_start']
        while self.replay_cursor < len(events) and events[self.replay_cursor]['relative_time_s'] <= elapsed + 1e-8:
            event = events[self.replay_cursor]
            self.replay_cursor += 1
            self.eye = np.asarray(event['origin']) + self.replay_translation
            self.direction = unit(event['direction'])
            if event['event'] == 'begin':
                if hasattr(self, '_native_settings'):
                    self.summary['native_settings_at_pick'] = {
                        key: self._native_settings.get('/physics/' + key)
                        for key in ('mouseGrab', 'forceGrab', 'pickingForce')}
                hit = self.query(tuple(self.eye), tuple(self.direction), 10.)
                selection = dict(origin=self.eye.tolist(), direction=self.direction.tolist(),
                    hit=bool(hit.get('hit')), rigid_body=str(hit.get('rigidBody', '')),
                    collider=str(hit.get('collision', '')))
                if not selection['hit'] or selection['rigid_body'] != self.target['fruit']:
                    raise ValueError(f'Retargeted recording did not select intended fruit: {selection}')
                selection['point'] = list(hit['position'])
                self.summary['selection'] = selection
                self.summary['recording'] = dict(source=recording['source'],
                    source_log_sha256=recording['source_log_sha256'],
                    translation_m=self.replay_translation.tolist())
                self.started = True
                self.native.update_interaction(tuple(self.eye), tuple(self.direction), self.events.MOUSE_DRAG_BEGAN)
                self.write(dict(time_s=time_s, event='begin', **selection))
            elif event['event'] == 'move':
                self.native.update_interaction(tuple(self.eye), tuple(self.direction), self.events.MOUSE_DRAG_CHANGED)
                self.write(dict(time_s=time_s, event='move', origin=self.eye.tolist(),
                    direction=self.direction.tolist(), command_force_n=None))
            elif event['event'] == 'release':
                self.release(time_s, 'recording_complete')
                break
        return 0.0

    def close(self, time_s):
        self.release(time_s, "monitor_exit")
        self.log.close()


class GuiDragBridge:
    """Route fruit drags to LimitedDrag, retaining native handling elsewhere.

    The Isaac 4.5 Python overlay resolves its interface factory at event time.
    This process-local adapter restores that factory on close; installed NVIDIA
    files and the global PhysX interface are never modified.
    """
    def __init__(self, stage, view, paths, records, broken, output, config=None, gravity=(0., 0., -9.81)):
        import carb.input
        import omni.appwindow
        import omni.physxui.scripts.physxViewportOverlays as overlay
        from omni.physx import get_physx_interface, get_physx_scene_query_interface
        from omni.physx.bindings._physx import PhysicsInteractionEvent
        self.native = get_physx_interface()
        self.query = get_physx_scene_query_interface().raycast_closest
        self.events = PhysicsInteractionEvent
        self.stage, self.view, self.broken = stage, view, broken
        self.indices = {p: i for i, p in enumerate(paths)}
        self.records = {r["fruit"]: r for r in records}
        self.coms = np.asarray(view.get_coms()[0]).reshape(len(paths), 3)
        self.drag = LimitedDrag((config or {}).get("drag_slew_rate", 2.4), (config or {}).get("drag_damping", 0.))
        from exporterV2.free_fruit_grip import FreeFruitGrip
        self.free_grip, self.free_phase = FreeFruitGrip(), False
        self.retain_grip = (config or {}).get("retain_fruit_grip", False)
        self.gravity = np.asarray(gravity, dtype=float)
        self.masses = np.asarray(view.get_masses()).reshape(-1)
        self.capture, self.record = None, None
        self.time_s = 0.
        self.summary = {"mode": "bounded", "stiffness_npm": 60., "force_cap_n": 12.,
                        "vector_slew_nps": self.drag.slew_rate, "grabs": [], "peak_command_n": 0.,
                        "retain_after_break": self.retain_grip}
        self.summary["drag_damping_ns_per_m"] = self.drag.damping
        if self.retain_grip:
            self.summary["free_grip"] = self.free_grip.settings()
        self.log = output.open("w")
        self.input = carb.input.acquire_input_interface()
        self.window = omni.appwindow.get_default_app_window()
        self.overlay, self.original_factory = overlay, overlay.get_physx_interface
        self.factory = lambda: self
        from exporterV2.fruit_drag_visuals import DragVisuals
        self.visuals = DragVisuals()
        overlay.get_physx_interface = self.factory

    def __getattr__(self, name):
        return getattr(self.native, name)

    def write(self, data):
        self.log.write(json.dumps({"time_s": self.time_s, **data}, allow_nan=False) + "\n")
        self.log.flush()

    def center(self, body):
        from exporterV2.fruit_diagnostics import rotate
        index = self.indices[body]
        positions, quats = self.view.get_world_poses(indices=np.array([index]))
        return np.asarray(positions)[0] + rotate(np.asarray(quats)[0], self.coms[index])

    def update_interaction(self, origin, direction, event):
        from pxr import Gf, UsdGeom
        from omni.kit.viewport.utility import get_active_viewport
        try:
            # omni.ui.scene.Vector3 is iterable but is not NumPy-array compatible.
            # Materialize scalar components before converting the native GUI ray.
            origin, direction = np.asarray(tuple(origin), dtype=float), unit(tuple(direction))
            if not np.isfinite(origin).all():
                raise ValueError("nonfinite ray origin")
        except (TypeError, ValueError) as error:
            self.write({"event": "invalid_ray", "error": str(error)})
            self.cancel("invalid_ray")
            return
        if event == self.events.MOUSE_DRAG_BEGAN:
            self.drag.end()
            self.free_phase = False
            hit = self.query(tuple(origin), tuple(direction), 1e4)
            body = str(hit.get("rigidBody", ""))
            record = self.records.get(body)
            if hit.get("hit") and record and record["joint"] not in self.broken:
                viewport = get_active_viewport()
                camera = self.stage.GetPrimAtPath(viewport.camera_path)
                normal = np.asarray(UsdGeom.Xformable(camera).ComputeLocalToWorldTransform(0).TransformDir(Gf.Vec3d(0, 0, -1)))
                self.capture, self.record = "bounded", record
                from exporterV2.fruit_diagnostics import rotate
                positions, quats = self.view.get_world_poses(indices=np.array([self.indices[body]]))
                inverse = np.asarray(quats)[0].copy()
                inverse[1:] *= -1
                self.local_grip = rotate(inverse, np.asarray(hit["position"]) - np.asarray(positions)[0])
                self.drag.begin(self.center(body), hit["position"], normal)
                grab = {"body": body, "joint": record["joint"], "collider": str(hit.get("collision", "")),
                        "origin": origin.tolist(), "direction": direction.tolist(), "point": list(hit["position"]),
                        "plane_normal": unit(normal).tolist(), "begin_s": self.time_s}
                self.summary["grabs"].append(grab)
                self.write({"event": "begin", **grab})
                print(f"[DRAG] selected={body}", flush=True)
                return
            self.capture, self.record = "native", None
            self.write({"event": "native_begin", "body": body, "origin": origin.tolist(), "direction": direction.tolist()})
        if self.capture == "bounded":
            if event == self.events.MOUSE_DRAG_CHANGED and self.drag.active:
                self.drag.move(origin, direction)
                self.write({"event": "move", "origin": origin.tolist(), "direction": direction.tolist()})
            elif event == self.events.MOUSE_DRAG_ENDED:
                self.cancel("mouse_up")
                self.capture = None
            return
        self.native.update_interaction(tuple(origin), tuple(direction), event)
        if event == self.events.MOUSE_DRAG_ENDED:
            self.capture = None

    def cancel(self, reason):
        if self.drag.active:
            self.write({"event": "cancel", "reason": reason, "body": self.record["fruit"]})
        self.drag.end()
        self.free_phase = False
        if self.visuals:
            self.visuals.hide(reason)

    def handle_break(self, joint, time_s):
        self.time_s = time_s
        authorized = bool(self.capture == "bounded" and self.drag.active and self.record["joint"] == joint)
        if authorized:
            if self.retain_grip:
                self.free_phase = True
                self.write({"event": "free_grip", "body": self.record["fruit"]})
            else:
                self.cancel("joint_break")
        return authorized

    def before_step(self, time_s, dt, broken):
        import carb.input
        self.time_s = time_s
        if not self.drag.active:
            return 0.
        mouse_down = self.input.get_mouse_value(self.window.get_mouse(), carb.input.MouseInput.LEFT_BUTTON)
        keyboard = self.window.get_keyboard()
        shift = any(self.input.get_keyboard_value(keyboard, key) for key in
                    (carb.input.KeyboardInput.LEFT_SHIFT, carb.input.KeyboardInput.RIGHT_SHIFT))
        escape = self.input.get_keyboard_value(keyboard, carb.input.KeyboardInput.ESCAPE)
        if not mouse_down or not shift or escape or (self.record["joint"] in broken and not self.free_phase):
            self.cancel("input_released_or_joint_broken")
            return 0.
        index = self.indices[self.record["fruit"]]
        center = self.center(self.record["fruit"])
        if self.free_phase:
            velocity = np.asarray(self.view.get_velocities(indices=np.array([index])))[0, :3]
            command = self.free_grip.force(center, velocity,
                                           self.drag.target, self.masses[index], self.gravity)
        else:
            command = self.drag.step(center, dt)
        self.view.apply_forces_and_torques_at_pos(forces=np.asarray([command], dtype=np.float32),
                                                indices=np.array([self.indices[self.record["fruit"]]], dtype=np.int32), is_global=True)
        magnitude = float(np.linalg.norm(command))
        self.summary["peak_command_n"] = max(magnitude, self.summary["peak_command_n"])
        self.write({"event": "force", "body": self.record["fruit"], "force_n": command.tolist(),
                    "target_com": self.drag.target.tolist(), "com_position": center.tolist(),
                    "phase": "free" if self.free_phase else "attached"})
        if self.visuals:
            from exporterV2.fruit_diagnostics import rotate
            positions, quats = self.view.get_world_poses(indices=np.array([self.indices[self.record["fruit"]]]))
            grip = np.asarray(positions)[0] + rotate(np.asarray(quats)[0], self.local_grip)
            self.visuals.update(center, grip,
                                self.drag.hit + self.drag.target - self.drag.center,
                                command, self.drag.normal, self.record["fruit"], time_s,
                                phase="free" if self.free_phase else "attached")
        return magnitude

    def close(self, time_s):
        self.time_s = time_s
        self.cancel("monitor_exit")
        if self.visuals:
            self.visuals.close()
            self.visuals = None
        if self.overlay.get_physx_interface is self.factory:
            self.overlay.get_physx_interface = self.original_factory
        self.factory = None  # Break the adapter/factory reference cycle before Kit shutdown.
        self.log.close()
