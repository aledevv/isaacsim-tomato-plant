"""Display-only feedback for the experimental bounded fruit controller."""
import numpy as np


class DragVisuals:
    def __init__(self):
        from omni.kit.viewport.utility import get_active_viewport_window
        from omni.ui_scene import scene as sc
        self.sc = sc
        self.window = get_active_viewport_window()
        self.frame = self.window.get_frame("autotom.fruit_drag_feedback")
        with self.frame:
            self.view = sc.SceneView()
            self.window.viewport_api.add_scene_view(self.view)
            with self.view.scene:
                self.group = sc.Transform(visible=False)
                with self.group:
                    self.point = sc.Points([[0, 0, 0]], colors=[0xff00ffff], sizes=[10])
                    self.tether = sc.Line([0, 0, 0], [0, 0, 0], color=0x99ffffff, thickness=1)
                    self.arrow = [sc.Line([0, 0, 0], [0, 0, 0], color=0xff00cfff, thickness=3)
                                  for _ in range(3)]
                    self.label_position = sc.Transform()
                    with self.label_position:
                        with sc.Transform(look_at=sc.Transform.LookAt.CAMERA, scale_to=sc.Space.SCREEN):
                            with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0, 24, 0)):
                                self.label = sc.Label("", color=0xffffffff, size=18)
        self.last_message_s = -1.

    def update(self, center, grip, target, force, plane_normal, body, time_s, phase="attached"):
        from omni.kit.viewport.utility import post_viewport_message
        center, grip, target, force = [np.asarray(v, dtype=float) for v in (center, grip, target, force)]
        magnitude = float(np.linalg.norm(force))
        self.group.visible = True
        self.point.positions = [grip.tolist()]
        self.tether.start, self.tether.end = grip.tolist(), target.tolist()
        # The orange arrow shows the actual COM force, at 2.5 cm per newton.
        tip = center + .025 * force
        self.label_position.transform = self.sc.Matrix44.get_translation_matrix(*tip.tolist())
        self.label.text = f"{'Held | ' if phase == 'free' else ''}Force: {magnitude:.2f} N"
        direction = force / magnitude if magnitude > 1e-8 else np.zeros(3)
        side = np.cross(direction, plane_normal)
        if np.linalg.norm(side) > 1e-8:
            side /= np.linalg.norm(side)
        head = min(.015, magnitude * .008)
        pairs = [(center, tip), (tip, tip-head*direction+.5*head*side),
                 (tip, tip-head*direction-.5*head*side)]
        for line, (start, end) in zip(self.arrow, pairs):
            line.start, line.end = start.tolist(), end.tolist()
        if time_s - self.last_message_s >= .25:
            post_viewport_message(self.window, f"{body.rsplit('/', 1)[-1]}  |  Force: {magnitude:.2f} N",
                                  "autotom.fruit_drag")
            self.last_message_s = time_s

    def hide(self, reason):
        from omni.kit.viewport.utility import post_viewport_message
        self.group.visible = False
        if reason == "joint_break":
            post_viewport_message(self.window, "Tomato detached — force released", "autotom.fruit_drag")

    def close(self):
        self.window.viewport_api.remove_scene_view(self.view)
        self.frame.clear()
        self.arrow = []
        self.label = self.label_position = None
        self.group = self.point = self.tether = self.view = self.frame = self.window = None
