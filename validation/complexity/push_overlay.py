"""Display-only viewport overlay and live panel for ``push_bench.py --gui``.

Uses the same ``omni.ui.scene`` pattern as ``exporterV2/fruit_drag_visuals.py``:
thin 3D arrows with a V head, a force label anchored to each arrow, coloured
contact points, and one caption above each plant. A separate panel shows the
phase, live and peak displacement per probe, and a live plot.
"""

from __future__ import annotations

import numpy as np

# omni.ui colours are 0xAABBGGRR.
PROBE_COLOURS = {"lateral_tip": 0xFFFF9919, "truss_tip": 0xFF19CCFF, "leaf_blade": 0xFF4CFF33}
PROBE_NAMES = {"lateral_tip": "lateral tip", "truss_tip": "truss tip", "leaf_blade": "leaf blade"}
ARROW = 0xFF2E2EF0
TEXT = 0xFFFFFFFF
PLOT_COLOURS = (0xFFFFFFFF, 0xFF33A0FF)
METRES_PER_NEWTON = 0.35
MIN_ARROW = 0.06


class PushOverlay:
    def __init__(self, targets, plants: dict[str, dict], history: int = 360):
        from omni import ui
        from omni.kit.viewport.utility import get_active_viewport_window
        from omni.ui_scene import scene as sc

        self.sc, self.ui = sc, ui
        self.targets = targets
        self.labels = list(plants)
        self.window = get_active_viewport_window()
        self.frame = self.window.get_frame("autotom.push_bench")
        self.arrows, self.force_labels, self.label_frames = [], [], []
        with self.frame:
            self.view = sc.SceneView()
            self.window.viewport_api.add_scene_view(self.view)
            with self.view.scene:
                # Contact markers: discs that always face the camera.
                self.markers = []
                for target in targets:
                    marker = sc.Transform()
                    with marker:
                        with sc.Transform(look_at=sc.Transform.LookAt.CAMERA, scale_to=sc.Space.SCREEN):
                            sc.Arc(7, color=PROBE_COLOURS[target[1]], tesselation=32)
                            sc.Arc(7, color=0xFF000000, thickness=1.5, wireframe=True, tesselation=32)
                    self.markers.append(marker)
                self.arrow_group = sc.Transform(visible=False)
                with self.arrow_group:
                    for _ in targets:
                        self.arrows.append([sc.Line([0, 0, 0], [0, 0, 0], color=ARROW, thickness=2.5)
                                            for _ in range(3)])
                        frame = sc.Transform()
                        with frame:
                            with sc.Transform(look_at=sc.Transform.LookAt.CAMERA, scale_to=sc.Space.SCREEN):
                                with sc.Transform(transform=sc.Matrix44.get_translation_matrix(0, 16, 0)):
                                    self.force_labels.append(
                                        sc.Label("", color=TEXT, size=16, alignment=ui.Alignment.CENTER)
                                    )
                        self.label_frames.append(frame)
                for label, info in plants.items():
                    with sc.Transform(transform=sc.Matrix44.get_translation_matrix(*info["caption_at"])):
                        with sc.Transform(look_at=sc.Transform.LookAt.CAMERA, scale_to=sc.Space.SCREEN):
                            sc.Label(info["caption"], color=TEXT, size=20, alignment=ui.Alignment.CENTER)

        self.panel = ui.Window("Push test – live", width=520, height=360)
        self.panel.position_x, self.panel.position_y = 20, 60
        self.history = history
        self.series = {label: [0.0] * history for label in self.labels}
        with self.panel.frame:
            with ui.VStack(spacing=4):
                self.phase_label = ui.Label("", height=22, style={"font_size": 18})
                self.rows = {}
                with ui.HStack(height=20):
                    ui.Label("probe", width=110)
                    ui.Label("force", width=70)
                    for label in self.labels:
                        ui.Label(f"{label} now / peak [mm]")
                for role in dict.fromkeys(t[1] for t in targets):
                    with ui.HStack(height=20):
                        ui.Label(PROBE_NAMES[role], width=110, style={"color": PROBE_COLOURS[role]})
                        force = next(np.linalg.norm(t[4]) for t in targets if t[1] == role)
                        ui.Label(f"{force:.2f} N", width=70)
                        self.rows[role] = {label: ui.Label("") for label in self.labels}
                ui.Label("lateral tip displacement along the push (last 6 s)", height=20)
                with ui.ZStack(height=150):
                    self.plots = [
                        ui.Plot(ui.Type.LINE, -20.0, 100.0, *self.series[label],
                                style={"color": PLOT_COLOURS[i % 2], "background_color": 0x00000000})
                        for i, label in enumerate(self.labels)
                    ]
                legend = "   ".join(f"{label}: {'white' if i % 2 == 0 else 'orange'}"
                                    for i, label in enumerate(self.labels))
                ui.Label(legend, height=18)
                self.speed_label = ui.Label("", height=18)
        self.peaks = {}

    def update(self, points, phase_name, phase_t, pushing, show_arrows, displacement, speed):
        sc = self.sc
        for marker, p in zip(self.markers, points):
            marker.transform = sc.Matrix44.get_translation_matrix(*p.tolist())
        self.arrow_group.visible = bool(show_arrows)
        if show_arrows:
            for i, (p, target) in enumerate(zip(points, self.targets)):
                force = target[4]
                magnitude = float(np.linalg.norm(force))
                direction = force / max(magnitude, 1e-9)
                length = max(MIN_ARROW, METRES_PER_NEWTON * magnitude)
                # Vector applied at the contact point, pointing where it pushes.
                tip = p + direction * length
                side = np.cross(direction, [0.0, 0.0, 1.0])
                side /= max(np.linalg.norm(side), 1e-9)
                head = 0.25 * min(length, 0.08)
                pairs = [(p, tip), (tip, tip - head * direction + 0.5 * head * side),
                         (tip, tip - head * direction - 0.5 * head * side)]
                for line, (start, end) in zip(self.arrows[i], pairs):
                    line.start, line.end = start.tolist(), end.tolist()
                    line.thickness = 3.5 if pushing else 1.5
                self.label_frames[i].transform = sc.Matrix44.get_translation_matrix(*tip.tolist())
                self.force_labels[i].text = f"F = {magnitude:.2f} N" if pushing else ""
        self.phase_label.text = f"{phase_name}   t = {phase_t:+.2f} s"
        for i, target in enumerate(self.targets):
            label, role = target[0], target[1]
            now = 1e3 * float(np.linalg.norm(displacement[i])) if displacement is not None else 0.0
            key = (label, role)
            if phase_name == "SETTLING":
                self.peaks[key] = 0.0
            self.peaks[key] = max(self.peaks.get(key, 0.0), now)
            self.rows[role][label].text = f"{now:6.1f} / {self.peaks[key]:6.1f}"
            if role == "lateral_tip":
                along = 1e3 * float(displacement[i] @ (target[4] / np.linalg.norm(target[4]))) if displacement is not None else 0.0
                series = self.series[label]
                series.pop(0)
                series.append(along)
        for plot, label in zip(self.plots, self.labels):
            plot.set_data(*self.series[label])
        self.speed_label.text = speed

    def close(self):
        self.window.viewport_api.remove_scene_view(self.view)
        self.frame.clear()
        self.panel.visible = False
