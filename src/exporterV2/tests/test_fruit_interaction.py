"""Interaction geometry tests independent of the Isaac runtime."""
import numpy as np
import pytest
import io
import sys
from types import SimpleNamespace

from exporterV2.fruit_interaction import GuiDragBridge, InteractionReplay, LimitedDrag, pick_ray, unit


def test_occluding_support_is_not_accepted_as_fruit():
    calls = []
    def query(origin, direction, distance):
        calls.append((origin, direction))
        assert np.linalg.norm(direction) == pytest.approx(1)
        if len(calls) == 1:
            return {"hit": True, "rigidBody": "/support", "position": [1, 0, 0]}
        return {"hit": True, "rigidBody": "/fruit", "position": [.01, 0, 0], "collision": "/fruit/shape"}
    eye, direction, hit, attempts = pick_ray([0, 0, 0], "/fruit", query)
    assert len(attempts) == 2
    assert attempts[0]["rigid_body"] == "/support"
    assert attempts[-1]["collider"] == "/fruit/shape"
    assert hit == pytest.approx([.01, 0, 0])


def test_occluded_target_fails_instead_of_grabbing_another_body():
    with pytest.raises(ValueError, match="no unobstructed fruit ray"):
        pick_ray([0, 0, 0], "/fruit", lambda *args: {"hit": True, "rigidBody": "/support"})


@pytest.mark.parametrize("vector", [[0, 0, 0], [np.nan, 0, 0], [np.inf, 0, 0]])
def test_invalid_ray_is_rejected(vector):
    with pytest.raises(ValueError, match="invalid interaction ray"):
        unit(vector)


@pytest.mark.parametrize("slew_rate", [2.4, 4.8])
def test_rapid_drag_and_direction_reversal_obey_vector_force_limits(slew_rate):
    drag = LimitedDrag(slew_rate)
    drag.begin([0, 0, 0], [0, 0, 0], [1, 0, 0])
    previous = np.zeros(3)
    for step in range(1200):
        side = 1 if step < 450 else -1
        drag.move(np.array([1., 0., 0.]), [-1, 0, 100 * side])
        force = drag.step(np.zeros(3), 1/60)
        assert np.linalg.norm(force) <= 12 + 1e-9
        assert np.linalg.norm(force - previous) <= slew_rate/60 + 1e-9
        previous = force
    assert np.linalg.norm(previous) == pytest.approx(12)


@pytest.mark.parametrize("slew_rate", [2.4, 4.8])
def test_release_cancels_force_immediately_and_new_grab_starts_at_zero(slew_rate):
    drag = LimitedDrag(slew_rate)
    drag.begin([0, 0, 0], [0, 0, 0], [1, 0, 0])
    drag.move(np.array([1., 0., 0.]), [-1, 0, -1])
    assert np.linalg.norm(drag.step(np.zeros(3), 1/60)) > 0
    drag.end()
    assert drag.step(np.zeros(3), 1/60) == pytest.approx([0, 0, 0])
    drag.begin([0, 0, 2], [0, 0, 2.02], [1, 0, 0])
    assert drag.step(np.array([0, 0, 2]), 1/60) == pytest.approx([0, 0, 0])


def test_ray_parallel_to_drag_plane_cancels_capture():
    drag = LimitedDrag()
    drag.begin([0, 0, 0], [0, 0, 0], [1, 0, 0])
    drag.move(np.array([1., 0., 0.]), [0, 0, 1])
    assert not drag.active


def test_com_hold_keeps_world_force_plateau_then_releases():
    replay = InteractionReplay.__new__(InteractionReplay)
    replay.config = {"force_start": 30., "drag_profile": "hold", "hold_seconds": 10.,
                     "hold_force": 3., "force_direction": [0., .8, -.6]}
    replay.kind, replay.index = "com", 0
    replay.target = {"joint": "/joint", "fruit": "/fruit"}
    replay.started, replay.released = True, False
    replay.drag, replay.log = LimitedDrag(), io.StringIO()
    commands = []
    replay.view = SimpleNamespace(
        get_world_poses=lambda **kw: (np.zeros((1, 3)), np.array([[1., 0, 0, 0]])),
        apply_forces_and_torques_at_pos=lambda **kw: commands.append(kw["forces"][0]))
    assert replay.before_step(30., 1/60, set()) < 3
    for time in (31.25, 35., 39.99):
        assert replay.before_step(time, 1/60, set()) == pytest.approx(3)
        assert commands[-1] == pytest.approx([0., 2.4, -1.8])
    count = len(commands)
    assert replay.before_step(40., 1/60, set()) == 0
    assert replay.before_step(41., 1/60, set()) == 0
    assert replay.released and len(commands) == count


@pytest.mark.parametrize("retain_grip", [False, True])
def test_gui_routes_attached_fruit_exclusively_and_preserves_native_support_drag(monkeypatch, retain_grip):
    from pxr import Usd, UsdGeom
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Camera.Define(stage, "/Camera")
    monkeypatch.setitem(sys.modules, "omni.kit.viewport.utility",
                        SimpleNamespace(get_active_viewport=lambda: SimpleNamespace(camera_path="/Camera")))
    bridge = GuiDragBridge.__new__(GuiDragBridge)
    calls = []
    bridge.native = SimpleNamespace(update_interaction=lambda *args: calls.append(args))
    bridge.events = SimpleNamespace(MOUSE_DRAG_BEGAN=0, MOUSE_DRAG_CHANGED=1, MOUSE_DRAG_ENDED=2)
    bridge.stage, bridge.broken = stage, set()
    bridge.records = {"/fruit": {"fruit": "/fruit", "joint": "/joint"}}
    bridge.indices, bridge.coms = {"/fruit": 0}, np.zeros((1, 3))
    bridge.view = SimpleNamespace(get_world_poses=lambda **kw: (np.zeros((1, 3)), np.array([[1., 0, 0, 0]])))
    bridge.drag, bridge.capture, bridge.record = LimitedDrag(), None, None
    bridge.retain_grip, bridge.free_phase = retain_grip, False
    bridge.visuals = None
    bridge.time_s, bridge.log, bridge.summary = 0., io.StringIO(), {"grabs": [], "peak_command_n": 0.}
    bridge.query = lambda *args: {"hit": True, "rigidBody": "/fruit", "collision": "/fruit/shape", "position": [0, 0, .01]}
    class NativeGuiVector:
        # omni.ui.scene.Vector3 supports iteration, but direct np.asarray fails.
        def __init__(self, values):
            self.values = values
        def __iter__(self):
            return iter(self.values)
        def __array__(self, *args, **kwargs):
            raise ValueError("setting an array element with a sequence")
    bridge.update_interaction(NativeGuiVector([0, 0, 1]), NativeGuiVector([0, 0, -1]), 0)
    bridge.update_interaction([0, 0, 1], [.1, 0, -1], 1)
    assert bridge.drag.active and calls == []
    assert not bridge.handle_break("/another_joint", .2)
    assert bridge.handle_break("/joint", .3)
    assert bridge.drag.active == retain_grip
    assert bridge.free_phase == retain_grip
    bridge.update_interaction([0, 0, 1], [.2, 0, -1], 1)
    forces = []
    if retain_grip:
        from exporterV2.free_fruit_grip import FreeFruitGrip
        inputs = SimpleNamespace(MouseInput=SimpleNamespace(LEFT_BUTTON=0),
                                 KeyboardInput=SimpleNamespace(LEFT_SHIFT=1, RIGHT_SHIFT=2, ESCAPE=3))
        monkeypatch.setitem(sys.modules, "carb", SimpleNamespace(input=inputs))
        monkeypatch.setitem(sys.modules, "carb.input", inputs)
        bridge.input = SimpleNamespace(get_mouse_value=lambda *a: 1,
                                       get_keyboard_value=lambda device, key: key != 3)
        bridge.window = SimpleNamespace(get_mouse=lambda: None, get_keyboard=lambda: None)
        bridge.view.get_velocities = lambda **kw: np.zeros((1, 6))
        bridge.view.apply_forces_and_torques_at_pos = lambda **kw: forces.append(kw["forces"].copy())
        bridge.free_grip, bridge.masses, bridge.gravity = FreeFruitGrip(), [.01], np.array([0, 0, -9.81])
        assert 0 < bridge.before_step(.4, 1/60, {"/joint"}) < .3
        assert len(forces) == 1 and calls == []
    bridge.update_interaction([0, 0, 1], [.2, 0, -1], 2)
    assert not bridge.drag.active and not bridge.free_phase
    if retain_grip:
        assert bridge.before_step(.5, 1/60, {"/joint"}) == 0
        assert len(forces) == 1
    assert calls == []  # Holding after detachment cannot start native grabbing.
    bridge.query = lambda *args: {"hit": True, "rigidBody": "/support"}
    for event in (0, 1, 2):
        bridge.update_interaction([0, 0, 1], [0, 0, -1], event)
    assert [call[-1] for call in calls] == [0, 1, 2]


@pytest.mark.parametrize("hold_after_break", [False, True])
def test_recorded_native_rays_translate_and_release_without_forces(hold_after_break):
    from types import SimpleNamespace
    from exporterV2.fruit_interaction import InteractionReplay
    calls, log = [], []
    replay = InteractionReplay.__new__(InteractionReplay)
    replay.config = {'force_start': 1., 'native_recording': {
        'reference_body_position': [1., 0., 0.], 'source': 'reference',
        'source_log_sha256': 'fixture', 'events': [
            {'relative_time_s': 0., 'event': 'begin', 'origin': [2., 0., 0.], 'direction': [-1., 0., 0.]},
            {'relative_time_s': .1, 'event': 'move', 'origin': [2., 0., 0.], 'direction': [-1., 0., 1.]},
            {'relative_time_s': .2, 'event': 'release', 'origin': [2., 0., 0.], 'direction': [-1., 0., 1.]},
        ]}}
    # No force-application method exists on this fake view.
    replay.view = SimpleNamespace(get_world_poses=lambda **kwargs: (np.array([[11., 0., 0.]]), None))
    replay.index = 0
    replay.target = {'fruit': '/Fruit', 'joint': '/Joint'}
    replay.retain_grip = False
    replay.config['native_replay_hold_after_break'] = hold_after_break
    replay.query = lambda *args: {'hit': True, 'rigidBody': '/Fruit', 'collision': '/Fruit/Sphere', 'position': [11., 0., 0.]}
    replay.native = SimpleNamespace(update_interaction=lambda *args: calls.append(args))
    replay.events = SimpleNamespace(MOUSE_DRAG_BEGAN=0, MOUSE_DRAG_CHANGED=1, MOUSE_DRAG_ENDED=2)
    replay.kind = 'native'
    replay.started = replay.released = False
    replay.replay_cursor = 0
    replay.replay_translation = None
    replay.summary = {}
    replay.drag = SimpleNamespace(end=lambda: None)
    replay.write = log.append
    replay.recorded_native_step(1., 1/60)
    assert calls[0][0] == (12., 0., 0.)
    replay.before_step(1.1, 1/60, {'/Joint'})
    replay.before_step(1.2, 1/60, {'/Joint'})
    assert [c[2] for c in calls] == ([0, 1, 2] if hold_after_break else [0, 2])
    assert replay.released
    assert log[-1]['reason'] == ('recording_complete' if hold_after_break else 'joint_break')
    if hold_after_break:
        assert log[1]['command_force_n'] is None
