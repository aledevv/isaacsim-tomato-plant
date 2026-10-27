"""The native recorder must never consume or alter a viewport command."""
import io
from types import SimpleNamespace

from exporterV2.native_drag_observer import NativeDragObserver


def observer(query):
    obj = NativeDragObserver.__new__(NativeDragObserver)
    calls = []
    obj.native = SimpleNamespace(update_interaction=lambda *args: calls.append(args) or 'native-result')
    obj.events = SimpleNamespace(MOUSE_DRAG_BEGAN=1, MOUSE_DRAG_ENDED=3)
    obj.query = query
    obj.time_s = 0
    obj.log = io.StringIO()
    obj.records = {'fruit': {'joint': 'joint'}}
    obj.active = None
    obj.summary = {'grabs': []}
    return obj, calls


def test_forward_original_objects_once_and_track_selected_break():
    obj, calls = observer(lambda *args: {'hit': True, 'rigidBody': 'fruit', 'position': (0, 0, 0)})
    origin, direction = [1, 0, 0], [-1, 0, 0]
    for event in (1, 2):
        assert obj.update_interaction(origin, direction, event) == 'native-result'
    assert obj.handle_break('joint', .5)
    assert not obj.handle_break('other-joint', .5)
    obj.update_interaction(origin, direction, 3)
    assert not obj.handle_break('joint', .6)
    assert len(calls) == 3
    assert all(call[0] is origin and call[1] is direction for call in calls)


def test_logging_failure_still_forwards_native_command():
    def fail(*args):
        raise ValueError('diagnostic raycast failed')
    obj, calls = observer(fail)
    assert obj.update_interaction([1, 0, 0], [-1, 0, 0], 1) == 'native-result'
    assert len(calls) == 1
    assert obj.summary['logging_errors'] == ['diagnostic raycast failed']
