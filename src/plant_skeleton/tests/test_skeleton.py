from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from plant_skeleton import (
    SkeletonFormatError,
    SkeletonValidationError,
    derive_frame,
    load_skeleton,
    save_skeleton,
    skeleton_errors,
    skeleton_from_dict,
    skeleton_to_dict,
    skeleton_warnings,
)
from plant_skeleton.geometry import closest_fraction, frame_columns, frame_error


PROJECT_ROOT = Path(__file__).resolve().parents[3]
SMALL_PLANT = PROJECT_ROOT / "input" / "examples" / "small_plant.json"


def _data():
    return json.loads(SMALL_PLANT.read_text())


def _errors(data):
    return skeleton_errors(skeleton_from_dict(data, validate=False))


def test_example_files_are_valid():
    small = load_skeleton(SMALL_PLANT)
    assert len(small.nodes) == 16
    assert small.root.id == "stem1"
    for path in [PROJECT_ROOT / "input" / "skeleton.json", *sorted((PROJECT_ROOT / "input" / "examples").glob("tomato_day*.json"))]:
        tomato = load_skeleton(path)
        assert tomato.metadata.source == "groimp"
        assert skeleton_warnings(tomato) == []


def test_save_and_load_round_trip(tmp_path):
    skeleton = load_skeleton(SMALL_PLANT)
    path = save_skeleton(skeleton, tmp_path / "copy.json")
    assert load_skeleton(path) == skeleton
    text = path.read_text()
    assert '"parent": null' in text
    assert '"start": [0.0, 0.0, 0.0]' in text  # vectors stay on one line


def test_optional_fields_are_omitted_when_empty():
    data = skeleton_to_dict(load_skeleton(SMALL_PLANT))
    assert "frame" not in data["nodes"][0]
    assert "axis" not in data["nodes"][0]


def test_unknown_and_missing_fields_are_explained():
    data = _data()
    data["nodes"][0]["colour"] = "green"
    with pytest.raises(SkeletonFormatError, match="unknown fields \\['colour'\\]"):
        skeleton_from_dict(data)
    data = _data()
    del data["nodes"][1]["parent"]
    with pytest.raises(SkeletonFormatError, match="missing \\['parent'\\]"):
        skeleton_from_dict(data)
    data = _data()
    data["schema_version"] = "plant_skeleton/0.9"
    with pytest.raises(SkeletonFormatError, match="schema_version"):
        skeleton_from_dict(data)


def test_validation_lists_every_problem_at_once():
    data = _data()
    data["nodes"][1]["kind"] = "trunk"
    data["nodes"][2]["radius"] = 0.0
    data["fruits"][0]["node"] = "stem4"
    with pytest.raises(SkeletonValidationError) as info:
        skeleton_from_dict(data)
    message = str(info.value)
    assert "unknown kind 'trunk'" in message
    assert "radius must be positive" in message
    assert "must hang from a pedicel" in message


@pytest.mark.parametrize(
    "change, expected",
    [
        (lambda d: d["nodes"][1].update(parent=None), "exactly one node must have parent null"),
        (lambda d: d["nodes"][4].update(parent="ghost"), "missing parent 'ghost'"),
        (lambda d: d["nodes"][13].update(parent="stem2"), "cannot hang from 'stem'"),
        (lambda d: d["nodes"][0].update(parent="stem4"), "parent cycle"),
        (lambda d: d["nodes"][4].update(kind="stem"), "the stem must be one chain"),
        (lambda d: d["nodes"].append(dict(d["nodes"][3])), "used more than once"),
        (lambda d: d["nodes"][3].update(end=d["nodes"][3]["start"]), "zero length"),
        (lambda d: d["leaflets"][0].update(node="stem1"), "petiole or leaf_rachis"),
        (lambda d: d["leaflets"][0].update(side="middle"), "side must be one of"),
        (lambda d: d["fruits"][1].update(node="pedicel1"), "carries two fruits"),
        (lambda d: d["fruits"][0].update(ripeness=1.5), "ripeness must be between 0 and 1"),
    ],
)
def test_structural_errors(change, expected):
    data = _data()
    change(data)
    assert any(expected in error for error in _errors(data)), _errors(data)


def test_frames_are_checked_against_the_segment():
    data = _data()
    data["nodes"][0]["frame"] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    assert _errors(data) == []
    data["nodes"][0]["frame"] = [[0, 1, 0], [1, 0, 0], [0, 0, 1]]
    assert any("left-handed" in error for error in _errors(data))
    data["nodes"][0]["frame"] = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
    assert any("does not follow" in error for error in _errors(data))


def test_gap_between_child_and_parent_is_a_warning():
    data = _data()
    data["nodes"][4]["start"] = [0.0, 0.0, 0.21]
    skeleton = skeleton_from_dict(data)
    assert any("10.0 mm away" in warning for warning in skeleton_warnings(skeleton))


def test_derive_frame_follows_direction_and_reference():
    frame = derive_frame((0.0, 1.0, 1.0), (1.0, 0.0, 0.0))
    x, y, z = frame_columns(frame)
    assert frame_error(frame, (0.0, 1.0, 1.0)) is None
    assert x == pytest.approx((1.0, 0.0, 0.0))
    assert z == pytest.approx((0.0, math.sqrt(0.5), math.sqrt(0.5)))
    # A reference parallel to the direction falls back to world axes.
    assert frame_error(derive_frame((1.0, 0.0, 0.0), (1.0, 0.0, 0.0)), (1.0, 0.0, 0.0)) is None


def test_closest_fraction_clamps_to_the_segment():
    assert closest_fraction((0.5, 1.0, 0.0), (0, 0, 0), (1, 0, 0)) == pytest.approx((1.0, 0.5))
    assert closest_fraction((2.0, 0.0, 0.0), (0, 0, 0), (1, 0, 0)) == pytest.approx((1.0, 1.0))
