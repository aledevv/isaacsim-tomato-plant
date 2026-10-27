from __future__ import annotations

import pytest
import yaml

from exporterV2.config import (
    DEFAULT_PHYSICS,
    DEFAULT_PHYSICS_PATH,
    DEFAULT_VISUALS,
    DEFAULT_VISUALS_PATH,
    ConfigError,
    load_physics_config,
    load_visuals_config,
    physics_config_from_dict,
    visuals_config_from_dict,
)
from exporterV2.exporter import resolve_global_scale
from plant_skeleton import load_skeleton
from exporterV2.cli import DEFAULT_INPUT_PATH


def _keys(values, prefix=""):
    """Every nested key path of a settings dictionary."""
    keys = set()
    for key, value in values.items():
        keys.add(prefix + key)
        if isinstance(value, dict):
            keys |= _keys(value, prefix + key + ".")
    return keys


def test_config_files_only_use_known_keys():
    # Users edit values and may delete lines (a missing key means its default),
    # so only settings the code does not know fail. The raw files are read:
    # loading would fill missing keys from the defaults.
    assert _keys(yaml.safe_load(DEFAULT_PHYSICS_PATH.read_text())) <= _keys(DEFAULT_PHYSICS)
    assert _keys(yaml.safe_load(DEFAULT_VISUALS_PATH.read_text())) <= _keys(DEFAULT_VISUALS)


def test_missing_file_means_defaults(tmp_path):
    config = load_physics_config(tmp_path / "absent.yaml")
    assert config.values == DEFAULT_PHYSICS
    assert config.source == "defaults"


def test_partial_override_keeps_other_defaults(tmp_path):
    path = tmp_path / "physics.yaml"
    path.write_text("segments:\n  pedicel:\n    young_modulus: 1e9\nfruits:\n  mode: none\n")
    config = load_physics_config(path)
    assert config.segment("pedicel")["young_modulus"] == 1e9
    assert config.segment("pedicel")["bend_limit_deg"] == 25.0
    assert config.fruits["mode"] == "none"
    assert config.simulation["physics_hz"] == 480


@pytest.mark.parametrize(
    "data, message",
    [
        ({"segmnts": {}}, "unknown setting 'segmnts'"),
        ({"segments": {"stem": {"stiffness": 1}}}, "unknown setting 'segments.stem.stiffness'"),
        ({"fruits": {"mode": "heavy"}}, "fruits.mode must be one of"),
        ({"segments": {"branch": {"joint": "ball"}}}, "segments.branch.joint must be one of"),
        ({"segments": {"branch": {"density": "light"}}}, "must be a number"),
        ({"segments": {"branch": {"damping_ratio": 0}}}, "must be positive"),
        ({"global_scale": -1}, "global_scale"),
        ({"joints": {"joint_target": 300}}, "joint_target cannot exceed"),
        ({"collisions": {"branches": "yes"}}, "collisions.branches must be true or false"),
    ],
)
def test_bad_physics_settings_are_rejected(data, message):
    with pytest.raises(ConfigError, match=message):
        physics_config_from_dict(data)


def test_bad_visual_settings_are_rejected():
    with pytest.raises(ConfigError, match="leaves.generator must be one of"):
        visuals_config_from_dict({"leaves": {"generator": "photoreal"}})
    with pytest.raises(ConfigError, match="seed"):
        visuals_config_from_dict({"leaves": {"seed": -1}})
    with pytest.raises(ConfigError, match="one_per_leaflet"):
        visuals_config_from_dict({"leaves": {"one_per_leaflet": "always"}})
    with pytest.raises(ConfigError, match="droop_deg"):
        visuals_config_from_dict({"leaves": {"droop_deg": "low"}})
    with pytest.raises(ConfigError, match="trusses.pedicel_thickness"):
        visuals_config_from_dict({"trusses": {"pedicel_thickness": 1.5}})
    with pytest.raises(ConfigError, match="trusses.pedicel_length_scale"):
        visuals_config_from_dict({"trusses": {"pedicel_length_scale": 10}})
    with pytest.raises(ConfigError, match="stems.color"):
        visuals_config_from_dict({"stems": {"color": [2.0, 0.0, 0.0]}})
    with pytest.raises(ConfigError, match="stems.color"):
        visuals_config_from_dict({"stems": {"color": "green"}})


def test_global_scale_auto_keeps_thin_segments_thick_enough():
    skeleton = load_skeleton(DEFAULT_INPUT_PATH)
    thinnest = min(node.radius for node in skeleton.nodes)
    scale = resolve_global_scale(physics_config_from_dict({"global_scale": "auto"}), skeleton)
    assert scale * thinnest == pytest.approx(0.002) or scale in (1.0, 10.0)
    assert resolve_global_scale(physics_config_from_dict(), skeleton) == 2.0
