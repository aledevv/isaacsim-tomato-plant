from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from exporterV2.config import physics_config_from_dict, visuals_config_from_dict
from exporterV2.skeleton_branches import PEDICEL_TILT_DEG, SkeletonBranchesError, build_branch_plan
from plant_skeleton import skeleton_from_dict


PROJECT_ROOT = Path(__file__).resolve().parents[3]
SMALL_PLANT = PROJECT_ROOT / "input" / "examples" / "small_plant.json"


def _data():
    return json.loads(SMALL_PLANT.read_text())


def _plan(data=None, physics=None, visuals=None):
    skeleton = skeleton_from_dict(data or _data())
    return build_branch_plan(skeleton, physics_config_from_dict(physics), visuals_config_from_dict(visuals))


def _by_id(plan):
    return {branch["id"]: branch for branch in plan.branches}


def test_nodes_are_grouped_into_chains():
    branches = _by_id(_plan())
    assert list(branches) == [
        "trunk",
        "Branch_shoot1",
        "Leaf_leafA_petiole",
        "Leaf_leafA_rachis",
        "Leaf_leafB_petiole",
        "Leaf_leafB_rachis",
        "Truss_truss_rachis1_rachis",
        "Truss_truss_rachis1_pedicel_01",
        "Truss_truss_rachis1_pedicel_02",
        "Truss_truss_rachis1_pedicel_03",
    ]
    assert branches["trunk"]["n_links"] == 4
    assert branches["Branch_shoot1"]["n_links"] == 2
    assert branches["Truss_truss_rachis1_rachis"]["n_links"] == 2
    assert branches["Leaf_leafA_rachis"]["n_links"] == 2


def test_children_attach_to_the_end_of_their_parent_node():
    branches = _by_id(_plan())
    assert (branches["Branch_shoot1"]["parent"], branches["Branch_shoot1"]["attach_link"]) == ("trunk", 2)
    assert (branches["Truss_truss_rachis1_rachis"]["parent"], branches["Truss_truss_rachis1_rachis"]["attach_link"]) == ("Branch_shoot1", 1)
    assert (branches["Truss_truss_rachis1_pedicel_03"]["parent"], branches["Truss_truss_rachis1_pedicel_03"]["attach_link"]) == ("Truss_truss_rachis1_rachis", 2)
    assert (branches["Leaf_leafB_petiole"]["parent"], branches["Leaf_leafB_petiole"]["attach_link"]) == ("trunk", 4)


def test_petiole_and_rachis_share_one_continuous_mesh():
    branches = _by_id(_plan())
    assert branches["Leaf_leafA_petiole"]["visual_axis_id"] == "Leaf_leafA_axis"
    assert branches["Leaf_leafA_rachis"]["visual_axis_id"] == "Leaf_leafA_axis"
    assert branches["Truss_truss_rachis1_rachis"]["visual_axis_id"] == "Truss_truss_rachis1_rachis"


def test_explicit_axis_overrides_the_chain_rule():
    data = _data()
    data["nodes"][4]["axis"] = "shoot_a"
    data["nodes"][5]["axis"] = "shoot_b"
    branches = _by_id(_plan(data))
    assert branches["Branch_shoot1"]["n_links"] == 1
    assert branches["Branch_shoot2"]["parent"] == "Branch_shoot1"


def test_an_axis_must_be_one_unbranched_chain():
    data = _data()
    data["nodes"][4]["axis"] = data["nodes"][5]["axis"] = "shoot"
    data["nodes"].append(
        {"id": "shoot3", "parent": "shoot1", "kind": "branch", "start": [0.08, 0.0, 0.26], "end": [0.1, 0.05, 0.3], "radius": 0.002, "axis": "shoot"}
    )
    with pytest.raises(SkeletonBranchesError, match="axis 'shoot' branches at node 'shoot1'"):
        _plan(data)


def test_physics_comes_from_the_segment_settings():
    branches = _by_id(_plan(physics={"segments": {"branch": {"joint": "fixed", "young_modulus": 1e7}}}))
    assert branches["trunk"]["joint_type"] == "fixed"
    assert branches["Branch_shoot1"]["joint_type"] == "fixed"
    assert branches["Branch_shoot1"]["young_modulus"] == 1e7
    pedicel = branches["Truss_truss_rachis1_pedicel_01"]
    assert (pedicel["young_modulus"], pedicel["bend_limit_deg"], pedicel["drive_stiffness_scale"]) == (4e9, 25.0, 0.2)
    assert pedicel["physics_profile"] == "truss"
    assert branches["Leaf_leafA_petiole"]["damping_ratio"] == 0.3


def test_visual_switches():
    plan = _plan(visuals={"quality": "performance", "stems": {"junction_bulge": False, "terminal_forks": False}, "pedicels": {"gravity_curve": False}})
    branches = _by_id(plan)
    assert branches["trunk"]["visual_profile"]["radial_segments"] == 12
    assert branches["trunk"]["visual_profile"]["junction_bulge_amplitude"] == 0.0
    assert branches["Leaf_leafB_petiole"]["disable_terminal_fork"] is True
    assert "visual_style" not in branches["Truss_truss_rachis1_pedicel_01"]
    assert branches["Truss_truss_rachis1_rachis"]["visual_style"] == "historical_truss_rachis"


@pytest.mark.parametrize("mode, physical", [("none", None), ("visual", False), ("physical", True)])
def test_fruit_modes(mode, physical):
    plan = _plan(physics={"fruits": {"mode": mode}})
    if physical is None:
        assert plan.terminal_bodies == ()
        return
    assert len(plan.terminal_bodies) == 3
    body = plan.terminal_bodies[0]
    assert body["physical"] is physical
    assert body["detachment_enabled"] is physical
    assert body["parent_branch_id"] == "Truss_truss_rachis1_pedicel_01"
    assert body["maturation"] == 0.2


def _z(frame):
    return [row[2] for row in frame[:3]]


def test_v2_pose_tilts_pedicels_and_spreads_leaflets():
    plan = _plan()
    branches = _by_id(plan)
    rachis = [spec["rest_frame"] for spec in branches["Truss_truss_rachis1_rachis"]["link_specs"]]
    pedicels = [branches[f"Truss_truss_rachis1_pedicel_0{i}"]["link_specs"][0]["rest_frame"] for i in (1, 2, 3)]
    tilt = math.cos(math.radians(PEDICEL_TILT_DEG))
    assert sum(a * b for a, b in zip(_z(pedicels[0]), _z(rachis[0]))) == pytest.approx(tilt)
    assert sum(a * b for a, b in zip(_z(pedicels[1]), _z(rachis[1]))) == pytest.approx(tilt)
    # Odd count: the last pedicel continues straight along its rachis segment.
    assert _z(pedicels[2]) == pytest.approx(_z(rachis[1]))
    left, right = plan.rigid_leaf_visuals[0], plan.rigid_leaf_visuals[1]
    assert (left["leaflet_role"], right["leaflet_role"]) == ("left", "right")
    assert _z(left["rest_frame"])[0] == pytest.approx(-_z(right["rest_frame"])[0])


def test_canonical_pose_uses_skeleton_frames():
    plan = _plan(visuals={"pose": "canonical"})
    pedicel = _by_id(plan)["Truss_truss_rachis1_pedicel_01"]["link_specs"][0]
    assert pedicel["rest_frame"] == pedicel["source_rest_frame"]
    assert [row[2] for row in pedicel["rest_frame"][:3]] == pytest.approx([0.0, 0.0, -1.0])
