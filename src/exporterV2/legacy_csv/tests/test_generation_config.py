"""Configuration, hierarchy, and resolution tests for exporterV2."""

import math
import json
import sys
from pathlib import Path


SRC_DIR = Path(__file__).resolve().parents[3]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from exporterV2.legacy_csv import parser
from exporterV2.core.optimizations.techniques.lateral_reduce import (
    LateralBranchReductionTechnique,
)
from exporterV2.core.tree_config import (
    OrganGenerationConfig,
    PhysicsRuntimeConfig,
    limit_branch_resolution,
)


def test_resolution_cap_preserves_lengths_and_nested_attachment_positions():
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "n_links": 20,
            "height": 0.05,
            "radius": 0.01,
        },
        {
            "id": "Branch_r1_o0",
            "parent": "trunk",
            "attach_link": 5,
            "attach_frac": 0.5,
            "n_links": 12,
            "height": 0.02,
            "radius": 0.005,
        },
        {
            "id": "child",
            "parent": "Branch_r1_o0",
            "attach_link": 7,
            "attach_frac": 0.25,
            "n_links": 2,
            "height": 0.01,
            "radius": 0.002,
        },
    ]
    original_lengths = {
        branch["id"]: branch["n_links"] * branch["height"]
        for branch in branches
    }
    original_positions = {
        "Branch_r1_o0": (5 - 1 + 0.5) / 20,
        "child": (7 - 1 + 0.25) / 12,
    }

    limited, changes = limit_branch_resolution(branches, max_links=10, verbose=False)
    by_id = {branch["id"]: branch for branch in limited}

    assert branches[0]["n_links"] == 20
    assert branches[1]["n_links"] == 12
    assert {change["branch_id"] for change in changes} == {"trunk", "Branch_r1_o0"}
    assert all(branch["n_links"] <= 10 for branch in limited)
    for branch in limited:
        assert math.isclose(
            branch["n_links"] * branch["height"],
            original_lengths[branch["id"]],
            rel_tol=0.0,
            abs_tol=1e-12,
        )

    lateral = by_id["Branch_r1_o0"]
    child = by_id["child"]
    assert math.isclose(
        (lateral["attach_link"] - 1 + lateral["attach_frac"]) / by_id["trunk"]["n_links"],
        original_positions["Branch_r1_o0"],
        abs_tol=1e-12,
    )
    assert math.isclose(
        (child["attach_link"] - 1 + child["attach_frac"]) / lateral["n_links"],
        original_positions["child"],
        abs_tol=1e-12,
    )


def test_rigid_trunk_flag_controls_generated_joint_type(monkeypatch):
    internodes = [
        {"rank": 0, "organ_index": 0, "width_m": 0.02, "length": 0.10},
        {"rank": 1, "organ_index": 0, "width_m": 0.02, "length": 0.12},
    ]

    monkeypatch.setattr(PhysicsRuntimeConfig, "RIGID_TRUNK", True)
    rigid = parser.internodes_to_branch_config(internodes)
    assert rigid["joint_type"] == "fixed"

    monkeypatch.setattr(PhysicsRuntimeConfig, "RIGID_TRUNK", False)
    flexible = parser.internodes_to_branch_config(internodes)
    assert flexible["joint_type"] == "d6"


def test_optimizer_can_reduce_below_configured_cap():
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "n_links": 4,
            "height": 0.1,
            "radius": 0.01,
        },
        {
            "id": "Branch_r1_o0",
            "parent": "trunk",
            "attach_link": 2,
            "n_links": 14,
            "height": 0.02,
            "radius": 0.005,
        },
    ]
    limited, _ = limit_branch_resolution(branches, max_links=10, verbose=False)
    optimized, _ = LateralBranchReductionTechnique(min_segments=5).apply(limited)
    lateral = next(branch for branch in optimized if branch["id"] == "Branch_r1_o0")

    assert lateral["n_links"] == 9
    assert lateral["n_links"] < 10


def test_global_switches_cascade_and_respect_profile(monkeypatch):
    profile = {
        "lateral_branches": {"enabled": True},
        "trunk_leaves": {"enabled": True},
        "lateral_leaves": {"enabled": True},
    }
    monkeypatch.setattr(OrganGenerationConfig, "CREATE_LATERAL_BRANCHES", False)
    settings = parser._effective_generation_settings(profile)
    assert settings["lateral_branches"] is False
    assert settings["lateral_leaves"] is False
    assert settings["trunk_leaves"] is True

    monkeypatch.setattr(OrganGenerationConfig, "CREATE_LEAF_RACHIS", False)
    settings = parser._effective_generation_settings(profile)
    assert settings["petioles"] is True
    assert settings["leaf_rachis"] is False
    assert settings["petiolules"] is False

    monkeypatch.setattr(OrganGenerationConfig, "CREATE_PEDICELS", False)
    settings = parser._effective_generation_settings(profile)
    assert settings["truss_rachis"] is True
    assert settings["pedicels"] is False
    assert settings["tomatoes"] is False

    profile["trunk_leaves"]["enabled"] = False
    monkeypatch.setattr(OrganGenerationConfig, "CREATE_LEAF_RACHIS", True)
    settings = parser._effective_generation_settings(profile)
    assert settings["trunk_leaves"] is False

    monkeypatch.setattr(OrganGenerationConfig, "CREATE_PETIOLES", False)
    profile["trunk_leaves"]["enabled"] = True
    settings = parser._effective_generation_settings(profile)
    assert settings["trunk_leaves"] is False
    assert settings["petioles"] is False
    assert settings["leaf_rachis"] is False
    assert settings["petiolules"] is False


def test_lateral_switch_keeps_trunk_leaves(monkeypatch):
    monkeypatch.setattr(OrganGenerationConfig, "CREATE_LATERAL_BRANCHES", False)
    monkeypatch.setattr(OrganGenerationConfig, "CREATE_TRUSSES", False)
    branches, _, _ = parser.parse_csv_to_branches(
        day=80,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )
    branch_ids = {branch["id"] for branch in branches}

    assert any(branch_id.startswith("Leaf_") for branch_id in branch_ids)
    assert not any(branch_id.startswith("Branch_") for branch_id in branch_ids)
    assert not any(branch_id.startswith("LatLeaf_") for branch_id in branch_ids)


def test_leaf_master_switch_removes_complete_leaf_hierarchy(monkeypatch):
    monkeypatch.setattr(OrganGenerationConfig, "CREATE_LEAF_BRANCHES", False)
    monkeypatch.setattr(OrganGenerationConfig, "CREATE_TRUSSES", False)
    branches, _, _ = parser.parse_csv_to_branches(
        day=80,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )
    branch_ids = {branch["id"] for branch in branches}

    assert any(branch_id.startswith("Branch_") for branch_id in branch_ids)
    assert not any(branch_id.startswith(("Leaf_", "LatLeaf_")) for branch_id in branch_ids)


def test_leaf_rachis_switch_keeps_only_petiole_level(monkeypatch):
    monkeypatch.setattr(OrganGenerationConfig, "CREATE_LEAF_RACHIS", False)
    monkeypatch.setattr(OrganGenerationConfig, "CREATE_TRUSSES", False)
    branches, _, _ = parser.parse_csv_to_branches(
        day=80,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )
    leaf_ids = [
        branch["id"]
        for branch in branches
        if branch["id"].startswith(("Leaf_", "LatLeaf_"))
    ]

    assert leaf_ids
    assert all(branch_id.endswith("_petiole") for branch_id in leaf_ids)


def test_truss_master_switch_removes_branches_and_terminal_bodies(monkeypatch):
    monkeypatch.setattr(OrganGenerationConfig, "CREATE_TRUSSES", False)
    branches, terminal_bodies, _ = parser.parse_csv_to_branches(
        day=80,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )

    assert not any(branch["id"].startswith("Truss_") for branch in branches)
    assert not any(body.get("kind") == "tomato" for body in terminal_bodies)


def test_day80_truss_children_follow_debug_switches(monkeypatch):
    monkeypatch.setattr(OrganGenerationConfig, "CREATE_PEDICELS", False)
    branches, terminal_bodies, _ = parser.parse_csv_to_branches(
        day=80,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )
    truss_ids = [branch["id"] for branch in branches if branch["id"].startswith("Truss_")]

    assert truss_ids
    assert all(branch_id.endswith("_rachis") for branch_id in truss_ids)
    assert not any(body.get("kind") == "tomato" for body in terminal_bodies)


def test_day80_tomatoes_can_be_hidden_without_removing_pedicels(monkeypatch):
    monkeypatch.setattr(OrganGenerationConfig, "CREATE_TOMATOES", False)
    branches, terminal_bodies, _ = parser.parse_csv_to_branches(
        day=80,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )

    assert any("_pedicel_" in branch["id"] for branch in branches)
    assert not any(body.get("kind") == "tomato" for body in terminal_bodies)


def test_parse_pipeline_reads_csv_once(monkeypatch):
    real_read_csv = parser.pd.read_csv
    calls = []

    def recording_read_csv(*args, **kwargs):
        calls.append(args[0])
        return real_read_csv(*args, **kwargs)

    monkeypatch.setattr(parser.pd, "read_csv", recording_read_csv)
    parser.parse_csv_to_branches(
        day=30,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )

    assert len(calls) == 1


def test_standalone_loaders_read_their_own_dataframe(monkeypatch):
    project_root = Path(__file__).resolve().parents[4]
    csv_path = (
        project_root
        / "data/simulation_output/dynamic_output/graphs/graph_day_30.csv"
    )
    real_read_csv = parser.pd.read_csv
    calls = []

    def recording_read_csv(*args, **kwargs):
        calls.append(args[0])
        return real_read_csv(*args, **kwargs)

    monkeypatch.setattr(parser.pd, "read_csv", recording_read_csv)
    parser.load_trunk_internodes(str(csv_path), 30)
    parser.load_trusses(str(csv_path), 30)

    assert len(calls) == 2


def test_json_metadata_records_effective_configuration(tmp_path):
    output_path = parser.save_branches_json(
        branches=[
            {
                "id": "trunk",
                "parent": None,
                "n_links": 1,
                "height": 0.1,
                "radius": 0.01,
            }
        ],
        day=1,
        output_dir=str(tmp_path),
        internodes=[{"rank": 0}],
        csv_filename="synthetic.csv",
        generation_settings={"trusses": False},
        resolution_changes=[{"branch_id": "trunk"}],
    )

    data = json.loads(Path(output_path).read_text())
    metadata = data["metadata"]
    assert metadata["physics_runtime"]["physics_hz"] == 480
    assert metadata["physics_runtime"]["rigid_trunk"] is True
    assert metadata["physics_runtime"]["solver_position_iterations"] == 32
    assert metadata["physics_runtime"]["solver_velocity_iterations"] == 4
    assert metadata["physics_runtime"]["terminal_body_solver_position_iterations"] == 32
    assert metadata["physics_runtime"]["terminal_body_solver_velocity_iterations"] == 1
    assert metadata["branch_resolution"]["max_links_per_branch"] == 10
    assert metadata["branch_resolution"]["capped_branch_count"] == 1
    assert metadata["organ_generation"] == {"trusses": False}
