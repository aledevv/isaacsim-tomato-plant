"""
test_parser_truss_integration.py - CSV parser integration tests for trunk trusses.

Run with:
    uv run python src/exporterV2/legacy_csv/tests/test_parser_truss_integration.py
"""

import sys
from pathlib import Path

src_dir = Path(__file__).resolve().parents[4]
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

from exporterV2.tests.legacy_csv import parse_csv_to_branches
from exporterV2.core.optimizations import BudgetOptimizer
from exporterV2.core.optimizations.techniques.base import count_d6_joints
from exporterV2.core.tree_config import MAX_N_JOINTS, TrussGeometryConfig


def test_day80_trunk_trusses():
    """Day 80 should generate trunk trusses and terminal tomato bodies."""
    branches, terminal_bodies, _ = parse_csv_to_branches(
        day=80,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )

    branch_ids = {branch["id"] for branch in branches}
    truss_branches = [branch for branch in branches if branch["id"].startswith("Truss_")]
    truss_rachises = [branch for branch in truss_branches if branch["id"].endswith("_rachis")]

    assert len(truss_rachises) == 5, f"Expected 5 trunk trusses, got {len(truss_rachises)}"
    tomatoes = [body for body in terminal_bodies if body.get("kind") == "tomato"]
    assert len(tomatoes) == 40, f"Expected 40 tomatoes, got {len(tomatoes)}"

    for branch in truss_branches:
        assert branch["physics_profile"] == "truss", f"{branch['id']} should use truss physics"
        assert branch["height"] > 0.0, f"{branch['id']} should have positive height"

    for rachis in truss_rachises:
        assert TrussGeometryConfig.MIN_TILT_DEG <= rachis["tilt"] <= TrussGeometryConfig.MAX_TILT_DEG
        assert abs(rachis["height"] - TrussGeometryConfig.RACHIS_SEGMENT_LENGTH) < 1e-9

    for body in tomatoes:
        assert body["parent_branch_id"] in branch_ids, f"{body['id']} has missing parent"
        assert body["radius"] > 0.0, f"{body['id']} should have positive radius"


def test_day50_zero_dimensions_are_filtered():
    """Day 50 should not emit zero-height branches or invalid terminal bodies."""
    branches, terminal_bodies, _ = parse_csv_to_branches(
        day=50,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )

    branch_ids = {branch["id"] for branch in branches}

    for branch in branches:
        assert branch["height"] > 0.0, f"{branch['id']} has zero height"
        assert branch["radius"] > 0.0, f"{branch['id']} has zero radius"
        assert branch["n_links"] > 0, f"{branch['id']} has zero links"

    for body in terminal_bodies:
        if body.get("shape") != "sphere":
            continue
        assert body["radius"] > 0.0, f"{body['id']} has zero radius"
        assert body["parent_branch_id"] in branch_ids, f"{body['id']} has missing parent"


def test_day80_optimizer_meets_d6_budget():
    """Day 80 must already meet, or be reducible to, the configured budget."""
    branches, terminal_bodies, _ = parse_csv_to_branches(
        day=80,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )

    original_joints = count_d6_joints(branches)
    assert original_joints > 0

    optimized, report = BudgetOptimizer(max_joints=MAX_N_JOINTS).optimize(branches)
    final_joints = count_d6_joints(optimized)
    optimized_ids = {branch["id"] for branch in optimized}

    assert report.success, report.error_message
    assert final_joints <= MAX_N_JOINTS, f"Expected <= {MAX_N_JOINTS}, got {final_joints}"

    for body in terminal_bodies:
        assert body["parent_branch_id"] in optimized_ids, f"{body['id']} lost its parent after optimization"


if __name__ == "__main__":
    try:
        test_day80_trunk_trusses()
        test_day50_zero_dimensions_are_filtered()
        test_day80_optimizer_meets_d6_budget()
        print("\nALL PARSER TRUSS INTEGRATION TESTS PASSED")
        sys.exit(0)
    except AssertionError as exc:
        print(f"\nTEST FAILED: {exc}")
        sys.exit(1)
