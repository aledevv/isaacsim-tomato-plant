from __future__ import annotations

import json

import pytest

from exporterV2.joint_budget_merges import (
    FixedLinkMergeTechnique,
    LateralLinkMergeTechnique,
    LeafBranchMergeTechnique,
    StemLinkMergeTechnique,
    TrussRigidRachisTechnique,
    count_bodies,
    count_d6,
    validate_merges,
)
from exporterV2.joint_budget import optimize_branches
from exporterV2.tests.test_joint_budget import _branch, _result


def _ids(branches):
    return {b["id"]: b for b in branches}


def test_leaf_merge_absorbs_rachis_into_petiole():
    branches = list(_result().branches)
    technique = LeafBranchMergeTechnique()
    assert technique.estimate_reduction(branches) == 2
    modified, report = technique.apply(branches)
    rachis = _ids(modified)["Leaf_r0_o0_g1_rachis"]
    assert rachis["merged_links"] == [1, 2]
    assert rachis["n_links"] == 2
    assert report.joints_saved == 2
    assert report.details["bodies_after"] == report.details["bodies_before"] - 2
    assert validate_merges(branches, modified).valid
    assert not technique.can_apply(modified)


def test_lateral_merge_is_balanced_and_keeps_minimum():
    lateral = _branch("Branch_s1_o0_g9", "trunk", "lateral_branch", n_links=4)
    technique = LateralLinkMergeTechnique({"min_segments": 1})
    branches = [lateral]
    order = []
    while technique.can_apply(branches):
        branches, report = technique.apply(branches)
        order.append(report.details["link"])
    # 4 links -> groups [1][2][3][4] -> [1][2][3,4] -> [1,2][3,4] -> [1,2,3,4]
    assert order == [4, 2, 3]
    assert count_d6(branches) == 1 and count_bodies(branches) == 1


def test_stem_root_link_is_never_merged():
    stem = _branch("trunk", None, "stem", n_links=10, joint_type="fixed")
    technique = StemLinkMergeTechnique({"target_segments": 3})
    branches = [stem]
    while technique.can_apply(branches):
        branches, _ = technique.apply(branches)
    assert count_bodies(branches) == 3
    assert 1 not in branches[0]["merged_links"]
    assert validate_merges([stem], branches).valid


def test_truss_static_locks_pedicels_then_makes_rachis_rigid():
    branches = list(_result().branches)
    technique = TrussRigidRachisTechnique()
    stages = []
    while technique.can_apply(branches):
        branches, report = technique.apply(branches)
        stages.append(report.details["stage"])
    assert stages == ["pedicels_fixed", "pedicels_fixed", "rigid_rachis", "rigid_rachis"]
    by_id = _ids(branches)
    assert by_id["Truss_r1_o0_g2_rachis"]["merged_links"] == [2, 3]
    assert by_id["Truss_r1_o0_g2_pedicel_01"]["attachment_joint_type"] == "fixed"
    assert count_d6([by_id["Truss_r1_o0_g2_rachis"]]) == 1


def test_fixed_link_merge_takes_locked_petiolules_first_and_saves_no_d6():
    branches = [
        dict(b, joint_type="fixed", attachment_joint_type="fixed")
        if b["kind"] in ("petiolule", "pedicel") else b
        for b in _result().branches
    ]
    technique = FixedLinkMergeTechnique()
    d6 = count_d6(branches)
    modified, report = technique.apply(branches)
    assert report.details["branch_id"].startswith("LeafVisual_g1_")
    assert count_d6(modified) == d6
    assert count_bodies(modified) == count_bodies(branches) - 1
    # A fixed branch with a D6 attachment keeps link 1 as a body.
    hinged = _branch("X_rachis", "trunk", "leaf_rachis", n_links=3,
                     joint_type="fixed", attachment_joint_type="d6")
    assert technique._mergeable(hinged) == [2, 3]
    assert technique._mergeable(dict(_result().branches[0])) == []  # root


def test_validation_rejects_geometry_changes_and_root_merges():
    branches = list(_result().branches)
    changed = [dict(b, n_links=b["n_links"] + 1) if b["id"] == "trunk" else b for b in branches]
    assert not validate_merges(branches, changed).valid
    rooted = [dict(b, merged_links=[1]) if b["id"] == "trunk" else b for b in branches]
    assert not validate_merges(branches, rooted).valid


def test_full_optimizer_reaches_structural_lower_bound():
    optimized, report = optimize_branches(_result(), 3, techniques="full")
    # One D6 per leaf petiole and per truss rachis.
    assert report["lower_bound"] == 3
    assert report["final_d6"] == 3
    assert report["merged_links"]["Leaf_r0_o0_g1_rachis"] == [1, 2]
    assert count_d6(optimized.branches) == 3
