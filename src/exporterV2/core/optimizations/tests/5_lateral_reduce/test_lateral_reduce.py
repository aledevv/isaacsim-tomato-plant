"""
test_lateral_reduce.py - Unit tests for Lateral Branch Reduction technique

Tests:
1. Identification of lateral branches and lateral leaves
2. Reduction priority (smallest → lowest → alphabetical)
3. n_links reduction with height recalculation
4. Child attachment remapping
5. Minimum segments constraint
6. Validation of topology preservation
7. Multiple branches reduction
8. Edge cases (no reducible branches, already at minimum)
"""

import pytest
import sys
import os

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from techniques.lateral_reduce import LateralBranchReductionTechnique
from techniques.base import ValidationResult


def test_identify_lateral_branches():
    """Test identification of lateral branches."""
    technique = LateralBranchReductionTechnique()
    
    # Lateral branch
    branch1 = {"id": "Branch_r1_o0", "n_links": 3}
    assert technique._is_lateral_branch(branch1) is True
    
    # Lateral leaf
    leaf1 = {"id": "LateralLeaf_r2_o1", "n_links": 2}
    leaf2 = {"id": "LatLeaf_r2_o1", "n_links": 2}
    assert technique._is_lateral_leaf(leaf1) is True
    assert technique._is_lateral_leaf(leaf2) is True
    
    # Not lateral
    trunk = {"id": "trunk", "n_links": 5}
    assert technique._is_lateral_branch(trunk) is False
    assert technique._is_lateral_leaf(trunk) is False
    
    petiole = {"id": "Petiole_r1_o0", "n_links": 2}
    assert technique._is_lateral_branch(petiole) is False


def test_can_reduce():
    """Test can_reduce logic."""
    technique = LateralBranchReductionTechnique(min_segments=1)
    
    # Can reduce (n_links=3 > min=1)
    branch1 = {"id": "Branch_r1_o0", "n_links": 3, "radius": 0.03}
    assert technique._can_reduce(branch1) is True
    
    # Cannot reduce (at minimum)
    branch2 = {"id": "Branch_r1_o1", "n_links": 1, "radius": 0.03}
    assert technique._can_reduce(branch2) is False
    
    # Cannot reduce (not lateral)
    trunk = {"id": "trunk", "n_links": 5}
    assert technique._can_reduce(trunk) is False


def test_reduction_priority():
    """Test priority ordering (smallest → lowest → alphabetical)."""
    technique = LateralBranchReductionTechnique()
    
    # Different radii
    branch_small = {"id": "Branch_r1_o0", "radius": 0.02, "attach_link": 2}
    branch_large = {"id": "Branch_r1_o1", "radius": 0.05, "attach_link": 2}
    
    prio_small = technique._get_reduction_priority(branch_small)
    prio_large = technique._get_reduction_priority(branch_large)
    assert prio_small < prio_large  # Small radius first
    
    # Same radius, different attach_link
    branch_low = {"id": "Branch_r1_o0", "radius": 0.03, "attach_link": 2}
    branch_high = {"id": "Branch_r1_o1", "radius": 0.03, "attach_link": 5}
    
    prio_low = technique._get_reduction_priority(branch_low)
    prio_high = technique._get_reduction_priority(branch_high)
    assert prio_low < prio_high  # Lower attach first
    
    # Same radius and attach, alphabetical
    branch_a = {"id": "Branch_r1_o0", "radius": 0.03, "attach_link": 3}
    branch_b = {"id": "Branch_r1_o1", "radius": 0.03, "attach_link": 3}
    
    prio_a = technique._get_reduction_priority(branch_a)
    prio_b = technique._get_reduction_priority(branch_b)
    assert prio_a < prio_b  # Alphabetical


def test_can_apply():
    """Test can_apply method."""
    technique = LateralBranchReductionTechnique(min_segments=1)
    
    # No reducible branches
    branches1 = [
        {"id": "trunk", "n_links": 5},
        {"id": "Branch_r1_o0", "n_links": 1},  # At minimum
    ]
    assert technique.can_apply(branches1) is False
    
    # Has reducible branches
    branches2 = [
        {"id": "trunk", "n_links": 5},
        {"id": "Branch_r1_o0", "n_links": 3},  # Can reduce
    ]
    assert technique.can_apply(branches2) is True


def test_estimate_reduction():
    """Test reduction estimation."""
    technique = LateralBranchReductionTechnique(min_segments=1)
    
    branches = [
        {"id": "trunk", "n_links": 5},
        {"id": "Branch_r1_o0", "n_links": 3, "radius": 0.03},  # Can reduce 2
        {"id": "Branch_r1_o1", "n_links": 2, "radius": 0.03},  # Can reduce 1
        {"id": "Branch_r1_o2", "n_links": 1, "radius": 0.03},  # At minimum
        {"id": "LateralLeaf_r1_o0", "n_links": 2},  # Can reduce 1
    ]
    
    reduction = technique.estimate_reduction(branches)
    assert reduction == 4  # 2 + 1 + 0 + 1


def test_apply_single_branch():
    """Test applying reduction to single branch."""
    technique = LateralBranchReductionTechnique(min_segments=1)
    
    branches = [
        {"id": "trunk", "parent": None, "n_links": 5, "height": 0.20, "radius": 0.10},
        {"id": "Branch_r1_o0", "parent": "trunk", "attach_link": 3, 
         "n_links": 3, "height": 0.15, "radius": 0.03},
    ]
    
    modified, report = technique.apply(branches)
    
    # Check report
    assert report.technique_name == "lateral_reduce"
    assert report.joints_saved == 1
    assert report.details["branches_reduced"] == 1
    assert report.details["links_removed"] == 1
    
    # Check branch modified
    mod_dict = {b["id"]: b for b in modified}
    branch = mod_dict["Branch_r1_o0"]
    
    assert branch["n_links"] == 2  # 3 → 2
    
    # Check height recalculation (preserve total length)
    original_length = 3 * 0.15
    new_length = 2 * branch["height"]
    assert abs(original_length - new_length) < 0.001  # Sub-mm precision


def test_apply_with_child_remapping():
    """Test applying reduction with child attachment remapping."""
    technique = LateralBranchReductionTechnique(min_segments=1)
    
    branches = [
        {"id": "trunk", "parent": None, "n_links": 5, "height": 0.20, "radius": 0.10},
        {"id": "Branch_r1_o0", "parent": "trunk", "attach_link": 3,
         "n_links": 4, "height": 0.15, "radius": 0.03},
        {"id": "Petiole_r1_o0", "parent": "Branch_r1_o0", "attach_link": 3,
         "n_links": 2, "height": 0.20, "radius": 0.03},
    ]
    
    modified, report = technique.apply(branches)
    
    # Check child remapped
    mod_dict = {b["id"]: b for b in modified}
    branch = mod_dict["Branch_r1_o0"]
    child = mod_dict["Petiole_r1_o0"]
    
    assert branch["n_links"] == 3  # 4 → 3
    
    # Original: link 3 of 4 -> H = 0.75
    # New: 3 links. V = 0.75 * 3 = 2.25
    # k_new = floor(2.25) + 1 = 3
    # p_new = 0.25
    assert child["attach_link"] == 3
    assert child["attach_frac"] == 0.25
    assert report.details["children_remapped"] == 1


def test_apply_multiple_branches():
    """Test applying reduction to multiple branches with priority."""
    technique = LateralBranchReductionTechnique(min_segments=1)
    
    branches = [
        {"id": "trunk", "parent": None, "n_links": 5, "height": 0.20, "radius": 0.10},
        # Small radius (reduce first)
        {"id": "Branch_r1_o0", "parent": "trunk", "attach_link": 2,
         "n_links": 3, "height": 0.15, "radius": 0.02},
        # Large radius (reduce second)
        {"id": "Branch_r1_o1", "parent": "trunk", "attach_link": 4,
         "n_links": 3, "height": 0.15, "radius": 0.05},
        # Lateral leaf
        {"id": "LateralLeaf_r1_o0", "parent": "Branch_r1_o0", "attach_link": 1,
         "n_links": 2, "height": 0.10, "radius": 0.01},
    ]
    
    modified, report = technique.apply(branches)
    
    # Check all reducible branches were reduced
    assert report.details["branches_reduced"] == 3  # 2 branches + 1 leaf
    assert report.joints_saved == 3
    
    # Check each branch
    mod_dict = {b["id"]: b for b in modified}
    
    assert mod_dict["Branch_r1_o0"]["n_links"] == 2
    assert mod_dict["Branch_r1_o1"]["n_links"] == 2
    assert mod_dict["LateralLeaf_r1_o0"]["n_links"] == 1


def test_apply_respects_minimum():
    """Test that reduction respects minimum segments."""
    technique = LateralBranchReductionTechnique(min_segments=2)
    
    branches = [
        {"id": "trunk", "parent": None, "n_links": 5, "height": 0.20, "radius": 0.10},
        {"id": "Branch_r1_o0", "parent": "trunk", "attach_link": 3,
         "n_links": 3, "height": 0.15, "radius": 0.03},  # Can reduce to 2
        {"id": "Branch_r1_o1", "parent": "trunk", "attach_link": 4,
         "n_links": 2, "height": 0.15, "radius": 0.03},  # At minimum
    ]
    
    modified, report = technique.apply(branches)
    
    # Only first branch reduced
    assert report.details["branches_reduced"] == 1
    
    mod_dict = {b["id"]: b for b in modified}
    assert mod_dict["Branch_r1_o0"]["n_links"] == 2
    assert mod_dict["Branch_r1_o1"]["n_links"] == 2  # Unchanged


def test_validate_success():
    """Test validation of successful reduction."""
    technique = LateralBranchReductionTechnique(min_segments=1)
    
    original = [
        {"id": "trunk", "parent": None, "n_links": 5, "height": 0.20, "radius": 0.10},
        {"id": "Branch_r1_o0", "parent": "trunk", "attach_link": 3,
         "n_links": 3, "height": 0.15, "radius": 0.03},
    ]
    
    modified, _ = technique.apply(original)
    result = technique.validate(original, modified)
    
    assert result.valid is True
    assert len(result.errors) == 0


def test_validate_detects_errors():
    """Test validation detects invalid modifications."""
    technique = LateralBranchReductionTechnique(min_segments=1)
    
    original = [
        {"id": "trunk", "parent": None, "n_links": 5, "height": 0.20, "radius": 0.10},
        {"id": "Branch_r1_o0", "parent": "trunk", "attach_link": 3,
         "n_links": 3, "height": 0.15, "radius": 0.03},
    ]
    
    # Invalid: trunk n_links changed
    modified_invalid = [
        {"id": "trunk", "parent": None, "n_links": 3, "height": 0.20, "radius": 0.10},
        {"id": "Branch_r1_o0", "parent": "trunk", "attach_link": 3,
         "n_links": 2, "height": 0.225, "radius": 0.03},
    ]
    
    result = technique.validate(original, modified_invalid)
    assert result.valid is False
    assert len(result.errors) > 0


def test_no_reducible_branches():
    """Test behavior when no branches can be reduced."""
    technique = LateralBranchReductionTechnique(min_segments=1)
    
    branches = [
        {"id": "trunk", "parent": None, "n_links": 5, "height": 0.20, "radius": 0.10},
        {"id": "Branch_r1_o0", "parent": "trunk", "attach_link": 3,
         "n_links": 1, "height": 0.15, "radius": 0.03},  # At minimum
    ]
    
    modified, report = technique.apply(branches)
    
    # No changes
    assert report.joints_saved == 0
    assert report.details["branches_reduced"] == 0
    assert modified == branches


def test_skip_fixed_branches():
    """Test that Fixed branches are excluded from lateral reduction.

    When thin_link_lock or petiole_lock runs before lateral_reduce (as per
    priority order), a lateral branch may already have joint_type="fixed".
    lateral_reduce must NOT reduce such branches — they carry zero D6 joints,
    so reducing their n_links saves nothing and the report would be wrong.
    """
    technique = LateralBranchReductionTechnique(min_segments=1)

    branches = [
        {"id": "trunk", "parent": None, "n_links": 5, "height": 0.20, "radius": 0.10},
        # Normal lateral branch — should be reduced
        {"id": "Branch_r1_o0", "parent": "trunk", "attach_link": 2,
         "n_links": 3, "height": 0.15, "radius": 0.03},
        # Thin lateral branch already converted to Fixed — must NOT be reduced
        {"id": "Branch_r1_o1", "parent": "trunk", "attach_link": 4,
         "n_links": 3, "height": 0.10, "radius": 0.001, "joint_type": "fixed"},
    ]

    # Fixed branch should not count as reducible
    assert technique._can_reduce(branches[2]) is False, \
        "Fixed branch should not be reducible"

    modified, report = technique.apply(branches)

    mod_dict = {b["id"]: b for b in modified}

    # Only the normal branch is reduced
    assert mod_dict["Branch_r1_o0"]["n_links"] == 2
    # Fixed branch must be completely unchanged
    assert mod_dict["Branch_r1_o1"]["n_links"] == 3
    assert mod_dict["Branch_r1_o1"]["joint_type"] == "fixed"

    # Report must only count D6 joints actually saved
    assert report.joints_saved == 1, "Only one D6 joint saved (Fixed branch excluded)"
    assert report.details["branches_reduced"] == 1


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
