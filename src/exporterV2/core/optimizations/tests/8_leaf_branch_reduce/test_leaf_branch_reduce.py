"""
test_leaf_branch_reduce.py - Unit tests for Leaf Branch Reduction technique

Tests:
1. Identification of petiole and rachis
2. Finding petiole+rachis pairs
3. Merging: length preservation, radius averaging
4. Petiolule remapping
5. Validation
6. Edge cases
"""

import pytest
import sys
import os

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from techniques.leaf_branch_reduce import LeafBranchReductionTechnique
from techniques.base import ValidationResult


def test_identify_petiole_rachis():
    """Test identification of petiole and rachis branches."""
    technique = LeafBranchReductionTechnique()
    
    petiole = {"id": "Leaf_r1_o0_petiole"}
    rachis = {"id": "Leaf_r1_o0_rachis"}
    other = {"id": "Branch_r1_o0"}
    
    assert technique._is_petiole(petiole) is True
    assert technique._is_rachis(rachis) is True
    assert technique._is_petiole(rachis) is False
    assert technique._is_rachis(petiole) is False
    assert technique._is_petiole(other) is False
    assert technique._is_rachis(other) is False





def test_can_apply():
    """Test can_apply method."""
    technique = LeafBranchReductionTechnique()
    
    # No pairs
    branches1 = [
        {"id": "trunk", "parent": None},
    ]
    assert technique.can_apply(branches1) is False
    
    # Has pairs
    branches2 = [
        {"id": "trunk", "parent": None},
        {"id": "Leaf_r1_o0_petiole", "parent": "trunk"},
        {"id": "Leaf_r1_o0_rachis", "parent": "Leaf_r1_o0_petiole"},
    ]
    assert technique.can_apply(branches2) is True


def test_estimate_reduction():
    """Test reduction estimation."""
    technique = LeafBranchReductionTechnique()
    
    branches = [
        {"id": "trunk", "parent": None},
        {"id": "Leaf_r1_o0_petiole", "parent": "trunk", "n_links": 1},
        {"id": "Leaf_r1_o0_rachis", "parent": "Leaf_r1_o0_petiole", "n_links": 3},
        {"id": "Leaf_r2_o0_petiole", "parent": "trunk", "n_links": 1},
        {"id": "Leaf_r2_o0_rachis", "parent": "Leaf_r2_o0_petiole", "n_links": 2},
    ]
    
    reduction = technique.estimate_reduction(branches)
    assert reduction == 5  # 3 + 2 rachis links


def test_apply_single_pair():
    """Test merging single petiole+rachis pair."""
    technique = LeafBranchReductionTechnique()
    
    branches = [
        {"id": "trunk", "parent": None, "n_links": 5},
        {"id": "Leaf_r1_o0_petiole", "parent": "trunk", "attach_link": 2,
         "n_links": 1, "height": 0.10, "radius": 0.030,
         "tilt": 45.0, "rot": 90.0},
        {"id": "Leaf_r1_o0_rachis", "parent": "Leaf_r1_o0_petiole", "attach_link": 1,
         "n_links": 3, "height": 0.05, "radius": 0.020,
         "tilt": 0.0, "rot": 0.0},
    ]
    
    modified, report = technique.apply(branches)
    
    # Check report
    assert report.technique_name == "leaf_branch_reduce"
    assert report.joints_saved == 3  # Removed 3 rachis links
    assert report.details["pairs_merged"] == 1
    
    # Check rachis removed
    mod_dict = {b["id"]: b for b in modified}
    assert "Leaf_r1_o0_rachis" not in mod_dict
    
    # Check petiole merged
    merged = mod_dict["Leaf_r1_o0_merged"]
    assert merged["n_links"] == 1
    
    # Check total length preserved
    orig_length = 1 * 0.10 + 3 * 0.05  # 0.10 + 0.15 = 0.25
    merged_length = merged["n_links"] * merged["height"]
    assert abs(orig_length - merged_length) < 0.001


def test_apply_with_petiolules():
    """Test merging with petiolule remapping."""
    technique = LeafBranchReductionTechnique()
    
    branches = [
        {"id": "trunk", "parent": None},
        {"id": "Leaf_r1_o0_petiole", "parent": "trunk", "attach_link": 1,
         "n_links": 1, "height": 0.10, "radius": 0.030},
        {"id": "Leaf_r1_o0_rachis", "parent": "Leaf_r1_o0_petiole", "attach_link": 1,
         "n_links": 3, "height": 0.05, "radius": 0.020},
        {"id": "Petiolule_r1_o0_lf1", "parent": "Leaf_r1_o0_rachis", "attach_link": 1,
         "n_links": 1, "height": 0.03, "radius": 0.010},
        {"id": "Petiolule_r1_o0_lf2", "parent": "Leaf_r1_o0_rachis", "attach_link": 2,
         "n_links": 1, "height": 0.03, "radius": 0.010},
    ]
    
    modified, report = technique.apply(branches)
    
    # Check petiolules remapped
    assert report.details["petiolules_remapped"] == 2
    
    mod_dict = {b["id"]: b for b in modified}
    pet1 = mod_dict["Petiolule_r1_o0_lf1"]
    pet2 = mod_dict["Petiolule_r1_o0_lf2"]
    
    assert pet1["parent"] == "Leaf_r1_o0_merged"
    assert pet2["parent"] == "Leaf_r1_o0_merged"
    assert pet1["attach_link"] == 1
    assert pet2["attach_link"] == 1


def test_apply_multiple_pairs():
    """Test merging multiple petiole+rachis pairs.

    apply() is intentionally designed to merge ONE pair per call — the
    BudgetOptimizer's while loop calls it repeatedly until can_apply()
    returns False. This test mirrors that pattern.
    """
    technique = LeafBranchReductionTechnique()

    branches = [
        {"id": "trunk", "parent": None},
        # Leaf 1
        {"id": "Leaf_r1_o0_petiole", "parent": "trunk", "attach_link": 1,
         "n_links": 1, "height": 0.10, "radius": 0.030},
        {"id": "Leaf_r1_o0_rachis", "parent": "Leaf_r1_o0_petiole", "attach_link": 1,
         "n_links": 3, "height": 0.05, "radius": 0.020},
        # Leaf 2
        {"id": "Leaf_r2_o0_petiole", "parent": "trunk", "attach_link": 2,
         "n_links": 1, "height": 0.12, "radius": 0.035},
        {"id": "Leaf_r2_o0_rachis", "parent": "Leaf_r2_o0_petiole", "attach_link": 1,
         "n_links": 2, "height": 0.06, "radius": 0.025},
    ]

    # First call: merges one pair
    step1, report1 = technique.apply(branches)
    assert report1.details["pairs_merged"] == 1

    # Second call: merges the remaining pair
    assert technique.can_apply(step1), "Technique should still be applicable after first merge"
    step2, report2 = technique.apply(step1)
    assert report2.details["pairs_merged"] == 1

    # After two iterations both rachis should be gone
    mod_dict = {b["id"]: b for b in step2}
    assert "Leaf_r1_o0_rachis" not in mod_dict, "First rachis should be removed"
    assert "Leaf_r2_o0_rachis" not in mod_dict, "Second rachis should be removed"

    # Cumulative joints saved = 3 + 2 = 5
    total_saved = report1.joints_saved + report2.joints_saved
    assert total_saved == 5, f"Expected 5 total joints saved, got {total_saved}"

    # No more pairs to merge
    assert not technique.can_apply(step2), "No more pairs should remain"


def test_validate_success():
    """Test validation of successful merge."""
    technique = LeafBranchReductionTechnique()
    
    original = [
        {"id": "trunk", "parent": None},
        {"id": "Leaf_r1_o0_petiole", "parent": "trunk", "attach_link": 1,
         "n_links": 1, "height": 0.10, "radius": 0.030},
        {"id": "Leaf_r1_o0_rachis", "parent": "Leaf_r1_o0_petiole", "attach_link": 1,
         "n_links": 3, "height": 0.05, "radius": 0.020},
    ]
    
    modified, _ = technique.apply(original)
    result = technique.validate(original, modified)
    
    assert result.valid is True
    assert len(result.errors) == 0


def test_validate_detects_errors():
    """Test validation detects invalid merges."""
    technique = LeafBranchReductionTechnique()
    
    original = [
        {"id": "trunk", "parent": None},
        {"id": "Leaf_r1_o0_petiole", "parent": "trunk", "attach_link": 1,
         "n_links": 1, "height": 0.10, "radius": 0.030},
        {"id": "Leaf_r1_o0_rachis", "parent": "Leaf_r1_o0_petiole", "attach_link": 1,
         "n_links": 3, "height": 0.05, "radius": 0.020},
    ]
    
    # Invalid: child with broken parent reference
    modified_invalid = [
        {"id": "trunk", "parent": None},
        {"id": "Leaf_r1_o0_rachis", "parent": "Leaf_r1_o0_petiole"}
    ]
    
    result = technique.validate(original, modified_invalid)
    assert result.valid is False
    assert len(result.errors) > 0


def test_no_pairs():
    """Test behavior when no pairs exist."""
    technique = LeafBranchReductionTechnique()
    
    branches = [
        {"id": "trunk", "parent": None},
        {"id": "Branch_r1_o0", "parent": "trunk", "n_links": 3},
    ]
    
    modified, report = technique.apply(branches)
    
    # No changes
    assert report.joints_saved == 0
    assert report.details["pairs_merged"] == 0
    assert modified == branches


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
