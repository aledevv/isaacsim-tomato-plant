"""
test_petiole_lock.py - Unit Tests for Petiole Lock Technique

Tests for converting petiolule joints from D6 to Fixed.

Run with: uv run python src/exporterV2/core/optimizations/tests/4_petiole_lock/test_petiole_lock.py
"""

import sys
import os

# Add optimizations directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
optimizations_dir = os.path.join(script_dir, "../..")
sys.path.insert(0, optimizations_dir)

from techniques.petiole_lock import PetioleLockTechnique


def test_identify_petiolules():
    """Test petiolule identification."""
    print("\n[TEST] Identify Petiolules...")
    
    technique = PetioleLockTechnique()
    
    branches = [
        {"id": "trunk", "n_links": 5},
        {"id": "Branch_r1_o0", "n_links": 3, "parent": "trunk"},
        {"id": "Petiole_r1_o0", "n_links": 2, "parent": "Branch_r1_o0"},
        {"id": "Rachis_r1_o0_l0", "n_links": 1, "parent": "Petiole_r1_o0"},
        {"id": "Petiolule_r1_o0_l0_lf0", "n_links": 1, "parent": "Rachis_r1_o0_l0"},
        {"id": "Petiolule_r1_o0_l0_lf1", "n_links": 1, "parent": "Rachis_r1_o0_l0"},
    ]
    
    petiolule_count = sum(1 for b in branches if technique._is_target(b))
    assert petiolule_count == 2, f"Expected 2 petiolules, found {petiolule_count}"
    print(f"  ✓ Identified {petiolule_count} petiolules")


def test_can_apply():
    """Test can_apply() method."""
    print("\n[TEST] Can Apply...")
    
    technique = PetioleLockTechnique()
    
    # Test 1: With petiolules
    branches_with = [
        {"id": "trunk", "n_links": 5},
        {"id": "Petiolule_r1_o0_l0_lf0", "n_links": 1, "parent": "Rachis_r1_o0_l0"},
    ]
    
    assert technique.can_apply(branches_with), "Should be applicable with petiolules"
    print("  ✓ Can apply with petiolules")
    
    # Test 2: Without petiolules
    branches_without = [
        {"id": "trunk", "n_links": 5},
        {"id": "Branch_r1_o0", "n_links": 3, "parent": "trunk"},
    ]
    
    assert not technique.can_apply(branches_without), "Should not be applicable without petiolules"
    print("  ✓ Cannot apply without petiolules")
    
    # Test 3: With petiolules already fixed
    branches_fixed = [
        {"id": "Petiolule_r1_o0_l0_lf0", "n_links": 1, "joint_type": "fixed"},
    ]
    
    assert not technique.can_apply(branches_fixed), "Should not apply to already-fixed petiolules"
    print("  ✓ Cannot apply to already-fixed petiolules")


def test_estimate_reduction():
    """Test DOF reduction estimation."""
    print("\n[TEST] Estimate Reduction...")
    
    technique = PetioleLockTechnique()
    
    branches = [
        {"id": "trunk", "n_links": 5},
        {"id": "Petiolule_r1_o0_l0_lf0", "n_links": 1},
        {"id": "Petiolule_r1_o0_l0_lf1", "n_links": 1},
        {"id": "Petiolule_r1_o0_l0_lf2", "n_links": 1},
        {"id": "Petiolule_r1_o0_l0_lf3", "n_links": 1, "joint_type": "fixed"},  # Already fixed
    ]
    
    reduction = technique.estimate_reduction(branches)
    # 3 petiolules (each 1 link) = 3 joints (one already fixed, so not counted)
    assert reduction == 3, f"Expected 3 joints reduction, got {reduction}"
    print(f"  ✓ Estimated {reduction} DOF reduction (3 petiolules × 6 DOF)")


def test_apply_simple():
    """Test applying technique to simple case."""
    print("\n[TEST] Apply - Simple Case...")
    
    technique = PetioleLockTechnique()
    
    branches = [
        {"id": "trunk", "n_links": 5},
        {"id": "Petiolule_r1_o0_l0_lf0", "n_links": 1, "parent": "Rachis_r1_o0_l0"},
        {"id": "Petiolule_r1_o0_l0_lf1", "n_links": 1, "parent": "Rachis_r1_o0_l0"},
    ]
    
    modified, report = technique.apply(branches)
    
    assert report.joints_saved == 2, f"Should report 2 joints saved"
    assert report.details["dof_reduced"] == 12, f"Expected 12 DOF reduced, got {report.details['dof_reduced']}"
    
    # Check that petiolules got joint_type metadata
    petiolules_locked = [b for b in modified if b["id"].startswith("Petiolule") and b.get("joint_type") == "fixed"]
    assert len(petiolules_locked) == 2, f"Expected 2 locked petiolules, got {len(petiolules_locked)}"
    
    print(f"  ✓ Locked {len(petiolules_locked)} petiolules")
    print(f"  ✓ Report: DOF reduced = {report.details['dof_reduced']}")


def test_apply_preserves_geometry():
    """Test that apply() preserves geometry."""
    print("\n[TEST] Apply Preserves Geometry...")
    
    technique = PetioleLockTechnique()
    
    branches = [
        {"id": "trunk", "n_links": 5, "height": 0.2, "radius": 0.05},
        {"id": "Petiolule_r1_o0_l0_lf0", "n_links": 1, "height": 0.1, "radius": 0.01, "parent": "Rachis"},
    ]
    
    modified, report = technique.apply(branches)
    
    # Check trunk unchanged
    trunk_orig = branches[0]
    trunk_mod = [b for b in modified if b["id"] == "trunk"][0]
    
    assert trunk_mod["n_links"] == trunk_orig["n_links"], "Trunk n_links changed"
    assert trunk_mod["height"] == trunk_orig["height"], "Trunk height changed"
    assert trunk_mod["radius"] == trunk_orig["radius"], "Trunk radius changed"
    
    # Check petiolule geometry unchanged (only joint_type added)
    pet_orig = branches[1]
    pet_mod = [b for b in modified if b["id"] == "Petiolule_r1_o0_l0_lf0"][0]
    
    assert pet_mod["n_links"] == pet_orig["n_links"], "Petiolule n_links changed"
    assert pet_mod["height"] == pet_orig["height"], "Petiolule height changed"
    assert pet_mod["radius"] == pet_orig["radius"], "Petiolule radius changed"
    assert pet_mod.get("joint_type") == "fixed", "Petiolule should have fixed joint"
    
    print("  ✓ Geometry preserved (n_links, height, radius)")
    print("  ✓ Only joint_type metadata added")


def test_apply_mixed_branches():
    """Test applying to mixed branch types."""
    print("\n[TEST] Apply - Mixed Branches...")
    
    technique = PetioleLockTechnique()
    
    branches = [
        {"id": "trunk", "n_links": 5},
        {"id": "Branch_r1_o0", "n_links": 3, "parent": "trunk"},
        {"id": "Petiole_r1_o0", "n_links": 2, "parent": "Branch_r1_o0"},
        {"id": "Rachis_r1_o0_l0", "n_links": 1, "parent": "Petiole_r1_o0"},
        {"id": "Petiolule_r1_o0_l0_lf0", "n_links": 1, "parent": "Rachis_r1_o0_l0"},
        {"id": "Petiolule_r1_o0_l0_lf1", "n_links": 1, "parent": "Rachis_r1_o0_l0"},
        {"id": "Petiolule_r1_o0_l0_lf2", "n_links": 1, "parent": "Rachis_r1_o0_l0", "joint_type": "fixed"},  # Already fixed
    ]
    
    modified, report = technique.apply(branches)
    
    # Should lock 2 petiolules (third already fixed)
    assert report.details["dof_reduced"] == 12, f"Expected 12 DOF, got {report.details['dof_reduced']}"
    
    # Check only petiolules got joint_type
    non_petiolules_with_joint_type = [
        b for b in modified 
        if not b["id"].startswith("Petiolule") and "joint_type" in b
    ]
    assert len(non_petiolules_with_joint_type) == 0, "Non-petiolules should not have joint_type"
    
    print(f"  ✓ Only petiolules got joint_type metadata")
    print(f"  ✓ DOF reduced: {report.details['dof_reduced']}")


def test_validate():
    """Test validation of optimized branches."""
    print("\n[TEST] Validation...")
    
    technique = PetioleLockTechnique()
    
    original = [
        {"id": "trunk", "n_links": 5, "height": 0.2, "radius": 0.05, "parent": None},
        {"id": "Petiolule_r1_o0_l0_lf0", "n_links": 1, "height": 0.1, "radius": 0.01, "parent": "Rachis"},
    ]
    
    # Test 1: Valid modification (only joint_type added)
    modified_valid = [
        {"id": "trunk", "n_links": 5, "height": 0.2, "radius": 0.05, "parent": None},
        {"id": "Petiolule_r1_o0_l0_lf0", "n_links": 1, "height": 0.1, "radius": 0.01, "parent": "Rachis", "joint_type": "fixed"},
    ]
    
    result = technique.validate(original, modified_valid)
    assert result.valid, f"Valid modification failed: {result.errors}"
    print("  ✓ Valid modification passes")
    
    # Test 2: Invalid modification (geometry changed)
    modified_invalid_geom = [
        {"id": "trunk", "n_links": 5, "height": 0.2, "radius": 0.05, "parent": None},
        {"id": "Petiolule_r1_o0_l0_lf0", "n_links": 2, "height": 0.1, "radius": 0.01, "parent": "Rachis", "joint_type": "fixed"},  # n_links changed!
    ]
    
    result = technique.validate(original, modified_invalid_geom)
    assert not result.valid, "Invalid modification should fail"
    assert len(result.errors) > 0, "Should have error messages"
    print(f"  ✓ Invalid modification rejected: {result.errors[0]}")
    
    # Test 3: Invalid modification (parent changed)
    modified_invalid_parent = [
        {"id": "trunk", "n_links": 5, "height": 0.2, "radius": 0.05, "parent": None},
        {"id": "Petiolule_r1_o0_l0_lf0", "n_links": 1, "height": 0.1, "radius": 0.01, "parent": "DifferentParent", "joint_type": "fixed"},
    ]
    
    result = technique.validate(original, modified_invalid_parent)
    assert not result.valid, "Parent change should fail validation"
    print(f"  ✓ Parent change rejected")


def test_no_petiolules():
    """Test behavior when no petiolules present."""
    print("\n[TEST] No Petiolules...")
    
    technique = PetioleLockTechnique()
    
    branches = [
        {"id": "trunk", "n_links": 5},
        {"id": "Branch_r1_o0", "n_links": 3, "parent": "trunk"},
    ]
    
    assert not technique.can_apply(branches), "Should not apply"
    
    reduction = technique.estimate_reduction(branches)
    assert reduction == 0, f"Expected 0 reduction, got {reduction}"
    
    modified, report = technique.apply(branches)
    assert report.details["dof_reduced"] == 0, "Should report 0 DOF reduced"
    assert report.joints_saved == 0, "Should report 0 joints saved"
    
    print("  ✓ Correctly handles case with no petiolules")


def main():
    """Run all tests."""
    print("="*70)
    print("  Petiole Lock Technique - Test Suite")
    print("="*70)
    
    tests = [
        test_identify_petiolules,
        test_can_apply,
        test_estimate_reduction,
        test_apply_simple,
        test_apply_preserves_geometry,
        test_apply_mixed_branches,
        test_validate,
        test_no_petiolules,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            test()
            passed += 1
        except AssertionError as e:
            failed += 1
            print(f"\n  ✗ FAILED: {e}")
            import traceback
            traceback.print_exc()
        except Exception as e:
            failed += 1
            print(f"\n  ✗ ERROR: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "="*70)
    print(f"  Test Results: {passed} passed, {failed} failed")
    print("="*70)
    
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
