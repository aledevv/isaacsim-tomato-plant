"""
test_optimizer_simple.py - Simple Tests without pytest

Manual testing script for BudgetOptimizer functionality.
Run directly with: python3 test_optimizer_simple.py
"""

import sys
import os
import tempfile
import yaml

# Add optimizations directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
optimizations_dir = os.path.join(script_dir, "../..")  # Go up two levels to optimizations/
sys.path.insert(0, optimizations_dir)

from optimizer import BudgetOptimizer, BudgetConfig


def test_config_loading():
    """Test configuration loading from YAML."""
    print("\n[TEST] Configuration Loading...")
    
    # Create temporary config
    config_data = {
        "budget": {"max_joints": 250, "warning_threshold": 230},
        "structural_limits": {
            "trunk": {"min_links": 1},
            "lateral_branch": {"min_links": 1},
            "petiole": {"min_links": 1},
            "rachis": {"min_links": 0},
            "petiolule": {"min_links": 0},
            "truss": {"min_links": 1}
        },
        "techniques": [
            {"id": "petiole_lock", "priority": 1, "enabled": True},
            {"id": "lateral_reduce", "priority": 2, "enabled": True},
        ],
        "logging": {"level": "INFO"}
    }
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.dump(config_data, f)
        temp_path = f.name
    
    try:
        config = BudgetConfig.load(temp_path)
        assert config.max_joints == 250, "max_joints mismatch"
        assert config.warning_threshold == 230, "warning_threshold mismatch"
        assert len(config.techniques) == 2, "techniques count mismatch"
        print("  ✓ Config loading works")
    finally:
        os.unlink(temp_path)


def test_joint_calculation():
    """Test total joint calculation."""
    print("\n[TEST] Joint Calculation...")
    
    optimizer = BudgetOptimizer(max_joints=250)
    
    # Test 1: Empty branches
    branches = []
    total = optimizer.calculate_total_joints(branches)
    assert total == 0, f"Expected 0, got {total}"
    print(f"  ✓ Empty branches: {total} joints")
    
    # Test 2: Single branch
    branches = [{"id": "trunk", "n_links": 10}]
    total = optimizer.calculate_total_joints(branches)
    assert total == 10, f"Expected 10, got {total}"
    print(f"  ✓ Single branch (10 links): {total} joints")
    
    # Test 3: Multiple branches
    branches = [
        {"id": "trunk", "n_links": 5},
        {"id": "branch1", "n_links": 3},
        {"id": "branch2", "n_links": 2},
    ]
    total = optimizer.calculate_total_joints(branches)
    assert total == 10, f"Expected 10, got {total}"
    print(f"  ✓ Multiple branches (5+3+2): {total} joints")


def test_lower_bound_calculation():
    """Test lower bound calculation."""
    print("\n[TEST] Lower Bound Calculation...")
    
    optimizer = BudgetOptimizer(max_joints=250)
    
    # Test 1: Trunk only
    branches = [{"id": "trunk", "parent": None, "n_links": 10}]
    lb = optimizer.calculate_lower_bound(branches)
    assert lb == 1, f"Expected 1 (trunk min), got {lb}"
    print(f"  ✓ Trunk only: lower bound = {lb}")
    
    # Test 2: Trunk + lateral branches
    branches = [
        {"id": "trunk", "parent": None, "n_links": 10},
        {"id": "Branch_r1_o0", "parent": "trunk", "n_links": 5},
        {"id": "Branch_r2_o0", "parent": "trunk", "n_links": 5},
    ]
    lb = optimizer.calculate_lower_bound(branches)
    assert lb == 3, f"Expected 3 (trunk + 2 laterals), got {lb}"
    print(f"  ✓ Trunk + 2 laterals: lower bound = {lb}")
    
    # Test 3: Complex plant
    branches = [
        {"id": "trunk", "parent": None, "n_links": 5},
        {"id": "Branch_r1_o0", "parent": "trunk", "n_links": 3},
        {"id": "Petiole_r1_o0", "parent": "Branch_r1_o0", "n_links": 2},
    ]
    lb = optimizer.calculate_lower_bound(branches)
    expected = 1 + 1 + 1  # trunk + lateral + petiole
    assert lb == expected, f"Expected {expected}, got {lb}"
    print(f"  ✓ Complex plant: lower bound = {lb}")


def test_optimize_within_budget():
    """Test optimization when already within budget."""
    print("\n[TEST] Optimize - Already Within Budget...")
    
    optimizer = BudgetOptimizer(max_joints=250)
    branches = [{"id": "trunk", "parent": None, "n_links": 10}]
    
    optimized, report = optimizer.optimize(branches)
    
    assert report.success is True, "Should succeed"
    assert report.original_joints == 10, "Original joints mismatch"
    assert report.final_joints == 10, "Final joints mismatch"
    assert len(report.technique_reports) == 0, "Should apply no techniques"
    print(f"  ✓ No optimization needed (10 joints < 250 budget)")
    print(f"  ✓ Report: {report.success}, {report.original_joints} → {report.final_joints}")


def test_optimize_impossible_budget():
    """Test optimization fails when budget impossible."""
    print("\n[TEST] Optimize - Impossible Budget...")
    
    optimizer = BudgetOptimizer(max_joints=250)
    
    # Create 300 lateral branches (lower bound = 1 trunk + 300 laterals = 301 > 250)
    branches = [{"id": "trunk", "parent": None, "n_links": 1}]
    for i in range(300):
        branches.append({
            "id": f"Branch_r{i}_o0",
            "parent": "trunk",
            "n_links": 1
        })
    
    try:
        optimized, report = optimizer.optimize(branches)
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "Budget impossible to meet" in str(e), "Wrong error message"
        print(f"  ✓ Correctly raised error: Budget impossible")
        print(f"    Lower bound: 301, Budget: 250")


def test_report_formatting():
    """Test report string formatting."""
    print("\n[TEST] Report Formatting...")
    
    optimizer = BudgetOptimizer(max_joints=250)
    branches = [{"id": "trunk", "parent": None, "n_links": 10}]
    
    optimized, report = optimizer.optimize(branches)
    report_str = str(report)
    
    assert "Joint-Budget Optimization Report" in report_str, "Missing header"
    assert "Original joints:" in report_str, "Missing original joints"
    assert "Budget:" in report_str, "Missing budget"
    assert "Structural lower bound:" in report_str, "Missing lower bound"
    print(f"  ✓ Report formatting correct")
    print("\n" + "="*60)
    print("Sample Report:")
    print(report_str)
    print("="*60)


def main():
    """Run all tests."""
    print("="*60)
    print("  BudgetOptimizer - Simple Test Suite")
    print("="*60)
    
    tests = [
        test_config_loading,
        test_joint_calculation,
        test_lower_bound_calculation,
        test_optimize_within_budget,
        test_optimize_impossible_budget,
        test_report_formatting,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            failed += 1
            print(f"\n  ✗ FAILED: {e}")
            import traceback
            traceback.print_exc()
    
    print("\n" + "="*60)
    print(f"  Test Results: {passed} passed, {failed} failed")
    print("="*60)
    
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
