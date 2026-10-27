"""
test_integration.py - Integration Tests for Optimization Pipeline

Tests the complete flow: CSV → parse → optimize → verify
"""

import pytest
import sys
from pathlib import Path

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))
sys.path.insert(0, str(Path(__file__).parent.parent))

from exporterV2.legacy_csv import parse_csv_to_branches
from exporterV2.core.optimizations import BudgetOptimizer
from exporterV2.core.optimizations.techniques.base import count_d6_joints


def test_optimize_csv_day_100():
    """Test optimization on real CSV data (day 100)."""
    # Parse CSV
    branches, _ = parse_csv_to_branches(day=100, plant_id=1)
    
    original_joints = count_d6_joints(branches)
    print(f"\nOriginal joints: {original_joints}")
    
    # Optimize
    optimizer = BudgetOptimizer(max_joints=250)
    optimized_branches, report = optimizer.optimize(branches)
    
    final_joints = count_d6_joints(optimized_branches)
    print(f"Final joints: {final_joints}")
    print(f"Budget: {optimizer.config.max_joints}")
    print(f"Reduction: {original_joints - final_joints} joints")
    
    # Verify result is valid
    assert final_joints > 0, "Should have at least some joints remaining"
    assert final_joints <= optimizer.config.max_joints, "Should be within budget"
    
    # Verify report
    assert report.original_joints == original_joints
    assert report.final_joints == final_joints
    assert report.total_reduction == original_joints - final_joints
    
    # If already within budget, no techniques should be applied
    if original_joints <= optimizer.config.max_joints:
        assert report.total_reduction == 0, "Should not optimize if already within budget"
        assert len(report.technique_reports) == 0, "No techniques should be applied"
        print("✓ Plant already within budget, no optimization needed")
    else:
        # If over budget, optimization should reduce joints
        assert final_joints < original_joints, "Optimization should reduce joint count"
        assert len(report.technique_reports) > 0, "At least one technique should be applied"
        print(f"✓ Optimization reduced joints: {original_joints} → {final_joints}")


def test_optimize_within_budget():
    """Test that optimizer stops when budget is met."""
    # Create a simple plant that can be optimized within budget
    branches = [
        {"id": "trunk", "n_links": 10, "height": 0.1, "radius": 0.04},
        {
            "id": "branch1",
            "parent": "trunk",
            "attach_link": 5,
            "n_links": 5,
            "height": 0.08,
            "radius": 0.03,
        },
    ]
    
    original_joints = count_d6_joints(branches)
    
    # Optimize with default budget (250)
    optimizer = BudgetOptimizer(max_joints=250)
    optimized_branches, report = optimizer.optimize(branches)
    
    final_joints = count_d6_joints(optimized_branches)
    
    # Should be within budget
    assert final_joints <= optimizer.config.max_joints
    assert report.success


def test_impossible_budget_error():
    """Test that impossible budget raises clear error."""
    # Create a minimal plant
    branches = [
        {"id": "trunk", "n_links": 1, "height": 0.1, "radius": 0.04},
    ]
    
    # Modify config to have impossible budget (below lower bound)
    optimizer = BudgetOptimizer(max_joints=250)
    
    # Lower bound is 0 (trunk with 1 link), so set budget to -1 (impossible)
    optimizer.config.max_joints = -1
    
    with pytest.raises(ValueError) as exc_info:
        optimizer.optimize(branches)
    
    # Check error message is clear
    error_msg = str(exc_info.value)
    assert "budget" in error_msg.lower() or "lower bound" in error_msg.lower()


def test_no_optimization_needed():
    """Test that optimizer returns unchanged when already within budget."""
    # Create very small plant (well within budget)
    branches = [
        {"id": "trunk", "n_links": 2, "height": 0.1, "radius": 0.04},
    ]
    
    original_joints = count_d6_joints(branches)
    
    optimizer = BudgetOptimizer(max_joints=250)
    optimized_branches, report = optimizer.optimize(branches)
    
    final_joints = count_d6_joints(optimized_branches)
    
    # Should be unchanged
    assert final_joints == original_joints
    assert report.total_reduction == 0
    assert len(report.technique_reports) == 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
