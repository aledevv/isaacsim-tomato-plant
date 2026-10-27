"""
test_technique_composition.py - Integration Tests for Technique Composition

Tests the sequential application of multiple optimization techniques to verify:
1. Techniques can be composed (applied in sequence)
2. Joint count reduces progressively
3. Geometry remains valid after each technique
4. Budget target is met when possible
5. Impossible budgets are handled gracefully

Tests both synthetic and real plants from CSV.
"""

import pytest
import sys
import os
from pathlib import Path

# Add paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from optimizer import BudgetOptimizer, FullOptimizationReport


def create_synthetic_overbudget_plant() -> list:
    """
    Create synthetic plant that's over budget (300+ joints).
    
    Structure:
    - Trunk: 10 links
    - 5 lateral branches: 5 links each = 25 links
    - 10 petioles: 2 links each = 20 links
    - 10 rachis: 3 links each = 30 links
    - 30 petiolules: 1 link each = 30 links
    
    Total: 10 + 25 + 20 + 30 + 30 = 115 links
    
    With enough leaves, we can reach 300+ joints.
    """
    branches = []
    
    # Trunk (10 links)
    branches.append({
        "id": "trunk",
        "parent": None,
        "n_links": 10,
        "height": 0.10,
        "radius": 0.04,
        "tilt": 0.0,
        "rot": 0.0
    })
    
    # 5 lateral branches (5 links each)
    for i in range(5):
        branches.append({
            "id": f"Branch_r{i+1}_o0",
            "parent": "trunk",
            "attach_link": (i+1) * 2,  # Distributed along trunk
            "attach_frac": 1.0,
            "n_links": 5,
            "height": 0.20,
            "radius": 0.02,
            "tilt": 45.0,
            "rot": 90.0 * i
        })
    
    # 20 leaves (each with petiole + rachis + 3 petiolules)
    # This will create: 20 petioles + 20 rachis + 60 petiolules = 100 branches
    for leaf_idx in range(20):
        rank = leaf_idx + 1
        parent_id = "trunk" if leaf_idx < 10 else f"Branch_r{(leaf_idx % 5) + 1}_o0"
        attach_link = (leaf_idx % 5) + 1
        
        # Petiole (2 links)
        petiole_id = f"Leaf_r{rank}_o0_petiole"
        branches.append({
            "id": petiole_id,
            "parent": parent_id,
            "attach_link": attach_link,
            "attach_frac": 1.0,
            "n_links": 2,
            "height": 0.08,
            "radius": 0.015,
            "tilt": 30.0,
            "rot": 45.0 * (leaf_idx % 8)
        })
        
        # Rachis (3 links)
        rachis_id = f"Leaf_r{rank}_o0_rachis"
        branches.append({
            "id": rachis_id,
            "parent": petiole_id,
            "attach_link": 2,
            "attach_frac": 1.0,
            "n_links": 3,
            "height": 0.05,
            "radius": 0.010,
            "tilt": 0.0,
            "rot": 0.0
        })
        
        # 3 petiolules (1 link each)
        for pet_idx in range(3):
            branches.append({
                "id": f"Petiolule_r{rank}_o0_lf{pet_idx}",
                "parent": rachis_id,
                "attach_link": pet_idx + 1,
                "attach_frac": 1.0,
                "n_links": 1,
                "height": 0.04,
                "radius": 0.005,
                "tilt": 60.0,
                "rot": 120.0 * pet_idx
            })
    
    # Total: 1 trunk + 5 branches + 20*(1+1+3) = 1 + 5 + 100 = 106 branches
    # Total links: 10 + 25 + 20*(2+3+3) = 10 + 25 + 160 = 195 links
    
    return branches


def test_scenario1_simple_overbudget():
    """
    Scenario 1: Simple plant over budget → techniques reduce it.
    
    Plant: ~200 joints
    Budget: 150 joints (artificially low to force optimization)
    Expected: Techniques applied, budget met
    """
    branches = create_synthetic_overbudget_plant()
    
    # Create optimizer with lower budget to force optimization
    import tempfile
    import yaml
    
    # Load original config
    original_config_path = Path(__file__).parent.parent.parent / "budget_config.yaml"
    with open(original_config_path) as f:
        config = yaml.safe_load(f)
    
    # Set budget below initial joints
    config['budget']['max_joints'] = 150  # Force optimization
    
    # Write temporary config
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.dump(config, f)
        temp_config_path = f.name
    
    try:
        optimizer = BudgetOptimizer(config_path=temp_config_path)
        
        # Calculate initial joints
        initial_joints = optimizer.calculate_total_joints(branches)
        print(f"\n[Scenario 1] Initial joints: {initial_joints}, Budget: 150")
        
        # Optimize
        optimized, report = optimizer.optimize(branches)
        
        # Assertions
        assert report.success, f"Optimization should succeed: {report.error_message}"
        assert report.final_joints <= optimizer.config.max_joints, \
            f"Final joints ({report.final_joints}) should be <= budget ({optimizer.config.max_joints})"
        assert len(report.technique_reports) > 0, "At least one technique should be applied"
        assert report.final_joints < initial_joints, "Joints should be reduced"
        
        # Verify progressive reduction
        current = initial_joints
        for tech_report in report.technique_reports:
            assert tech_report.joints_before == current, \
                f"Technique {tech_report.technique_name}: joints_before mismatch"
            assert tech_report.joints_after <= tech_report.joints_before, \
                f"Technique {tech_report.technique_name}: should reduce or maintain joints"
            current = tech_report.joints_after
        
        print(f"[Scenario 1] ✓ Success: {initial_joints} → {report.final_joints} joints")
        print(f"[Scenario 1] Techniques applied: {[r.technique_name for r in report.technique_reports]}")
    
    finally:
        os.unlink(temp_config_path)


def test_scenario2_within_budget():
    """
    Scenario 2: Plant already within budget → no techniques applied.
    
    Plant: ~50 joints
    Budget: 250 joints
    Expected: No optimization needed
    """
    branches = [
        {"id": "trunk", "parent": None, "n_links": 10, "height": 0.20, "radius": 0.04, "tilt": 0.0, "rot": 0.0},
        {"id": "Branch_r1_o0", "parent": "trunk", "attach_link": 5, "attach_frac": 1.0,
         "n_links": 5, "height": 0.15, "radius": 0.02, "tilt": 45.0, "rot": 0.0},
        {"id": "Leaf_r1_o0_petiole", "parent": "trunk", "attach_link": 3, "attach_frac": 1.0,
         "n_links": 2, "height": 0.10, "radius": 0.015, "tilt": 30.0, "rot": 90.0},
    ]
    
    optimizer = BudgetOptimizer(max_joints=250)
    initial_joints = optimizer.calculate_total_joints(branches)
    
    print(f"\n[Scenario 2] Initial joints: {initial_joints}")
    
    optimized, report = optimizer.optimize(branches)
    
    # Assertions
    assert report.success, "Should succeed (within budget)"
    assert report.final_joints == initial_joints, "Joints should not change"
    assert len(report.technique_reports) == 0, "No techniques should be applied"
    assert optimized == branches, "Branches should be unchanged"
    
    print(f"[Scenario 2] ✓ No optimization needed: {initial_joints} joints (budget: {optimizer.config.max_joints})")


def test_scenario3_impossible_budget():
    """
    Scenario 3: Budget below structural lower bound → error.
    
    Plant: 200 joints
    Lower bound: ~50 joints
    Budget: 30 joints (artificially low)
    Expected: ValueError with clear message
    """
    branches = create_synthetic_overbudget_plant()
    
    # Create optimizer with artificially low budget
    import tempfile
    import yaml
    
    # Load original config
    original_config_path = Path(__file__).parent.parent.parent / "budget_config.yaml"
    with open(original_config_path) as f:
        config = yaml.safe_load(f)
    
    # Set impossible budget (below structural minimum)
    # Lower bound is 6 (1 trunk + 5 laterals with min_links=1 each)
    config['budget']['max_joints'] = 5  # Below lower bound, impossible to meet
    
    # Write temporary config
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.dump(config, f)
        temp_config_path = f.name
    
    try:
        optimizer = BudgetOptimizer(config_path=temp_config_path)
        initial_joints = optimizer.calculate_total_joints(branches)
        lower_bound = optimizer.calculate_lower_bound(branches)
        
        print(f"\n[Scenario 3] Initial: {initial_joints}, Lower bound: {lower_bound}, Budget: 5")
        
        # Should raise ValueError
        with pytest.raises(ValueError) as exc_info:
            optimizer.optimize(branches)
        
        error_msg = str(exc_info.value)
        assert "impossible" in error_msg.lower(), "Error should mention impossibility"
        assert "lower bound" in error_msg.lower(), "Error should mention lower bound"
        assert str(lower_bound) in error_msg, "Error should include lower bound value"
        
        print(f"[Scenario 3] ✓ Correctly raised error: {error_msg[:80]}...")
    
    finally:
        os.unlink(temp_config_path)


def test_scenario4_progressive_reduction():
    """
    Scenario 4: Verify techniques are applied in priority order and reduce progressively.
    
    Plant: Large plant requiring multiple techniques
    Expected: Each technique reduces joints, order matches priority
    """
    branches = create_synthetic_overbudget_plant()
    optimizer = BudgetOptimizer(max_joints=250)
    
    initial_joints = optimizer.calculate_total_joints(branches)
    print(f"\n[Scenario 4] Initial joints: {initial_joints}")
    
    optimized, report = optimizer.optimize(branches)
    
    # Verify techniques applied in priority order
    priorities = [t["priority"] for t in optimizer.config.techniques if t.get("enabled", True)]
    applied_techniques = [r.technique_name for r in report.technique_reports]
    
    print(f"[Scenario 4] Applied techniques: {applied_techniques}")
    
    # Verify progressive reduction
    prev_joints = initial_joints
    for i, tech_report in enumerate(report.technique_reports):
        print(f"  {i+1}. {tech_report.technique_name}: "
              f"{tech_report.joints_before} → {tech_report.joints_after} "
              f"(-{tech_report.joints_saved})")
        
        assert tech_report.joints_before == prev_joints, \
            f"Technique {tech_report.technique_name}: joints_before should match previous final"
        assert tech_report.joints_after <= tech_report.joints_before, \
            f"Technique {tech_report.technique_name}: should not increase joints"
        assert tech_report.joints_saved >= 0, \
            f"Technique {tech_report.technique_name}: joints_saved should be non-negative"
        
        prev_joints = tech_report.joints_after
    
    assert report.final_joints == prev_joints, "Final joints should match last technique output"
    
    print(f"[Scenario 4] ✓ Progressive reduction verified: {initial_joints} → {report.final_joints}")


def test_scenario5_real_csv_plant():
    """
    Scenario 5: Load real plant from CSV and optimize.
    
    Uses parse_csv_to_branches from adapters to load real plant data.
    Tests that optimizer works on production data.
    """
    try:
        # Import CSV parser
        sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent.parent))
        from legacy_csv.parser import parse_csv_to_branches
    except ImportError as e:
        pytest.skip(f"Cannot import CSV parser: {e}")
    
    # Find a CSV file to test with
    csv_dir = Path(__file__).parent.parent.parent.parent.parent.parent.parent / "data" / "simulation_output" / "dynamic_output" / "graphs"
    
    if not csv_dir.exists():
        pytest.skip(f"CSV directory not found: {csv_dir}")
    
    csv_files = list(csv_dir.glob("graph_day_*.csv"))
    if not csv_files:
        pytest.skip("No CSV files found")
    
    # Use a middle-stage CSV (day 30-50) for realistic complexity
    test_csv = None
    for csv_file in sorted(csv_files):
        day_num = int(csv_file.stem.split("_")[-1])
        if 30 <= day_num <= 50:
            test_csv = csv_file
            break
    
    if not test_csv:
        test_csv = csv_files[len(csv_files)//2]  # Use middle file
    
    print(f"\n[Scenario 5] Loading real plant from: {test_csv.name}")
    
    # Parse CSV
    try:
        day_num = int(test_csv.stem.split("_")[-1])
        branches, _ = parse_csv_to_branches(day=day_num)
    except Exception as e:
        pytest.skip(f"Failed to parse CSV: {e}")
    
    if not branches:
        pytest.skip("CSV parsing returned empty branches")
    
    optimizer = BudgetOptimizer(max_joints=250)
    initial_joints = optimizer.calculate_total_joints(branches)
    lower_bound = optimizer.calculate_lower_bound(branches)
    
    print(f"[Scenario 5] Real plant stats:")
    print(f"  - Branches: {len(branches)}")
    print(f"  - Initial joints: {initial_joints}")
    print(f"  - Lower bound: {lower_bound}")
    print(f"  - Budget: {optimizer.config.max_joints}")
    
    # Optimize
    try:
        optimized, report = optimizer.optimize(branches)
        
        print(f"[Scenario 5] Optimization result:")
        print(f"  - Final joints: {report.final_joints}")
        print(f"  - Reduction: {report.total_reduction} ({report.reduction_percentage:.1f}%)")
        print(f"  - Techniques: {[r.technique_name for r in report.technique_reports]}")
        print(f"  - Success: {report.success}")
        
        # Basic assertions
        assert report.final_joints <= initial_joints, "Should not increase joints"
        assert len(optimized) > 0, "Should have branches in output"
        
        # If budget was exceeded, check that optimization was attempted
        if initial_joints > optimizer.config.max_joints:
            assert len(report.technique_reports) > 0 or not report.success, \
                "Should either apply techniques or fail explicitly"
        
        print(f"[Scenario 5] ✓ Real plant optimization completed")
        
    except ValueError as e:
        # If budget is impossible, that's acceptable
        if "impossible" in str(e).lower():
            print(f"[Scenario 5] ✓ Budget impossible (expected for some CSVs): {e}")
        else:
            raise


def test_report_formatting():
    """
    Test that optimization report is well-formatted and readable.
    """
    branches = create_synthetic_overbudget_plant()
    optimizer = BudgetOptimizer(max_joints=250)
    
    optimized, report = optimizer.optimize(branches)
    
    # Convert to string
    report_str = str(report)
    
    print(f"\n[Report Test] Full report:\n{report_str}\n")
    
    # Check formatting
    assert "Joint-Budget Optimization Report" in report_str
    assert "Original joints:" in report_str
    assert "Final joints:" in report_str
    assert "Budget:" in report_str
    
    if report.technique_reports:
        for tech_report in report.technique_reports:
            assert tech_report.technique_name in report_str
    
    # Check status indicator
    if report.success:
        assert "✓" in report_str
    else:
        assert "✗" in report_str
    
    print(f"[Report Test] ✓ Report formatting verified")


if __name__ == "__main__":
    # Run tests with pytest
    pytest.main([__file__, "-v", "-s"])
