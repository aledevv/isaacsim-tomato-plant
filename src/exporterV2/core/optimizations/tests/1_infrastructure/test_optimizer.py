"""
test_optimizer.py - Unit Tests for BudgetOptimizer

Tests for configuration loading, joint calculation, and lower bound calculation.
"""

import pytest
import tempfile
import yaml
from pathlib import Path

# Add optimizations directory to path
import sys
import os
script_dir = os.path.dirname(os.path.abspath(__file__))
optimizations_dir = os.path.join(script_dir, "../..")
sys.path.insert(0, optimizations_dir)

from optimizer import BudgetOptimizer, BudgetConfig, FullOptimizationReport


# ==============================================================================
# Fixtures
# ==============================================================================

@pytest.fixture
def valid_config_dict():
    """Valid configuration dictionary."""
    return {
        "budget": {
            "max_joints": 250,
            "warning_threshold": 230
        },
        "structural_limits": {
            "trunk": {"min_links": 1, "description": "Main stem"},
            "lateral_branch": {"min_links": 1, "description": "Lateral branch"},
            "petiole": {"min_links": 1, "description": "Petiole"},
            "rachis": {"min_links": 0, "description": "Rachis"},
            "petiolule": {"min_links": 0, "description": "Petiolule"},
            "truss": {"min_links": 1, "description": "Truss"}
        },
        "techniques": [
            {"id": "petiole_lock", "priority": 1, "enabled": True, "params": {}},
            {"id": "lateral_reduce", "priority": 2, "enabled": True, "params": {}},
        ],
        "logging": {
            "level": "INFO"
        }
    }


@pytest.fixture
def temp_config_file(valid_config_dict):
    """Create temporary config file."""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.dump(valid_config_dict, f)
        temp_path = f.name
    
    yield temp_path
    
    # Cleanup
    os.unlink(temp_path)


@pytest.fixture
def simple_branches():
    """Simple branches configuration for testing."""
    return [
        {
            "id": "trunk",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.05,
            "height": 0.20,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "Branch_r1_o0",
            "parent": "trunk",
            "attach_link": 2,
            "n_links": 3,
            "radius": 0.02,
            "height": 0.15,
            "tilt": 45.0,
            "rot": 0.0,
        },
        {
            "id": "Petiole_r1_o0",
            "parent": "Branch_r1_o0",
            "attach_link": 1,
            "n_links": 2,
            "radius": 0.01,
            "height": 0.10,
            "tilt": 30.0,
            "rot": 90.0,
        }
    ]


# ==============================================================================
# Configuration Loading Tests
# ==============================================================================

def test_load_valid_config(temp_config_file):
    """Test loading valid configuration from YAML."""
    config = BudgetConfig.load(temp_config_file)
    
    assert config.max_joints == 250
    assert config.warning_threshold == 230
    assert config.max_rigid_bodies is None
    assert "trunk" in config.structural_limits
    assert len(config.techniques) == 2
    assert config.techniques[0]["priority"] == 1  # Sorted by priority


def test_load_config_file_not_found():
    """Test error when config file doesn't exist."""
    with pytest.raises(FileNotFoundError, match="Config file not found"):
        BudgetConfig.load("/nonexistent/path/config.yaml")


def test_load_config_missing_section(tmp_path):
    """Test error when required section is missing."""
    # Create config missing 'budget' section
    invalid_config = {
        "structural_limits": {},
        "techniques": []
    }
    
    config_path = tmp_path / "invalid.yaml"
    with open(config_path, 'w') as f:
        yaml.dump(invalid_config, f)
    
    with pytest.raises(ValueError, match="Missing required section 'budget'"):
        BudgetConfig.load(str(config_path))


def test_load_config_invalid_max_joints(tmp_path):
    """Test error when max_joints is invalid."""
    invalid_config = {
        "budget": {"max_joints": 0},  # Invalid: must be positive
        "structural_limits": {},
        "techniques": []
    }
    
    config_path = tmp_path / "invalid.yaml"
    with open(config_path, 'w') as f:
        yaml.dump(invalid_config, f)
    
    with pytest.raises(ValueError, match="max_joints must be positive"):
        BudgetConfig.load(str(config_path))


def test_techniques_sorted_by_priority(temp_config_file):
    """Test that techniques are sorted by priority."""
    config = BudgetConfig.load(temp_config_file)
    
    priorities = [t["priority"] for t in config.techniques]
    assert priorities == sorted(priorities), "Techniques should be sorted by priority"


# ==============================================================================
# Optimizer Initialization Tests
# ==============================================================================

def test_optimizer_init_with_config_path(temp_config_file):
    """Test optimizer initialization with explicit config path."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    
    assert optimizer.config.max_joints == 250


def test_optimizer_init_auto_detect_config():
    """Test optimizer initialization with auto-detected config."""
    # This should find budget_config.yaml in the optimizations directory
    optimizer = BudgetOptimizer(max_joints=250)
    
    assert optimizer.config.max_joints > 0
    assert optimizer.config.structural_limits is not None


# ==============================================================================
# Joint Calculation Tests
# ==============================================================================

def test_calculate_total_joints_empty(temp_config_file):
    """Test joint calculation with empty branches."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    branches = []
    
    total = optimizer.calculate_total_joints(branches)
    assert total == 0


def test_calculate_total_joints_simple(temp_config_file, simple_branches):
    """Test joint calculation with simple branches."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    
    total = optimizer.calculate_total_joints(simple_branches)
    expected = 5 + 3 + 2  # trunk + branch + petiole
    assert total == expected


def test_calculate_total_joints_single_branch(temp_config_file):
    """Test joint calculation with single branch."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    branches = [{"id": "trunk", "n_links": 10, "parent": None}]
    
    total = optimizer.calculate_total_joints(branches)
    assert total == 10


# ==============================================================================
# Lower Bound Calculation Tests
# ==============================================================================

def test_calculate_lower_bound_trunk_only(temp_config_file):
    """Test lower bound with only trunk."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    branches = [
        {"id": "trunk", "parent": None, "n_links": 10}
    ]
    
    lower_bound = optimizer.calculate_lower_bound(branches)
    assert lower_bound == 1  # trunk min = 1


def test_calculate_lower_bound_with_laterals(temp_config_file):
    """Test lower bound with trunk + lateral branches."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    branches = [
        {"id": "trunk", "parent": None, "n_links": 10},
        {"id": "Branch_r1_o0", "parent": "trunk", "n_links": 5},
        {"id": "Branch_r2_o0", "parent": "trunk", "n_links": 5},
    ]
    
    lower_bound = optimizer.calculate_lower_bound(branches)
    assert lower_bound == 1 + 2  # trunk (1) + 2 laterals (1 each)


def test_calculate_lower_bound_complex(temp_config_file, simple_branches):
    """Test lower bound with complex plant."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    
    lower_bound = optimizer.calculate_lower_bound(simple_branches)
    # trunk (1) + lateral branch (1) + petiole (1) = 3
    assert lower_bound == 3


def test_calculate_lower_bound_with_truss(temp_config_file):
    """Test lower bound includes truss."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    branches = [
        {"id": "trunk", "parent": None, "n_links": 5},
        {"id": "Truss_r1", "parent": "trunk", "n_links": 4},
    ]
    
    lower_bound = optimizer.calculate_lower_bound(branches)
    assert lower_bound == 1 + 1  # trunk + truss


# ==============================================================================
# Optimize Tests (Skeleton - Full Implementation in Later Tasks)
# ==============================================================================

def test_optimize_already_within_budget(temp_config_file):
    """Test optimization when already within budget."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    branches = [
        {"id": "trunk", "parent": None, "n_links": 10}
    ]
    
    optimized, report = optimizer.optimize(branches)
    
    assert report.success is True
    assert report.original_joints == 10
    assert report.final_joints == 10
    assert len(report.technique_reports) == 0
    assert optimized == branches  # No changes


def test_rigid_body_budget_is_diagnostic_only(valid_config_dict, tmp_path):
    """Record rigid-body pressure without changing the D6 stopping condition."""
    config = valid_config_dict.copy()
    config["budget"] = {
        "max_joints": 10,
        "max_rigid_bodies": 7,
        "warning_threshold": 8,
    }
    config["techniques"] = [
        {
            "id": "leaf_branch_reduce",
            "priority": 1,
            "enabled": True,
            "params": {},
        }
    ]
    config_path = tmp_path / "body_budget.yaml"
    config_path.write_text(yaml.dump(config))

    branches = [
        {"id": "trunk", "parent": None, "n_links": 1, "joint_type": "fixed"},
        {
            "id": "Leaf_r1_o0_petiole",
            "parent": "trunk",
            "attach_link": 1,
            "n_links": 1,
            "height": 0.02,
            "radius": 0.002,
        },
        {
            "id": "Leaf_r1_o0_rachis",
            "parent": "Leaf_r1_o0_petiole",
            "attach_link": 1,
            "n_links": 3,
            "height": 0.02,
            "radius": 0.001,
        },
        {
            "id": "Leaf_r1_o0_rachis_petiolule_term",
            "parent": "Leaf_r1_o0_rachis",
            "attach_link": 3,
            "n_links": 1,
            "height": 0.01,
            "radius": 0.001,
        },
    ]

    optimizer = BudgetOptimizer(config_path=str(config_path))
    optimized, report = optimizer.optimize(branches, terminal_body_count=3)

    assert report.success is True
    assert report.original_joints <= report.budget
    assert report.original_rigid_bodies == 9
    assert report.final_rigid_bodies == report.original_rigid_bodies
    assert report.final_rigid_bodies > report.rigid_body_budget
    assert report.technique_reports == []
    assert optimized == branches


def test_optimize_budget_impossible(temp_config_file):
    """Test optimization fails when budget impossible to meet."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    
    # Create plant with lower bound > budget
    # Lower bound = 100 trunks * 1 = 100
    # Budget = 250, but we'll set lower bound > budget by creating many components
    branches = []
    for i in range(300):  # 300 lateral branches
        branches.append({
            "id": f"Branch_r{i}_o0",
            "parent": "trunk",
            "n_links": 1
        })
    branches.append({"id": "trunk", "parent": None, "n_links": 1})
    
    # Lower bound = 1 (trunk) + 300 (laterals) = 301 > 250 (budget)
    with pytest.raises(ValueError, match="Budget impossible to meet"):
        optimizer.optimize(branches)


def test_optimize_report_structure(temp_config_file, simple_branches):
    """Test that optimization report has correct structure."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    
    optimized, report = optimizer.optimize(simple_branches)
    
    assert isinstance(report, FullOptimizationReport)
    assert report.original_joints > 0
    assert report.final_joints >= 0
    assert report.budget == 250
    assert report.lower_bound >= 0
    assert isinstance(report.success, bool)
    assert isinstance(report.technique_reports, list)


def test_optimize_report_string_representation(temp_config_file, simple_branches):
    """Test that report can be converted to string."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    
    optimized, report = optimizer.optimize(simple_branches)
    
    report_str = str(report)
    assert "Joint-Budget Optimization Report" in report_str
    assert "Original joints:" in report_str
    assert "Budget:" in report_str
    assert "Structural lower bound:" in report_str


# ==============================================================================
# Edge Cases
# ==============================================================================

def test_calculate_joints_with_zero_links(temp_config_file):
    """Test joint calculation with branches having 0 links."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    branches = [
        {"id": "trunk", "n_links": 5},
        {"id": "empty", "n_links": 0}
    ]
    
    total = optimizer.calculate_total_joints(branches)
    assert total == 5


def test_lower_bound_calculation_stability(temp_config_file, simple_branches):
    """Test that lower bound calculation is stable across multiple calls."""
    optimizer = BudgetOptimizer(config_path=temp_config_file)
    
    lb1 = optimizer.calculate_lower_bound(simple_branches)
    lb2 = optimizer.calculate_lower_bound(simple_branches)
    lb3 = optimizer.calculate_lower_bound(simple_branches)
    
    assert lb1 == lb2 == lb3


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
