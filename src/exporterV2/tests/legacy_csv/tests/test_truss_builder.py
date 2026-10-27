"""
test_truss_builder.py - Unit tests for truss_builder

Tests truss branch generation functions.

Run with:
    python test_truss_builder.py
"""

import sys
from pathlib import Path

# Add parent directory to path
script_dir = Path(__file__).parent
adapters_dir = script_dir.parent
sys.path.insert(0, str(adapters_dir))

from truss_builder import (
    _load_tree_config,
    truss_rachis_to_branch,
    create_lateral_pedicels,
    create_terminal_pedicel,
    truss_to_branch_config,
    create_tomato_definitions,
    truss_to_complete_config,
)


def test_rachis_generation():
    """Test basic rachis branch generation."""
    print("\n" + "="*80)
    print("TEST: Rachis Generation")
    print("="*80)
    
    truss_dict = {
        "rachis_length": 0.12,  # 12cm
        "rachis_radius": 0.001,  # 1mm
        "n_fruits": 5,
        "parent_rank": 3,
    }
    
    rachis = truss_rachis_to_branch(
        truss_dict,
        parent_trunk_id="trunk",
        rank=3,
        organ_index=0
    )
    
    print(f"\nRachis branch:")
    print(f"  ID: {rachis['id']}")
    print(f"  Parent: {rachis['parent']}")
    print(f"  Attach link: {rachis['attach_link']}")
    print(f"  N links: {rachis['n_links']}")
    print(f"  Radius: {rachis['radius']:.6f}m")
    print(f"  Height (per link): {rachis['height']:.6f}m")
    print(f"  Tilt: {rachis['tilt']:.1f}°")
    print(f"  Rot: {rachis['rot']:.1f}°")
    
    # Validate structure
    assert rachis["id"] == "Truss_r3_o0_rachis", f"Unexpected ID: {rachis['id']}"
    assert rachis["parent"] == "trunk", "Should attach to trunk"
    assert rachis["attach_link"] == 4, f"Should attach to link 4 (parent_rank + 1), got {rachis['attach_link']}"
    assert rachis["n_links"] >= 1, "Should have at least 1 link"
    assert rachis["radius"] > 0, "Radius should be positive"
    assert rachis["height"] > 0, "Height should be positive"
    assert 0 <= rachis["tilt"] <= 180, "Tilt should be in [0, 180]"
    assert 0 <= rachis["rot"] < 360, "Rotation should be in [0, 360)"
    
    print("\n✓ Rachis generation test PASSED")


def test_lateral_pedicels():
    """Test lateral pedicel generation."""
    print("\n" + "="*80)
    print("TEST: Lateral Pedicels")
    print("="*80)
    
    truss_dict = {
        "n_fruits": 7,  # 7 fruits = 3 lateral pairs + 1 terminal
        "pedicel_length": 0.008,  # 8mm
        "pedicel_angle": 90.0,
    }
    
    rachis_id = "Truss_r3_o0_rachis"
    rachis_n_links = 4
    rachis_radius = 0.001
    
    pedicels = create_lateral_pedicels(
        truss_dict,
        rachis_id,
        rachis_n_links,
        rachis_radius
    )
    
    print(f"\nGenerated {len(pedicels)} lateral pedicels:")
    
    # Should have 3 pairs = 6 pedicels
    expected_pairs = (7 - 1) // 2  # = 3
    expected_pedicels = expected_pairs * 2  # = 6
    
    assert len(pedicels) == expected_pedicels, f"Expected {expected_pedicels} pedicels, got {len(pedicels)}"
    
    # Check alternating L/R pattern
    for i in range(0, len(pedicels), 2):
        left = pedicels[i]
        right = pedicels[i + 1]
        
        print(f"  Pair {i//2}: {left['id']} (rot={left['rot']}°) | {right['id']} (rot={right['rot']}°)")
        
        assert "_L" in left["id"], f"Left pedicel should have _L suffix: {left['id']}"
        assert "_R" in right["id"], f"Right pedicel should have _R suffix: {right['id']}"
        assert left["rot"] == 90.0, f"Left should be 90°, got {left['rot']}"
        assert right["rot"] == 270.0, f"Right should be 270°, got {right['rot']}"
        assert left["parent"] == rachis_id, "Should attach to rachis"
        assert right["parent"] == rachis_id, "Should attach to rachis"
        assert left["n_links"] == 1, "Pedicel should have 1 link"
        assert right["n_links"] == 1, "Pedicel should have 1 link"
    
    print("\n✓ Lateral pedicels test PASSED")


def test_terminal_pedicel():
    """Test terminal pedicel generation."""
    print("\n" + "="*80)
    print("TEST: Terminal Pedicel")
    print("="*80)
    
    truss_dict = {
        "pedicel_length": 0.008,
    }
    
    rachis_id = "Truss_r3_o0_rachis"
    rachis_n_links = 4
    rachis_radius = 0.001
    
    terminal = create_terminal_pedicel(
        truss_dict,
        rachis_id,
        rachis_n_links,
        rachis_radius
    )
    
    print(f"\nTerminal pedicel:")
    print(f"  ID: {terminal['id']}")
    print(f"  Parent: {terminal['parent']}")
    print(f"  Attach link: {terminal['attach_link']}")
    print(f"  Tilt: {terminal['tilt']:.1f}°")
    print(f"  Rot: {terminal['rot']:.1f}°")
    
    assert terminal["id"] == f"{rachis_id}_pedicel_term", f"Unexpected ID: {terminal['id']}"
    assert terminal["parent"] == rachis_id, "Should attach to rachis"
    assert terminal["attach_link"] == rachis_n_links, f"Should attach to last link ({rachis_n_links})"
    assert terminal["tilt"] == 0.0, "Terminal should be coaxial (tilt=0)"
    assert terminal["rot"] == 0.0, "Terminal should be aligned (rot=0)"
    assert terminal["n_links"] == 1, "Pedicel should have 1 link"
    
    print("\n✓ Terminal pedicel test PASSED")


def test_default_pedicel_length_uses_truss_geometry_config():
    """Test default tomato pedicels use the configured visual/physics length."""
    truss_dict = {
        "n_fruits": 3,
    }

    pedicels = create_lateral_pedicels(
        truss_dict,
        "Truss_r3_o0_rachis",
        2,
        0.001,
    )
    terminal = create_terminal_pedicel(
        truss_dict,
        "Truss_r3_o0_rachis",
        2,
        0.001,
    )

    assert pedicels
    expected_length = _load_tree_config().TrussGeometryConfig.PEDICEL_LENGTH
    assert all(pedicel["height"] == expected_length for pedicel in pedicels)
    assert terminal["height"] == expected_length


def test_complete_truss():
    """Test complete truss branch configuration."""
    print("\n" + "="*80)
    print("TEST: Complete Truss Configuration")
    print("="*80)
    
    truss_dict = {
        "rachis_length": 0.12,
        "rachis_radius": 0.001,
        "n_fruits": 5,  # 5 fruits = 2 lateral pairs (4) + 1 terminal
        "pedicel_length": 0.008,
        "pedicel_angle": 90.0,
        "parent_rank": 2,
        "tilt_deg": 60.0,
        "azimuth_deg": 90.0,
    }
    
    branches = truss_to_branch_config(
        truss_dict,
        parent_trunk_id="trunk",
        rank=2,
        organ_index=0
    )
    
    print(f"\nGenerated {len(branches)} branches:")
    for i, branch in enumerate(branches):
        print(f"  {i+1}. {branch['id']} (parent={branch['parent']}, n_links={branch['n_links']})")
    
    # Validate structure
    # 1 rachis + 2 lateral pairs (4 pedicels) + 1 terminal = 6 branches
    expected_count = 1 + (2 * 2) + 1  # rachis + lateral pairs + terminal
    assert len(branches) == expected_count, f"Expected {expected_count} branches, got {len(branches)}"
    
    # First should be rachis
    assert "rachis" in branches[0]["id"], "First branch should be rachis"
    assert branches[0]["parent"] == "trunk", "Rachis should attach to trunk"
    
    # Last should be terminal pedicel
    assert "term" in branches[-1]["id"], "Last branch should be terminal pedicel"
    assert branches[-1]["tilt"] == 0.0, "Terminal should be coaxial"
    
    # Middle ones should be lateral pedicels
    lateral_count = len(branches) - 2  # Exclude rachis and terminal
    assert lateral_count == 4, f"Should have 4 lateral pedicels, got {lateral_count}"
    
    # Check all pedicels attach to rachis
    rachis_id = branches[0]["id"]
    for branch in branches[1:]:
        assert branch["parent"] == rachis_id, f"Pedicel {branch['id']} should attach to rachis"
    
    print("\n✓ Complete truss test PASSED")


def test_radius_clamping():
    """Test radius clamping for very thin structures."""
    print("\n" + "="*80)
    print("TEST: Radius Clamping")
    print("="*80)
    
    # Very thin truss that should trigger clamping
    truss_dict = {
        "rachis_length": 0.10,
        "rachis_radius": 0.0001,  # 0.1mm = very thin, should be clamped
        "n_fruits": 3,
        "pedicel_radius": 0.00005,  # 0.05mm = extremely thin
        "parent_rank": 1,
    }
    
    print("\nInput radii (pre-scale, world units):")
    print(f"  Rachis: {truss_dict['rachis_radius']}m (0.1mm)")
    print(f"  Pedicel: {truss_dict['pedicel_radius']}m (0.05mm)")
    print("\nWith GLOBAL_SCALE=2.0:")
    print(f"  Rachis world: {truss_dict['rachis_radius'] * 2.0}m (0.2mm)")
    print(f"  Pedicel world: {truss_dict['pedicel_radius'] * 2.0}m (0.1mm)")
    print("\nMIN_LINK_RADIUS_WORLD = 0.002m (2mm)")
    print("Expected: Both should be clamped to 0.001m pre-scale (2mm world)")
    
    branches = truss_to_branch_config(
        truss_dict,
        parent_trunk_id="trunk",
        rank=1
    )
    
    rachis = branches[0]
    print(f"\nActual rachis radius: {rachis['radius']:.6f}m pre-scale ({rachis['radius'] * 2.0:.6f}m world)")
    
    # Check clamping occurred (radius should be increased)
    assert rachis['radius'] >= 0.001, f"Rachis radius should be clamped to at least 0.001m, got {rachis['radius']}"
    
    if len(branches) > 1:
        pedicel = branches[1]
        print(f"Actual pedicel radius: {pedicel['radius']:.6f}m pre-scale ({pedicel['radius'] * 2.0:.6f}m world)")
        assert pedicel['radius'] >= 0.001, f"Pedicel radius should be clamped to at least 0.001m, got {pedicel['radius']}"
    
    print("\n✓ Radius clamping test PASSED")


def test_tomato_definitions():
    """Test tomato definition generation."""
    print("\n" + "="*80)
    print("TEST: Tomato Definitions")
    print("="*80)
    
    truss_dict = {
        "n_fruits": 5,
        "tomato_radii": [0.025, 0.03, 0.028, 0.032, 0.027],
        "maturation": [0.0, 0.5, 1.0, 0.0, 1.0],
    }
    
    pedicel_ids = [
        "Truss_r3_o0_rachis_pedicel_lat_0_L",
        "Truss_r3_o0_rachis_pedicel_lat_0_R",
        "Truss_r3_o0_rachis_pedicel_lat_1_L",
        "Truss_r3_o0_rachis_pedicel_lat_1_R",
        "Truss_r3_o0_rachis_pedicel_term",
    ]
    
    tomatoes = create_tomato_definitions(truss_dict, pedicel_ids)
    
    print(f"\nGenerated {len(tomatoes)} tomato definitions:")
    
    assert len(tomatoes) == 5, f"Expected 5 tomatoes, got {len(tomatoes)}"
    
    for i, tomato in enumerate(tomatoes):
        print(f"  {i+1}. {tomato['id']}")
        print(f"      Pedicel: {tomato['pedicel_id']}")
        print(f"      Radius: {tomato['radius']:.4f}m")
        print(f"      Mass: {tomato['mass']:.6f}kg")
        print(f"      Maturation: {tomato['maturation']:.1f} ({'ripe' if tomato['maturation'] > 0.5 else 'unripe'})")
        
        assert tomato["pedicel_id"] == pedicel_ids[i], f"Pedicel ID mismatch"
        assert tomato["radius"] == truss_dict["tomato_radii"][i], f"Radius mismatch"
        assert tomato["maturation"] == truss_dict["maturation"][i], f"Maturation mismatch"
        assert tomato["mass"] > 0, "Mass should be positive"
        assert "_tomato" in tomato["id"], "ID should contain _tomato suffix"
    
    print("\n✓ Tomato definitions test PASSED")


def test_complete_config():
    """Test complete truss configuration with tomatoes."""
    print("\n" + "="*80)
    print("TEST: Complete Configuration (Branches + Tomatoes)")
    print("="*80)
    
    truss_dict = {
        "rachis_length": 0.12,
        "rachis_radius": 0.001,
        "n_fruits": 5,
        "pedicel_length": 0.008,
        "parent_rank": 2,
        "tomato_radii": [0.025, 0.03, 0.028, 0.032, 0.027],
    }
    
    branches, tomatoes = truss_to_complete_config(
        truss_dict,
        parent_trunk_id="trunk",
        rank=2
    )
    
    print(f"\nGenerated {len(branches)} branches and {len(tomatoes)} tomatoes:")
    print(f"\nBranches:")
    for i, branch in enumerate(branches):
        print(f"  {i+1}. {branch['id']}")
    
    print(f"\nTomatoes:")
    for i, tomato in enumerate(tomatoes):
        print(f"  {i+1}. {tomato['id']} → {tomato['pedicel_id']}")
    
    # Validate structure
    assert len(branches) == 6, f"Expected 6 branches (1 rachis + 4 lateral + 1 terminal), got {len(branches)}"
    assert len(tomatoes) == 5, f"Expected 5 tomatoes, got {len(tomatoes)}"
    
    # Check each tomato has a corresponding pedicel
    pedicel_ids = [b["id"] for b in branches[1:]]  # Skip rachis
    for tomato in tomatoes:
        assert tomato["pedicel_id"] in pedicel_ids, f"Tomato {tomato['id']} has invalid pedicel reference"
    
    print("\n✓ Complete configuration test PASSED")


def test_even_fruit_count():
    """Test that even fruit counts create all lateral fruits and no terminal."""
    print("\n" + "="*80)
    print("TEST: Even Fruit Count")
    print("="*80)

    truss_dict = {
        "rachis_length": 0.12,
        "rachis_radius": 0.001,
        "n_fruits": 8,
        "pedicel_length": 0.008,
        "parent_rank": 2,
        "tomato_radii": [0.02] * 8,
    }

    branches, tomatoes = truss_to_complete_config(
        truss_dict,
        parent_trunk_id="trunk",
        rank=2
    )

    pedicels = branches[1:]
    terminal_pedicels = [b for b in pedicels if "term" in b["id"]]

    print(f"\nGenerated {len(pedicels)} pedicels and {len(tomatoes)} tomatoes")

    assert len(pedicels) == 8, f"Expected 8 pedicels, got {len(pedicels)}"
    assert len(tomatoes) == 8, f"Expected 8 tomatoes, got {len(tomatoes)}"
    assert not terminal_pedicels, "Even fruit counts should not create a terminal pedicel"

    for tomato in tomatoes:
        assert tomato["pedicel_id"] in [p["id"] for p in pedicels], "Tomato should reference a valid pedicel"

    print("\n✓ Even fruit count test PASSED")


if __name__ == "__main__":
    try:
        print("\n" + "="*80)
        print("  TRUSS BUILDER UNIT TESTS")
        print("="*80)
        
        test_rachis_generation()
        test_lateral_pedicels()
        test_terminal_pedicel()
        test_complete_truss()
        test_radius_clamping()
        test_tomato_definitions()
        test_complete_config()
        test_even_fruit_count()
        
        print("\n" + "="*80)
        print("  ALL TESTS PASSED ✓")
        print("="*80)
        print()
        
        sys.exit(0)
        
    except AssertionError as e:
        print(f"\n✗ TEST FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ UNEXPECTED ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
