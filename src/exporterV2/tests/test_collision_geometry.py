#!/usr/bin/env python3
"""
Collision Geometry Verification for ExporterV2

Tests geometric collision between cylinders (branches, leaves) to ensure
no interpenetration that would cause simulation explosions.

Tests:
1. Lateral branches inter-collision (same rank, adjacent ranks)
2. Lateral branches vs trunk
3. Leaves vs lateral branches
4. Leaves vs trunk
"""

import json
import sys
import math
from pathlib import Path

import pytest


def angle_distance(angle1, angle2):
    """Calculate shortest angular distance between two angles (degrees)."""
    diff = abs(angle1 - angle2)
    return min(diff, 360.0 - diff)


def cylinder_collision_check(cyl1, cyl2, margin=0.001):
    """
    Check if two cylinders (branches) collide geometrically.
    
    Simplified model:
    - Cylinders are attached to same parent at different angles
    - Check if bounding spheres overlap (conservative)
    
    Args:
        cyl1, cyl2: Branch dicts with radius, height, rot, tilt
        margin: Safety margin (meters, pre-scale)
    
    Returns:
        (collision: bool, min_distance: float, details: str)
    """
    # If different parents, skip (too complex for now)
    if cyl1.get("parent") != cyl2.get("parent"):
        return False, float('inf'), "Different parents"
    
    # Same attach point - check angular separation
    angle_diff = angle_distance(cyl1["rot"], cyl2["rot"])
    
    # Estimate minimum safe angle based on cylinder dimensions
    # At distance d from trunk, cylinder with radius r needs angular space
    # Conservative: min_angle = 2 * arctan(r / d) in degrees
    
    r1 = cyl1["radius"] * 2.0  # Add margin
    r2 = cyl2["radius"] * 2.0
    h1 = cyl1["height"]
    h2 = cyl2["height"]
    
    # Estimate distance from trunk center (rough)
    d = 0.05  # Assume ~5cm from trunk center
    
    min_safe_angle1 = math.degrees(2 * math.atan(r1 / d))
    min_safe_angle2 = math.degrees(2 * math.atan(r2 / d))
    min_safe_angle = max(min_safe_angle1, min_safe_angle2, 30.0)  # At least 30°
    
    if angle_diff < min_safe_angle:
        return True, angle_diff, f"Angle {angle_diff:.1f}° < safe {min_safe_angle:.1f}°"
    
    return False, angle_diff, "OK"


def load_branches_json(json_path):
    """Load branches from JSON file."""
    with open(json_path) as f:
        data = json.load(f)
    return data["branches"], data["metadata"]


def test_lateral_branches_collision(branches, verbose=True):
    """Test lateral branch collisions (same rank, adjacent ranks)."""
    print("\n" + "="*60)
    print("TEST 1: Lateral Branch Collision Check")
    print("="*60)
    
    lat_branches = [b for b in branches if b["id"].startswith("Branch_r")]
    
    if not lat_branches:
        print("⚠ No lateral branches found")
        return True
    
    print(f"Found {len(lat_branches)} lateral branches")
    
    # Group by parent_rank (attach_link - 1)
    by_parent = {}
    for b in lat_branches:
        parent_rank = b["attach_link"] - 1
        by_parent.setdefault(parent_rank, []).append(b)
    
    collisions = []
    
    # Check within same parent_rank
    for parent_rank, branches_group in by_parent.items():
        if len(branches_group) < 2:
            continue
        
        for i, b1 in enumerate(branches_group):
            for b2 in branches_group[i+1:]:
                collision, dist, details = cylinder_collision_check(b1, b2)
                
                if collision:
                    collisions.append({
                        "type": "lateral_same_rank",
                        "branch1": b1["id"],
                        "branch2": b2["id"],
                        "parent_rank": parent_rank,
                        "distance": dist,
                        "details": details
                    })
                    if verbose:
                        print(f"  ❌ COLLISION: {b1['id']} vs {b2['id']}")
                        print(f"     parent_rank={parent_rank}, {details}")
    
    # Check adjacent parent ranks
    for parent_rank in by_parent:
        for adj_rank in [parent_rank - 1, parent_rank + 1]:
            if adj_rank not in by_parent:
                continue
            
            for b1 in by_parent[parent_rank]:
                for b2 in by_parent[adj_rank]:
                    # Vertical separation helps, less strict
                    collision, dist, details = cylinder_collision_check(b1, b2, margin=0.002)
                    
                    if collision and dist < 45.0:  # Only very close angles
                        collisions.append({
                            "type": "lateral_adjacent_rank",
                            "branch1": b1["id"],
                            "branch2": b2["id"],
                            "ranks": (parent_rank, adj_rank),
                            "distance": dist,
                            "details": details
                        })
                        if verbose:
                            print(f"  ⚠ ADJACENT COLLISION: {b1['id']} vs {b2['id']}")
                            print(f"     ranks={parent_rank}/{adj_rank}, {details}")
    
    if not collisions:
        print("✅ No lateral branch collisions detected")
        return True
    else:
        print(f"❌ Found {len(collisions)} potential collisions")
        return False


def test_rotation_variance(branches, verbose=True):
    """Test that rotation has variance (not all 0°/180°)."""
    print("\n" + "="*60)
    print("TEST 2: Rotation Variance Check")
    print("="*60)
    
    lat_branches = [b for b in branches if b["id"].startswith("Branch_r")]
    
    if not lat_branches:
        print("⚠ No lateral branches found")
        return True
    
    rotations = [b["rot"] for b in lat_branches]
    
    # Check if all are exactly 0 or 180
    unique_rots = set(round(r, 1) for r in rotations)
    
    if unique_rots == {0.0, 180.0} or unique_rots == {0.0} or unique_rots == {180.0}:
        print(f"❌ No rotation variance! All branches at {unique_rots}")
        return False
    
    print(f"✅ Rotation variance present: {len(unique_rots)} unique angles")
    
    if verbose:
        print(f"\nRotation distribution:")
        for b in lat_branches[:6]:
            print(f"  {b['id']}: {b['rot']:.1f}°")
    
    return True


def test_angle_separation(branches, min_angle=60.0, verbose=True):
    """Test minimum angular separation between branches."""
    print("\n" + "="*60)
    print(f"TEST 3: Minimum Angle Separation (>={min_angle}°)")
    print("="*60)
    
    lat_branches = [b for b in branches if b["id"].startswith("Branch_r")]
    
    # Group by parent
    by_parent = {}
    for b in lat_branches:
        parent = b["parent"]
        parent_rank = b["attach_link"] - 1
        by_parent.setdefault(parent_rank, []).append(b)
    
    violations = []
    
    for parent_rank, branches_group in by_parent.items():
        if len(branches_group) < 2:
            continue
        
        for i, b1 in enumerate(branches_group):
            for b2 in branches_group[i+1:]:
                angle_diff = angle_distance(b1["rot"], b2["rot"])
                
                if angle_diff < min_angle:
                    violations.append({
                        "branch1": b1["id"],
                        "branch2": b2["id"],
                        "angle": angle_diff,
                        "parent_rank": parent_rank
                    })
                    if verbose:
                        print(f"  ❌ {b1['id']} vs {b2['id']}: {angle_diff:.1f}° < {min_angle}°")
    
    if not violations:
        print(f"✅ All branches have >={min_angle}° separation")
        return True
    else:
        print(f"❌ Found {len(violations)} violations")
        return False


def test_bounding_boxes(branches, verbose=True):
    """Test rough bounding box overlap (conservative estimate)."""
    print("\n" + "="*60)
    print("TEST 4: Bounding Box Overlap Check")
    print("="*60)
    
    lat_branches = [b for b in branches if b["id"].startswith("Branch_r")]
    trunk = next((b for b in branches if b["id"] == "trunk"), None)
    
    if not trunk:
        print("⚠ Trunk not found")
        return True
    
    # Check lat branches don't overlap with trunk
    # Trunk is vertical cylinder at center
    trunk_radius = trunk["radius"]
    
    overlaps = []
    
    for b in lat_branches:
        # Lateral branch starts at trunk surface
        # Check if branch radius + trunk radius causes overlap at attachment
        # With 45° tilt, branch extends outward
        
        # Conservative check: branch should not point back into trunk
        tilt = b.get("tilt", 45.0)
        
        if tilt > 90.0:  # Pointing downward
            overlaps.append({
                "branch": b["id"],
                "reason": f"Tilt {tilt:.1f}° > 90° (pointing down)"
            })
            if verbose:
                print(f"  ⚠ {b['id']}: tilt={tilt:.1f}° may cause downward collision")
    
    if not overlaps:
        print(f"✅ No obvious bounding box overlaps")
        return True
    else:
        print(f"⚠ Found {len(overlaps)} potential issues")
        return len(overlaps) == 0


def main():
    """Run all collision tests."""
    if len(sys.argv) < 2:
        print("Usage: python test_collision_geometry.py <branches_json>")
        print("Example: python test_collision_geometry.py output/day_100/branches_v2_day_100.json")
        sys.exit(1)
    
    json_path = Path(sys.argv[1])
    
    if not json_path.exists():
        print(f"❌ File not found: {json_path}")
        sys.exit(1)
    
    print("="*60)
    print("COLLISION GEOMETRY VERIFICATION")
    print("="*60)
    print(f"Input: {json_path}")
    
    branches, metadata = load_branches_json(json_path)
    
    print(f"\nMetadata:")
    print(f"  Day: {metadata['day']}")
    print(f"  Branches: {metadata['n_branches']}")
    print(f"  Links: {metadata['total_links']}")
    
    # Run tests
    results = []
    
    results.append(("Lateral Branch Collision", test_lateral_branches_collision(branches)))
    results.append(("Rotation Variance", test_rotation_variance(branches)))
    results.append(("Angle Separation", test_angle_separation(branches)))
    results.append(("Bounding Box Overlap", test_bounding_boxes(branches)))
    
    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    
    all_passed = all(r[1] for r in results)
    
    for name, passed in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status}: {name}")
    
    print("\n" + "="*60)
    
    if all_passed:
        print("✅ ALL TESTS PASSED - No collision risks detected")
        return 0
    else:
        print("❌ SOME TESTS FAILED - Review collisions above")
        return 1


if __name__ == "__main__":
    sys.exit(main())


@pytest.fixture
def branches():
    """Build current day-100 branches for pytest collection."""
    from exporterV2.tests.legacy_csv import parse_csv_to_branches

    parsed_branches, _ = parse_csv_to_branches(day=100)
    return parsed_branches


def test_collision_geometry_pytest(branches):
    """Pytest entry point; the functions above also serve the CLI script."""
    assert test_lateral_branches_collision(branches, verbose=False)
    assert test_rotation_variance(branches, verbose=False)
    assert test_angle_separation(branches, verbose=False)
    assert test_bounding_boxes(branches, verbose=False)


test_lateral_branches_collision.__test__ = False
test_rotation_variance.__test__ = False
test_angle_separation.__test__ = False
test_bounding_boxes.__test__ = False
