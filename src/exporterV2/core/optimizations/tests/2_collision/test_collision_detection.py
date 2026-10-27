"""
test_collision_detection.py - Tests for Collision Detection System

Tests for sphere overlap, AABB overlap, and broad-phase orchestration.

Run with: uv run python src/exporterV2/core/optimizations/tests/test_collision_detection.py
"""

import sys
import os
import math

# Add optimizations directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
optimizations_dir = os.path.join(script_dir, "../..")
sys.path.insert(0, optimizations_dir)

from collision.sphere import Vec3, CylinderGeometry, calculate_bounding_sphere, check_sphere_overlap
from collision.aabb import calculate_aabb, check_aabb_overlap
from collision.broad_phase import check_attachment_collision, check_pairwise_collisions


def test_vec3_operations():
    """Test Vec3 basic operations."""
    print("\n[TEST] Vec3 Operations...")
    
    v1 = Vec3(1, 2, 3)
    v2 = Vec3(4, 5, 6)
    
    # Addition
    v3 = v1 + v2
    assert v3.x == 5 and v3.y == 7 and v3.z == 9, "Addition failed"
    
    # Subtraction
    v4 = v2 - v1
    assert v4.x == 3 and v4.y == 3 and v4.z == 3, "Subtraction failed"
    
    # Scalar multiplication
    v5 = v1 * 2
    assert v5.x == 2 and v5.y == 4 and v5.z == 6, "Scalar mult failed"
    
    # Length
    v6 = Vec3(3, 4, 0)
    assert abs(v6.length() - 5.0) < 1e-6, "Length calculation failed"
    
    print("  ✓ Vec3 operations correct")


def test_sphere_bounding():
    """Test bounding sphere calculation."""
    print("\n[TEST] Bounding Sphere Calculation...")
    
    # Vertical cylinder
    link = CylinderGeometry(
        base=Vec3(0, 0, 0),
        axis=Vec3(0, 0, 1),
        height=1.0,
        radius=0.1
    )
    
    center, radius = calculate_bounding_sphere(link)
    
    # Center should be at midpoint
    assert abs(center.x) < 1e-6, "Center X should be 0"
    assert abs(center.y) < 1e-6, "Center Y should be 0"
    assert abs(center.z - 0.5) < 1e-6, "Center Z should be 0.5"
    
    # Radius should be sqrt((height/2)^2 + radius^2)
    expected_radius = math.sqrt(0.5**2 + 0.1**2)
    assert abs(radius - expected_radius) < 1e-6, f"Radius mismatch: {radius} vs {expected_radius}"
    
    print(f"  ✓ Sphere: center={center}, radius={radius:.3f}")


def test_sphere_overlap_touching():
    """Test sphere overlap for touching spheres."""
    print("\n[TEST] Sphere Overlap - Touching...")
    
    # Two spheres touching at x=1
    s1 = (Vec3(0, 0, 0), 0.5)  # Center at origin, radius 0.5
    s2 = (Vec3(1, 0, 0), 0.5)  # Center at x=1, radius 0.5
    
    # Should be touching (distance = 1.0, sum of radii = 1.0)
    overlap = check_sphere_overlap(s1, s2)
    assert overlap, "Touching spheres should overlap"
    
    print("  ✓ Touching spheres detected")


def test_sphere_overlap_separated():
    """Test sphere overlap for separated spheres."""
    print("\n[TEST] Sphere Overlap - Separated...")
    
    s1 = (Vec3(0, 0, 0), 0.5)
    s2 = (Vec3(2, 0, 0), 0.5)  # Center at x=2, clearly separated
    
    overlap = check_sphere_overlap(s1, s2)
    assert not overlap, "Separated spheres should not overlap"
    
    print("  ✓ Separated spheres correctly rejected")


def test_sphere_overlap_with_margin():
    """Test sphere overlap with safety margin."""
    print("\n[TEST] Sphere Overlap - With Margin...")
    
    # Two spheres just touching
    s1 = (Vec3(0, 0, 0), 0.5)
    s2 = (Vec3(1, 0, 0), 0.5)
    
    # Without margin: touching (overlap)
    assert check_sphere_overlap(s1, s2, margin=0.0)
    
    # With margin: too close (overlap detected)
    assert check_sphere_overlap(s1, s2, margin=0.1)
    
    print("  ✓ Safety margin working correctly")


def test_aabb_vertical_cylinder():
    """Test AABB calculation for vertical cylinder."""
    print("\n[TEST] AABB - Vertical Cylinder...")
    
    link = CylinderGeometry(
        base=Vec3(0, 0, 0),
        axis=Vec3(0, 0, 1),
        height=1.0,
        radius=0.1
    )
    
    min_pt, max_pt = calculate_aabb(link)
    
    # For vertical cylinder: AABB should be symmetric around Z axis
    assert abs(min_pt.x + 0.1) < 0.01, f"Min X should be ~-0.1, got {min_pt.x}"
    assert abs(max_pt.x - 0.1) < 0.01, f"Max X should be ~0.1, got {max_pt.x}"
    assert abs(min_pt.y + 0.1) < 0.01, f"Min Y should be ~-0.1, got {min_pt.y}"
    assert abs(max_pt.y - 0.1) < 0.01, f"Max Y should be ~0.1, got {max_pt.y}"
    assert abs(min_pt.z) < 0.01, f"Min Z should be ~0, got {min_pt.z}"
    assert abs(max_pt.z - 1.0) < 0.01, f"Max Z should be ~1.0, got {max_pt.z}"
    
    print(f"  ✓ AABB: min={min_pt}, max={max_pt}")


def test_aabb_overlap_overlapping():
    """Test AABB overlap for overlapping boxes."""
    print("\n[TEST] AABB Overlap - Overlapping...")
    
    aabb1 = (Vec3(0, 0, 0), Vec3(1, 1, 1))
    aabb2 = (Vec3(0.5, 0.5, 0.5), Vec3(1.5, 1.5, 1.5))
    
    overlap = check_aabb_overlap(aabb1, aabb2)
    assert overlap, "Overlapping AABBs should be detected"
    
    print("  ✓ Overlapping AABBs detected")


def test_aabb_overlap_separated():
    """Test AABB overlap for separated boxes."""
    print("\n[TEST] AABB Overlap - Separated...")
    
    aabb1 = (Vec3(0, 0, 0), Vec3(1, 1, 1))
    aabb2 = (Vec3(2, 0, 0), Vec3(3, 1, 1))  # Separated on X axis
    
    overlap = check_aabb_overlap(aabb1, aabb2)
    assert not overlap, "Separated AABBs should not overlap"
    
    print("  ✓ Separated AABBs correctly rejected")


def test_aabb_overlap_touching():
    """Test AABB overlap for touching boxes."""
    print("\n[TEST] AABB Overlap - Touching...")
    
    aabb1 = (Vec3(0, 0, 0), Vec3(1, 1, 1))
    aabb2 = (Vec3(1, 0, 0), Vec3(2, 1, 1))  # Touching at x=1
    
    overlap = check_aabb_overlap(aabb1, aabb2)
    # Touching counts as overlap (intervals overlap at boundary)
    assert overlap, "Touching AABBs should overlap"
    
    print("  ✓ Touching AABBs detected as overlap")


def test_broad_phase_no_collision():
    """Test broad-phase with no collision."""
    print("\n[TEST] Broad-Phase - No Collision...")
    
    # New link
    new_link = CylinderGeometry(
        base=Vec3(0, 0, 0),
        axis=Vec3(0, 0, 1),
        height=1.0,
        radius=0.1
    )
    
    # Sibling far away
    siblings = [
        ("sibling1", CylinderGeometry(
            base=Vec3(5, 0, 0),
            axis=Vec3(0, 0, 1),
            height=1.0,
            radius=0.1
        ))
    ]
    
    # Parent (also far)
    parent = CylinderGeometry(
        base=Vec3(0, 5, 0),
        axis=Vec3(0, 0, 1),
        height=1.0,
        radius=0.1
    )
    
    result = check_attachment_collision(new_link, siblings, parent)
    
    assert not result.collision_detected, "Should not detect collision"
    assert result.stage_detected == "none", "Should be rejected at sphere stage"
    
    print(f"  ✓ No collision detected (stage: {result.stage_detected})")


def test_broad_phase_with_collision():
    """Test broad-phase with collision."""
    print("\n[TEST] Broad-Phase - With Collision...")
    
    # New link
    new_link = CylinderGeometry(
        base=Vec3(0, 0, 0),
        axis=Vec3(0, 0, 1),
        height=1.0,
        radius=0.1
    )
    
    # Sibling overlapping
    siblings = [
        ("sibling1", CylinderGeometry(
            base=Vec3(0.1, 0, 0),  # Very close, will overlap
            axis=Vec3(0, 0, 1),
            height=1.0,
            radius=0.1
        ))
    ]
    
    parent = CylinderGeometry(
        base=Vec3(0, 5, 0),  # Far away
        axis=Vec3(0, 0, 1),
        height=1.0,
        radius=0.1
    )
    
    result = check_attachment_collision(new_link, siblings, parent, check_parent=False)
    
    assert result.collision_detected, "Should detect collision"
    assert "sibling1" in result.colliding_with, "Should report sibling1"
    assert result.stage_detected == "aabb", "Should be detected at AABB stage"
    
    print(f"  ✓ Collision detected with: {result.colliding_with}")


def test_pairwise_collisions():
    """Test pairwise collision checking."""
    print("\n[TEST] Pairwise Collisions...")
    
    # Three links: two overlapping, one separate
    links = [
        ("link1", CylinderGeometry(
            base=Vec3(0, 0, 0),
            axis=Vec3(0, 0, 1),
            height=1.0,
            radius=0.1
        )),
        ("link2", CylinderGeometry(
            base=Vec3(0.1, 0, 0),  # Overlaps with link1
            axis=Vec3(0, 0, 1),
            height=1.0,
            radius=0.1
        )),
        ("link3", CylinderGeometry(
            base=Vec3(5, 0, 0),  # Far from others
            axis=Vec3(0, 0, 1),
            height=1.0,
            radius=0.1
        ))
    ]
    
    collisions = check_pairwise_collisions(links)
    
    assert len(collisions) == 1, f"Should find 1 collision, found {len(collisions)}"
    assert ("link1", "link2") in collisions or ("link2", "link1") in collisions, \
        "Should detect link1-link2 collision"
    
    print(f"  ✓ Found {len(collisions)} collision(s): {collisions}")


def main():
    """Run all collision detection tests."""
    print("=" * 70)
    print("  Collision Detection - Test Suite")
    print("=" * 70)
    
    tests = [
        test_vec3_operations,
        test_sphere_bounding,
        test_sphere_overlap_touching,
        test_sphere_overlap_separated,
        test_sphere_overlap_with_margin,
        test_aabb_vertical_cylinder,
        test_aabb_overlap_overlapping,
        test_aabb_overlap_separated,
        test_aabb_overlap_touching,
        test_broad_phase_no_collision,
        test_broad_phase_with_collision,
        test_pairwise_collisions,
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
    
    print("\n" + "=" * 70)
    print(f"  Test Results: {passed} passed, {failed} failed")
    print("=" * 70)
    
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
