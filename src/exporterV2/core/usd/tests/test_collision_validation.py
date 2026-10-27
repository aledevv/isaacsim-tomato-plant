"""
test_collision_validation.py - Unit tests for geometry validation

Tests collision detection functions for pre-simulation validation.

Run with:
    python test_collision_validation.py
"""

import sys
from pathlib import Path

# Add parent directories to path
script_dir = Path(__file__).parent
core_dir = script_dir.parent
sys.path.insert(0, str(core_dir))

# Import only math functions (no pxr dependencies)
import math


def check_sphere_sphere_intersection(pos_a, radius_a, pos_b, radius_b, margin=0.0):
    """Local copy for testing without pxr."""
    dx = pos_b[0] - pos_a[0]
    dy = pos_b[1] - pos_a[1]
    dz = pos_b[2] - pos_a[2]
    
    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    min_distance = radius_a + radius_b + margin
    
    intersects = distance <= min_distance
    overlap = min_distance - distance
    
    return intersects, distance, overlap


def check_sphere_cylinder_intersection(sphere_pos, sphere_radius, cyl_base, cyl_axis, cyl_height, cyl_radius, margin=0.0):
    """Local copy for testing without pxr."""
    dx = sphere_pos[0] - cyl_base[0]
    dy = sphere_pos[1] - cyl_base[1]
    dz = sphere_pos[2] - cyl_base[2]
    
    dot = dx * cyl_axis[0] + dy * cyl_axis[1] + dz * cyl_axis[2]
    t = max(0.0, min(cyl_height, dot))
    
    closest_x = cyl_base[0] + cyl_axis[0] * t
    closest_y = cyl_base[1] + cyl_axis[1] * t
    closest_z = cyl_base[2] + cyl_axis[2] * t
    
    dx_closest = sphere_pos[0] - closest_x
    dy_closest = sphere_pos[1] - closest_y
    dz_closest = sphere_pos[2] - closest_z
    
    radial_distance = math.sqrt(dx_closest**2 + dy_closest**2 + dz_closest**2)
    min_distance = sphere_radius + cyl_radius + margin
    
    intersects = radial_distance <= min_distance
    distance = radial_distance - cyl_radius
    overlap = min_distance - radial_distance
    
    return intersects, distance, overlap


def test_sphere_sphere_no_intersection():
    """Test sphere-sphere with no intersection."""
    print("\n" + "="*80)
    print("TEST: Sphere-Sphere No Intersection")
    print("="*80)
    
    # Two spheres 1m apart, radii 0.2m each
    pos_a = (0.0, 0.0, 0.0)
    pos_b = (1.0, 0.0, 0.0)
    radius_a = 0.2
    radius_b = 0.2
    
    intersects, distance, overlap = check_sphere_sphere_intersection(
        pos_a, radius_a, pos_b, radius_b
    )
    
    print(f"\nSphere A: pos={pos_a}, radius={radius_a}m")
    print(f"Sphere B: pos={pos_b}, radius={radius_b}m")
    print(f"Distance: {distance:.3f}m")
    print(f"Min distance: {radius_a + radius_b:.3f}m")
    print(f"Overlap: {overlap:.3f}m")
    print(f"Intersects: {intersects}")
    
    assert not intersects, "Should not intersect"
    assert distance == 1.0, f"Distance should be 1.0m, got {distance}"
    assert overlap < 0, "Overlap should be negative (separated)"
    
    print("\n✓ No intersection test PASSED")


def test_sphere_sphere_touching():
    """Test sphere-sphere just touching."""
    print("\n" + "="*80)
    print("TEST: Sphere-Sphere Touching")
    print("="*80)
    
    # Two spheres touching, radii 0.3m each, 0.6m apart
    pos_a = (0.0, 0.0, 0.0)
    pos_b = (0.6, 0.0, 0.0)
    radius_a = 0.3
    radius_b = 0.3
    
    intersects, distance, overlap = check_sphere_sphere_intersection(
        pos_a, radius_a, pos_b, radius_b
    )
    
    print(f"\nSphere A: pos={pos_a}, radius={radius_a}m")
    print(f"Sphere B: pos={pos_b}, radius={radius_b}m")
    print(f"Distance: {distance:.3f}m")
    print(f"Overlap: {overlap:.6f}m")
    print(f"Intersects: {intersects}")
    
    assert not intersects or abs(overlap) < 1e-6, "Should be just touching (no intersection)"
    
    print("\n✓ Touching test PASSED")


def test_sphere_sphere_overlapping():
    """Test sphere-sphere with overlap."""
    print("\n" + "="*80)
    print("TEST: Sphere-Sphere Overlapping")
    print("="*80)
    
    # Two spheres overlapping, radii 0.3m each, 0.4m apart
    pos_a = (0.0, 0.0, 0.0)
    pos_b = (0.4, 0.0, 0.0)
    radius_a = 0.3
    radius_b = 0.3
    
    intersects, distance, overlap = check_sphere_sphere_intersection(
        pos_a, radius_a, pos_b, radius_b
    )
    
    print(f"\nSphere A: pos={pos_a}, radius={radius_a}m")
    print(f"Sphere B: pos={pos_b}, radius={radius_b}m")
    print(f"Distance: {distance:.3f}m")
    print(f"Expected min distance: {radius_a + radius_b:.3f}m")
    print(f"Overlap: {overlap:.3f}m")
    print(f"Intersects: {intersects}")
    
    assert intersects, "Should intersect"
    assert overlap > 0, "Overlap should be positive"
    assert abs(overlap - 0.2) < 1e-6, f"Overlap should be 0.2m, got {overlap}"
    
    print("\n✓ Overlapping test PASSED")


def test_sphere_sphere_with_margin():
    """Test sphere-sphere with safety margin."""
    print("\n" + "="*80)
    print("TEST: Sphere-Sphere with Margin")
    print("="*80)
    
    # Two spheres 0.5m apart, radii 0.2m each
    # Without margin: no intersection
    # With 0.1m margin: should detect intersection
    pos_a = (0.0, 0.0, 0.0)
    pos_b = (0.5, 0.0, 0.0)
    radius_a = 0.2
    radius_b = 0.2
    margin = 0.1
    
    intersects_no_margin, _, _ = check_sphere_sphere_intersection(
        pos_a, radius_a, pos_b, radius_b, margin=0.0
    )
    
    intersects_with_margin, distance, overlap = check_sphere_sphere_intersection(
        pos_a, radius_a, pos_b, radius_b, margin=margin
    )
    
    print(f"\nSphere A: pos={pos_a}, radius={radius_a}m")
    print(f"Sphere B: pos={pos_b}, radius={radius_b}m")
    print(f"Distance: {distance:.3f}m")
    print(f"No margin: intersects={intersects_no_margin}")
    print(f"With {margin}m margin: intersects={intersects_with_margin}")
    
    assert not intersects_no_margin, "Should not intersect without margin"
    assert intersects_with_margin, "Should intersect with margin"
    
    print("\n✓ Margin test PASSED")


def test_sphere_cylinder_no_intersection():
    """Test sphere-cylinder with no intersection."""
    print("\n" + "="*80)
    print("TEST: Sphere-Cylinder No Intersection")
    print("="*80)
    
    # Vertical cylinder at origin, sphere offset
    sphere_pos = (1.0, 0.0, 0.5)
    sphere_radius = 0.1
    cyl_base = (0.0, 0.0, 0.0)
    cyl_axis = (0.0, 0.0, 1.0)
    cyl_height = 1.0
    cyl_radius = 0.1
    
    intersects, distance, overlap = check_sphere_cylinder_intersection(
        sphere_pos, sphere_radius,
        cyl_base, cyl_axis, cyl_height, cyl_radius
    )
    
    print(f"\nSphere: pos={sphere_pos}, radius={sphere_radius}m")
    print(f"Cylinder: base={cyl_base}, axis={cyl_axis}, h={cyl_height}m, r={cyl_radius}m")
    print(f"Distance: {distance:.3f}m")
    print(f"Intersects: {intersects}")
    
    assert not intersects, "Should not intersect"
    
    print("\n✓ No intersection test PASSED")


def test_sphere_cylinder_intersecting():
    """Test sphere-cylinder with intersection."""
    print("\n" + "="*80)
    print("TEST: Sphere-Cylinder Intersecting")
    print("="*80)
    
    # Vertical cylinder at origin, sphere touching side
    sphere_pos = (0.15, 0.0, 0.5)
    sphere_radius = 0.1
    cyl_base = (0.0, 0.0, 0.0)
    cyl_axis = (0.0, 0.0, 1.0)
    cyl_height = 1.0
    cyl_radius = 0.1
    
    intersects, distance, overlap = check_sphere_cylinder_intersection(
        sphere_pos, sphere_radius,
        cyl_base, cyl_axis, cyl_height, cyl_radius
    )
    
    print(f"\nSphere: pos={sphere_pos}, radius={sphere_radius}m")
    print(f"Cylinder: base={cyl_base}, axis={cyl_axis}, h={cyl_height}m, r={cyl_radius}m")
    print(f"Distance: {distance:.3f}m")
    print(f"Overlap: {overlap:.3f}m")
    print(f"Intersects: {intersects}")
    
    assert intersects, "Should intersect"
    assert overlap > 0, "Overlap should be positive"
    
    print("\n✓ Intersecting test PASSED")


if __name__ == "__main__":
    try:
        print("\n" + "="*80)
        print("  COLLISION VALIDATION UNIT TESTS")
        print("="*80)
        
        test_sphere_sphere_no_intersection()
        test_sphere_sphere_touching()
        test_sphere_sphere_overlapping()
        test_sphere_sphere_with_margin()
        test_sphere_cylinder_no_intersection()
        test_sphere_cylinder_intersecting()
        
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
