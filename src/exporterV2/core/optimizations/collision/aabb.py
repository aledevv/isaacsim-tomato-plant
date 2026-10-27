"""
aabb.py - Axis-Aligned Bounding Box Collision Detection (Stage 2)

Precision collision check using AABBs (Axis-Aligned Bounding Boxes).
This is the second stage of the two-stage broad-phase collision system.

Algorithm:
    1. Calculate AABB for oriented cylinder by sampling corner points
    2. Check overlap on all 3 axes (X, Y, Z)
    3. Overlap on all axes → collision detected

Complexity: O(1) per pair check (with fixed sampling)
"""

import math
from typing import Tuple, List
from .sphere import Vec3, CylinderGeometry


def calculate_aabb(link: CylinderGeometry) -> Tuple[Vec3, Vec3]:
    """
    Calculate Axis-Aligned Bounding Box for an oriented cylinder.
    
    Since the cylinder can be rotated, we sample points around the
    cylinder (caps + sides) and compute the min/max extents.
    
    Args:
        link: Cylinder geometry
    
    Returns:
        Tuple (min_point, max_point):
            min_point: Minimum corner (x_min, y_min, z_min)
            max_point: Maximum corner (x_max, y_max, z_max)
    
    Algorithm:
        1. Sample 16 points around cylinder:
           - 8 points on bottom cap (at different angles)
           - 8 points on top cap (at different angles)
        2. Compute min/max for each axis
    
    Example:
        >>> link = CylinderGeometry(
        ...     base=Vec3(0, 0, 0),
        ...     axis=Vec3(0, 0, 1),
        ...     height=1.0,
        ...     radius=0.1
        ... )
        >>> min_pt, max_pt = calculate_aabb(link)
        >>> print(f"Min: {min_pt}, Max: {max_pt}")
        Min: Vec3(-0.1, -0.1, 0), Max: Vec3(0.1, 0.1, 1.0)
    """
    # Sample points around the cylinder
    sample_points = []
    
    # Number of angular samples around cylinder
    n_samples = 8
    
    # Calculate two perpendicular vectors to cylinder axis
    # These define the plane perpendicular to the axis
    axis_norm = link.axis.normalized()
    
    # Find an arbitrary perpendicular vector
    if abs(axis_norm.z) < 0.9:
        perp1 = Vec3(0, 0, 1)
    else:
        perp1 = Vec3(1, 0, 0)
    
    # Cross product to get first perpendicular
    perp1 = Vec3(
        perp1.y * axis_norm.z - perp1.z * axis_norm.y,
        perp1.z * axis_norm.x - perp1.x * axis_norm.z,
        perp1.x * axis_norm.y - perp1.y * axis_norm.x
    ).normalized()
    
    # Cross product to get second perpendicular
    perp2 = Vec3(
        axis_norm.y * perp1.z - axis_norm.z * perp1.y,
        axis_norm.z * perp1.x - axis_norm.x * perp1.z,
        axis_norm.x * perp1.y - axis_norm.y * perp1.x
    ).normalized()
    
    # Sample points on bottom cap
    for i in range(n_samples):
        angle = 2.0 * math.pi * i / n_samples
        offset = perp1 * (link.radius * math.cos(angle)) + \
                 perp2 * (link.radius * math.sin(angle))
        point = link.base + offset
        sample_points.append(point)
    
    # Sample points on top cap
    top_center = link.base + axis_norm * link.height
    for i in range(n_samples):
        angle = 2.0 * math.pi * i / n_samples
        offset = perp1 * (link.radius * math.cos(angle)) + \
                 perp2 * (link.radius * math.sin(angle))
        point = top_center + offset
        sample_points.append(point)
    
    # Also add center points for better coverage
    sample_points.append(link.base)
    sample_points.append(top_center)
    
    # Compute min/max for each axis
    x_coords = [p.x for p in sample_points]
    y_coords = [p.y for p in sample_points]
    z_coords = [p.z for p in sample_points]
    
    min_point = Vec3(min(x_coords), min(y_coords), min(z_coords))
    max_point = Vec3(max(x_coords), max(y_coords), max(z_coords))
    
    return (min_point, max_point)


def check_aabb_overlap(
    aabb1: Tuple[Vec3, Vec3],
    aabb2: Tuple[Vec3, Vec3]
) -> bool:
    """
    Check if two AABBs overlap.
    
    Two AABBs overlap if and only if their intervals overlap on ALL three axes.
    
    Args:
        aabb1: First AABB (min_point, max_point)
        aabb2: Second AABB (min_point, max_point)
    
    Returns:
        True if overlap detected, False otherwise
    
    Algorithm:
        For each axis (x, y, z):
            intervals_overlap = (min1 <= max2) AND (max1 >= min2)
        
        total_overlap = overlap_x AND overlap_y AND overlap_z
    
    Example:
        >>> aabb1 = (Vec3(0, 0, 0), Vec3(1, 1, 1))
        >>> aabb2 = (Vec3(0.5, 0.5, 0.5), Vec3(1.5, 1.5, 1.5))
        >>> check_aabb_overlap(aabb1, aabb2)  # Overlapping
        True
        >>> aabb3 = (Vec3(2, 0, 0), Vec3(3, 1, 1))
        >>> check_aabb_overlap(aabb1, aabb3)  # Separated on X
        False
    """
    min1, max1 = aabb1
    min2, max2 = aabb2
    
    # Check overlap on X axis
    overlap_x = (min1.x <= max2.x) and (max1.x >= min2.x)
    
    # Check overlap on Y axis
    overlap_y = (min1.y <= max2.y) and (max1.y >= min2.y)
    
    # Check overlap on Z axis
    overlap_z = (min1.z <= max2.z) and (max1.z >= min2.z)
    
    # Overlap only if all axes overlap
    return overlap_x and overlap_y and overlap_z


def check_aabb_overlap_detailed(
    aabb1: Tuple[Vec3, Vec3],
    aabb2: Tuple[Vec3, Vec3]
) -> Tuple[bool, bool, bool, bool]:
    """
    Check AABB overlap with per-axis details.
    
    Like check_aabb_overlap() but returns detailed axis-by-axis info.
    
    Args:
        aabb1: First AABB (min_point, max_point)
        aabb2: Second AABB (min_point, max_point)
    
    Returns:
        Tuple (total_overlap, overlap_x, overlap_y, overlap_z):
            total_overlap: True if overlap on all axes
            overlap_x: True if overlap on X axis
            overlap_y: True if overlap on Y axis
            overlap_z: True if overlap on Z axis
    
    Useful for debugging which axis is causing separation.
    
    Example:
        >>> aabb1 = (Vec3(0, 0, 0), Vec3(1, 1, 1))
        >>> aabb2 = (Vec3(2, 0, 0), Vec3(3, 1, 1))  # Separated on X
        >>> total, x, y, z = check_aabb_overlap_detailed(aabb1, aabb2)
        >>> print(f"X: {x}, Y: {y}, Z: {z}, Total: {total}")
        X: False, Y: True, Z: True, Total: False
    """
    min1, max1 = aabb1
    min2, max2 = aabb2
    
    overlap_x = (min1.x <= max2.x) and (max1.x >= min2.x)
    overlap_y = (min1.y <= max2.y) and (max1.y >= min2.y)
    overlap_z = (min1.z <= max2.z) and (max1.z >= min2.z)
    
    total_overlap = overlap_x and overlap_y and overlap_z
    
    return (total_overlap, overlap_x, overlap_y, overlap_z)


def get_aabb_volume(aabb: Tuple[Vec3, Vec3]) -> float:
    """
    Calculate volume of AABB.
    
    Volume = (max_x - min_x) * (max_y - min_y) * (max_z - min_z)
    
    Args:
        aabb: AABB (min_point, max_point)
    
    Returns:
        Volume in cubic units
    """
    min_pt, max_pt = aabb
    
    width = max_pt.x - min_pt.x
    height = max_pt.y - min_pt.y
    depth = max_pt.z - min_pt.z
    
    return width * height * depth


def get_aabb_center(aabb: Tuple[Vec3, Vec3]) -> Vec3:
    """
    Calculate center point of AABB.
    
    Center = (min + max) / 2
    
    Args:
        aabb: AABB (min_point, max_point)
    
    Returns:
        Center position
    """
    min_pt, max_pt = aabb
    
    return Vec3(
        (min_pt.x + max_pt.x) / 2.0,
        (min_pt.y + max_pt.y) / 2.0,
        (min_pt.z + max_pt.z) / 2.0
    )
