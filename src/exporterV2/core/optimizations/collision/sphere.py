"""
sphere.py - Bounding Sphere Collision Detection (Stage 1)

Fast pre-check using bounding spheres for collision detection.
This is the first stage of the two-stage broad-phase collision system.

Algorithm:
    1. Calculate bounding sphere for each cylindrical link
    2. Check if distance between sphere centers < sum of radii + margin
    3. If no overlap → safe, if overlap → pass to Stage 2 (AABB)

Complexity: O(1) per pair check
"""

import math
from typing import Tuple
from dataclasses import dataclass


@dataclass
class Vec3:
    """Simple 3D vector for geometry calculations."""
    x: float
    y: float
    z: float
    
    def __add__(self, other: 'Vec3') -> 'Vec3':
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other: 'Vec3') -> 'Vec3':
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, scalar: float) -> 'Vec3':
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)
    
    def length(self) -> float:
        """Calculate vector length (magnitude)."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)
    
    def normalized(self) -> 'Vec3':
        """Return normalized (unit) vector."""
        length = self.length()
        if length < 1e-10:
            return Vec3(0, 0, 0)
        return Vec3(self.x / length, self.y / length, self.z / length)


@dataclass
class CylinderGeometry:
    """
    Geometric representation of a cylindrical link.
    
    Attributes:
        base: Base position (world coordinates)
        axis: Unit vector along cylinder axis
        height: Cylinder height
        radius: Cylinder radius
    """
    base: Vec3
    axis: Vec3
    height: float
    radius: float


def calculate_bounding_sphere(link: CylinderGeometry) -> Tuple[Vec3, float]:
    """
    Calculate bounding sphere for a cylindrical link.
    
    The bounding sphere is centered at the cylinder's midpoint and has
    radius sufficient to enclose the entire cylinder (including caps).
    
    Args:
        link: Cylinder geometry
    
    Returns:
        Tuple (center, radius):
            center: Sphere center position (midpoint of cylinder)
            radius: Sphere radius (distance from center to cylinder corner)
    
    Algorithm:
        - Center = base + 0.5 * height * axis
        - Radius = sqrt((height/2)^2 + radius^2)
        
        This ensures the sphere fully contains the cylinder including
        the circular caps at both ends.
    
    Example:
        >>> link = CylinderGeometry(
        ...     base=Vec3(0, 0, 0),
        ...     axis=Vec3(0, 0, 1),
        ...     height=1.0,
        ...     radius=0.1
        ... )
        >>> center, radius = calculate_bounding_sphere(link)
        >>> print(f"Center: {center}, Radius: {radius:.3f}")
        Center: Vec3(0, 0, 0.5), Radius: 0.510
    """
    # Calculate sphere center (midpoint of cylinder)
    center = link.base + link.axis * (link.height / 2.0)
    
    # Calculate sphere radius (distance from center to corner)
    # This is the hypotenuse of right triangle with legs: height/2 and radius
    sphere_radius = math.sqrt((link.height / 2.0)**2 + link.radius**2)
    
    return (center, sphere_radius)


def check_sphere_overlap(
    sphere1: Tuple[Vec3, float],
    sphere2: Tuple[Vec3, float],
    margin: float = 0.0
) -> bool:
    """
    Check if two bounding spheres overlap.
    
    Two spheres overlap if the distance between their centers is less
    than the sum of their radii (plus optional safety margin).
    
    Args:
        sphere1: First sphere (center, radius)
        sphere2: Second sphere (center, radius)
        margin: Safety margin in meters (default: 0.0)
    
    Returns:
        True if overlap detected, False otherwise
    
    Algorithm:
        distance = |center2 - center1|
        overlap = distance < (radius1 + radius2 + margin)
    
    Example:
        >>> s1 = (Vec3(0, 0, 0), 0.5)  # Center at origin, radius 0.5
        >>> s2 = (Vec3(1, 0, 0), 0.5)  # Center at x=1, radius 0.5
        >>> check_sphere_overlap(s1, s2)  # Touching
        True
        >>> s3 = (Vec3(2, 0, 0), 0.5)  # Center at x=2, radius 0.5
        >>> check_sphere_overlap(s1, s3)  # Separated
        False
    """
    center1, radius1 = sphere1
    center2, radius2 = sphere2
    
    # Calculate distance between centers
    distance = (center2 - center1).length()
    
    # Check if distance is less than or equal to sum of radii (plus margin)
    threshold = radius1 + radius2 + margin
    
    return distance <= threshold


def check_sphere_overlap_detailed(
    sphere1: Tuple[Vec3, float],
    sphere2: Tuple[Vec3, float],
    margin: float = 0.0
) -> Tuple[bool, float, float]:
    """
    Check sphere overlap with detailed information.
    
    Like check_sphere_overlap() but returns additional debug info.
    
    Args:
        sphere1: First sphere (center, radius)
        sphere2: Second sphere (center, radius)
        margin: Safety margin in meters
    
    Returns:
        Tuple (overlap, distance, threshold):
            overlap: True if overlap detected
            distance: Actual distance between centers
            threshold: Overlap threshold (r1 + r2 + margin)
    
    Useful for debugging and collision resolution strategies.
    """
    center1, radius1 = sphere1
    center2, radius2 = sphere2
    
    distance = (center2 - center1).length()
    threshold = radius1 + radius2 + margin
    overlap = distance <= threshold
    
    return (overlap, distance, threshold)
