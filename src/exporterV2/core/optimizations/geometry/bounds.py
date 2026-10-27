"""
bounds.py - Bounding Volume Calculation from Branch Configs

Converts branch configurations to CylinderGeometry for collision detection.
Handles coordinate transformations and link positioning.

Usage:
    from collision import CylinderGeometry
    from geometry.bounds import link_to_cylinder_geometry
    
    branch = {"id": "trunk", "n_links": 5, "height": 0.2, "radius": 0.05}
    cylinder = link_to_cylinder_geometry(branch, link_idx=2, base_z=0.4)
"""

import math
from typing import List, Optional, Tuple

# Import collision types
try:
    from ..collision.sphere import Vec3, CylinderGeometry
except ImportError:
    # Fallback for standalone testing
    import sys
    import os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from collision.sphere import Vec3, CylinderGeometry


def get_link_dimensions(branch: dict, link_idx: int) -> Tuple[float, float]:
    """
    Get dimensions (radius, height) for a specific link in a branch.
    
    Args:
        branch: Branch configuration dict
        link_idx: Index of link (0-indexed)
    
    Returns:
        (radius, height) tuple in meters
    
    Example:
        >>> branch = {"radius": 0.05, "height": 0.2, "n_links": 5}
        >>> get_link_dimensions(branch, 2)
        (0.05, 0.2)
    """
    radius = branch.get("radius", 0.05)  # Default 5cm
    height = branch.get("height", 0.2)   # Default 20cm per link
    
    # TODO: Handle tapering if implemented in future
    # For now, all links have same radius and height
    
    return radius, height


def link_to_cylinder_geometry(
    branch: dict,
    link_idx: int,
    base_position: Optional[Vec3] = None,
    parent_tip_position: Optional[Vec3] = None
) -> CylinderGeometry:
    """
    Convert a branch link to CylinderGeometry for collision detection.
    
    Args:
        branch: Branch configuration dict with keys:
            - n_links: number of links
            - height: height per link (m)
            - radius: radius (m)
            - tilt: tilt angle (degrees) [optional]
            - rot: rotation around parent axis (degrees) [optional]
        link_idx: Index of link within branch (0-indexed)
        base_position: Absolute base position (if None, assumes origin)
        parent_tip_position: Parent link tip position for computing tilt [optional]
    
    Returns:
        CylinderGeometry representing the link
    
    Note:
        Full 3D transformation (tilt, rotation) is complex. This version
        provides a simplified model suitable for collision pre-check.
        For precise geometry, integrate with USD stage builder.
    
    Example:
        >>> branch = {
        ...     "id": "trunk",
        ...     "n_links": 5,
        ...     "height": 0.2,
        ...     "radius": 0.05,
        ...     "tilt": 0.0
        ... }
        >>> geom = link_to_cylinder_geometry(branch, link_idx=2)
        >>> geom.base.z
        0.4  # 2 * 0.2m
    """
    n_links = branch.get("n_links", 1)
    if link_idx < 0 or link_idx >= n_links:
        raise ValueError(f"link_idx {link_idx} out of range [0, {n_links})")
    
    radius, link_height = get_link_dimensions(branch, link_idx)
    
    # Calculate base position
    if base_position is None:
        # Default: stack links vertically from origin
        base_z = link_idx * link_height
        base = Vec3(0.0, 0.0, base_z)
    else:
        base = base_position
    
    # Get tilt and rotation
    tilt_deg = branch.get("tilt", 0.0)
    rot_deg = branch.get("rot", 0.0)
    
    # Convert tilt to radians
    tilt_rad = math.radians(tilt_deg)
    rot_rad = math.radians(rot_deg)
    
    # Simplified axis calculation:
    # - No tilt: axis points up (0, 0, 1)
    # - With tilt: axis tilts in XZ plane by default
    # - With rot: rotate tilt direction around Z axis
    
    if abs(tilt_deg) < 1e-6:
        # Vertical (no tilt)
        axis = Vec3(0.0, 0.0, 1.0)
    else:
        # Tilt in direction determined by rot
        # Project tilt into XY plane based on rot
        axis_x = math.sin(tilt_rad) * math.cos(rot_rad)
        axis_y = math.sin(tilt_rad) * math.sin(rot_rad)
        axis_z = math.cos(tilt_rad)
        
        # Normalize
        axis_len = math.sqrt(axis_x**2 + axis_y**2 + axis_z**2)
        if axis_len > 1e-10:
            axis = Vec3(axis_x / axis_len, axis_y / axis_len, axis_z / axis_len)
        else:
            axis = Vec3(0.0, 0.0, 1.0)
    
    return CylinderGeometry(
        base=base,
        axis=axis,
        height=link_height,
        radius=radius
    )


def branch_to_cylinder_geometries(
    branch: dict,
    base_position: Optional[Vec3] = None
) -> List[CylinderGeometry]:
    """
    Convert all links in a branch to CylinderGeometry list.
    
    Args:
        branch: Branch configuration dict
        base_position: Absolute base position (default: origin)
    
    Returns:
        List of CylinderGeometry, one per link
    
    Example:
        >>> branch = {"n_links": 3, "height": 0.2, "radius": 0.05, "tilt": 0}
        >>> cylinders = branch_to_cylinder_geometries(branch)
        >>> len(cylinders)
        3
        >>> cylinders[0].base.z
        0.0
        >>> cylinders[2].base.z
        0.4
    """
    n_links = branch.get("n_links", 1)
    geometries = []
    
    if base_position is None:
        base_position = Vec3(0.0, 0.0, 0.0)
    
    # Get link dimensions
    _, link_height = get_link_dimensions(branch, 0)
    
    # Get tilt/rot for axis calculation
    tilt_deg = branch.get("tilt", 0.0)
    rot_deg = branch.get("rot", 0.0)
    tilt_rad = math.radians(tilt_deg)
    rot_rad = math.radians(rot_deg)
    
    # Calculate axis direction (same for all links in branch)
    if abs(tilt_deg) < 1e-6:
        axis = Vec3(0.0, 0.0, 1.0)
    else:
        axis_x = math.sin(tilt_rad) * math.cos(rot_rad)
        axis_y = math.sin(tilt_rad) * math.sin(rot_rad)
        axis_z = math.cos(tilt_rad)
        axis_len = math.sqrt(axis_x**2 + axis_y**2 + axis_z**2)
        if axis_len > 1e-10:
            axis = Vec3(axis_x / axis_len, axis_y / axis_len, axis_z / axis_len)
        else:
            axis = Vec3(0.0, 0.0, 1.0)
    
    # Create geometries for each link
    current_base = base_position
    for link_idx in range(n_links):
        radius, height = get_link_dimensions(branch, link_idx)
        
        geom = CylinderGeometry(
            base=current_base,
            axis=axis,
            height=height,
            radius=radius
        )
        geometries.append(geom)
        
        # Move to next link base
        current_base = Vec3(
            current_base.x + axis.x * height,
            current_base.y + axis.y * height,
            current_base.z + axis.z * height
        )
    
    return geometries
