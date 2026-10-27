"""
broad_phase.py - Two-Stage Broad-Phase Collision Detection

Orchestrates the two-stage collision detection pipeline:
    Stage 1: Sphere overlap (fast pre-check)
    Stage 2: AABB overlap (precision check)

This hybrid approach balances performance and accuracy:
- Sphere check is O(1) and very fast
- AABB check is also O(1) but slightly more expensive
- Most cases are rejected by sphere check, avoiding AABB computation

Usage:
    from collision.broad_phase import check_attachment_collision
    
    result = check_attachment_collision(new_link, siblings, parent, margin=0.01)
    if result.collision_detected:
        print(f"Collision with: {result.colliding_with}")
"""

from typing import List, Tuple
from dataclasses import dataclass

from .sphere import (
    Vec3, CylinderGeometry,
    calculate_bounding_sphere,
    check_sphere_overlap,
    check_sphere_overlap_detailed
)
from .aabb import (
    calculate_aabb,
    check_aabb_overlap,
    check_aabb_overlap_detailed
)


@dataclass
class CollisionResult:
    """
    Result of collision detection.
    
    Attributes:
        collision_detected: True if collision found
        colliding_with: List of IDs of colliding objects
        stage_detected: Which stage detected collision ("sphere", "aabb", "none")
        details: Optional dict with detailed collision info
    """
    collision_detected: bool
    colliding_with: List[str]
    stage_detected: str  # "sphere", "aabb", or "none"
    details: dict = None


def check_attachment_collision(
    new_link: CylinderGeometry,
    siblings: List[Tuple[str, CylinderGeometry]],
    parent: CylinderGeometry,
    margin: float = 0.01,
    check_parent: bool = True
) -> CollisionResult:
    """
    Two-stage broad-phase collision check for attachment validation.
    
    Checks if a new link (being attached after remapping) collides with:
    - Sibling links (other branches at same/adjacent parent ranks)
    - Parent link (optional, usually not needed for attachment)
    
    Args:
        new_link: Geometry of the link being attached
        siblings: List of (id, geometry) tuples for sibling links
        parent: Geometry of parent link
        margin: Safety margin in meters (default: 0.01 = 1cm)
        check_parent: Whether to check collision with parent (default: True)
    
    Returns:
        CollisionResult with detection status and details
    
    Algorithm:
        Stage 1 (Sphere Pre-check):
            - Calculate bounding spheres for all objects
            - Check new_link sphere against each sibling/parent sphere
            - If no overlap → return "no collision"
            - If overlap → collect candidates for Stage 2
        
        Stage 2 (AABB Precision):
            - For each candidate from Stage 1:
                - Calculate AABBs
                - Check AABB overlap
                - If overlap → collision detected
    
    Example:
        >>> new_link = CylinderGeometry(...)
        >>> siblings = [
        ...     ("branch1", CylinderGeometry(...)),
        ...     ("branch2", CylinderGeometry(...))
        ... ]
        >>> parent = CylinderGeometry(...)
        >>> result = check_attachment_collision(new_link, siblings, parent)
        >>> if result.collision_detected:
        ...     print(f"Collision with: {result.colliding_with}")
    """
    # Stage 1: Sphere overlap pre-check
    new_sphere = calculate_bounding_sphere(new_link)
    
    sphere_candidates = []  # Objects that passed sphere check
    
    # Check against siblings
    for sibling_id, sibling_geom in siblings:
        sibling_sphere = calculate_bounding_sphere(sibling_geom)
        
        if check_sphere_overlap(new_sphere, sibling_sphere, margin):
            sphere_candidates.append((sibling_id, sibling_geom))
    
    # Check against parent (if requested)
    if check_parent:
        parent_sphere = calculate_bounding_sphere(parent)
        if check_sphere_overlap(new_sphere, parent_sphere, margin):
            sphere_candidates.append(("parent", parent))
    
    # If no sphere overlaps, we're done
    if not sphere_candidates:
        return CollisionResult(
            collision_detected=False,
            colliding_with=[],
            stage_detected="none",
            details={"sphere_checks": len(siblings) + (1 if check_parent else 0)}
        )
    
    # Stage 2: AABB precision check for candidates
    new_aabb = calculate_aabb(new_link)
    
    collisions = []
    aabb_details = []
    
    for candidate_id, candidate_geom in sphere_candidates:
        candidate_aabb = calculate_aabb(candidate_geom)
        
        overlap, overlap_x, overlap_y, overlap_z = check_aabb_overlap_detailed(
            new_aabb, candidate_aabb
        )
        
        if overlap:
            collisions.append(candidate_id)
            aabb_details.append({
                "id": candidate_id,
                "overlap_x": overlap_x,
                "overlap_y": overlap_y,
                "overlap_z": overlap_z
            })
    
    # Return result
    if collisions:
        return CollisionResult(
            collision_detected=True,
            colliding_with=collisions,
            stage_detected="aabb",
            details={
                "sphere_candidates": len(sphere_candidates),
                "aabb_checks": len(sphere_candidates),
                "aabb_details": aabb_details
            }
        )
    else:
        # Sphere overlap but no AABB overlap (false positive from sphere)
        return CollisionResult(
            collision_detected=False,
            colliding_with=[],
            stage_detected="sphere_only",
            details={
                "sphere_candidates": len(sphere_candidates),
                "aabb_checks": len(sphere_candidates),
                "note": "Sphere overlap but AABB separated (conservative sphere)"
            }
        )


def check_pairwise_collisions(
    links: List[Tuple[str, CylinderGeometry]],
    margin: float = 0.01
) -> List[Tuple[str, str]]:
    """
    Check all pairwise collisions in a list of links.
    
    Useful for validating an entire branches configuration.
    
    Args:
        links: List of (id, geometry) tuples
        margin: Safety margin in meters
    
    Returns:
        List of (id1, id2) tuples for colliding pairs
    
    Complexity: O(n²) where n is number of links
    
    Example:
        >>> links = [
        ...     ("branch1", CylinderGeometry(...)),
        ...     ("branch2", CylinderGeometry(...)),
        ...     ("branch3", CylinderGeometry(...))
        ... ]
        >>> collisions = check_pairwise_collisions(links)
        >>> if collisions:
        ...     print(f"Found {len(collisions)} collisions")
    """
    collisions = []
    n = len(links)
    
    for i in range(n):
        id1, geom1 = links[i]
        
        for j in range(i + 1, n):
            id2, geom2 = links[j]
            
            # Stage 1: Sphere check
            sphere1 = calculate_bounding_sphere(geom1)
            sphere2 = calculate_bounding_sphere(geom2)
            
            if not check_sphere_overlap(sphere1, sphere2, margin):
                continue  # No overlap, skip AABB
            
            # Stage 2: AABB check
            aabb1 = calculate_aabb(geom1)
            aabb2 = calculate_aabb(geom2)
            
            if check_aabb_overlap(aabb1, aabb2):
                collisions.append((id1, id2))
    
    return collisions

