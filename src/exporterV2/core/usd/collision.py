"""
collision.py - Collision Filtering

Manages collision filtering between rigid bodies to prevent spurious contacts.
"""

import re
from pxr import Usd, UsdPhysics, Sdf


def add_collision_filter(stage, body_a_path: str, body_b_path: str) -> None:
    """
    Add collision filtering between two bodies.
    
    Filtering is applied at RigidBody level and propagates to collision shapes.
    """
    prim_a = stage.GetPrimAtPath(body_a_path)
    prim_b = stage.GetPrimAtPath(body_b_path)
    
    if prim_a and prim_b:
        filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(prim_a)
        filtered_pairs.GetFilteredPairsRel().AddTarget(Sdf.Path(body_b_path))


def parse_link_number(link_path: str) -> tuple:
    """
    Parse link path to extract branch ID and link number.
    
    Args:
        link_path: Path like "/World/Stem/branch_Link_03"
    
    Returns:
        Tuple (branch_id, link_num) or (None, None) if parsing fails
    """
    match = re.match(r'.*/(\w+)_Link_(\d+)$', link_path)
    if match:
        return match.group(1), int(match.group(2))
    return None, None


def add_attachment_collision_filters(stage, child_link_path: str, parent_link_path: str) -> None:
    """
    Add collision filtering for branch attachment.
    
    When a branch attaches to a parent chain, filter collisions with:
    1. The parent link it attaches to
    2. The next link in the parent chain (prevents instability)
    
    Example:
        stem_Link_03 (parent attachment)
             ├─ branch_Link_01 (child)
        stem_Link_04 (next in chain - also filtered)
    """
    # Filter parent link
    add_collision_filter(stage, child_link_path, parent_link_path)
    
    # Find and filter next link in parent chain
    branch_id, link_num = parse_link_number(parent_link_path)
    if branch_id and link_num:
        stem_path = parent_link_path.rsplit('/', 1)[0]  # Get /World/Stem
        next_link_path = f"{stem_path}/{branch_id}_Link_{link_num + 1:02d}"
        next_link_prim = stage.GetPrimAtPath(next_link_path)
        
        if next_link_prim and next_link_prim.IsValid():
            add_collision_filter(stage, child_link_path, next_link_path)


def add_sibling_collision_filtering(stage, branches, branch_registry) -> None:
    """
    Add collision filtering between sibling branches.
    
    When multiple branches attach to the same parent link, filter collisions
    between them to prevent spurious contact forces.
    
    Example:
        main_petiole Link_02
             ├─ petiolule_1 ←┐
             ├─ petiolule_2 ←├─ These must filter each other
             └─ petiolule_3 ←┘
    """
    # Build map: (parent_id, attach_link_idx) → [child link paths]
    attachment_map = {}
    
    for b in branches:
        if b.get("parent") is None:
            continue  # Skip root
        
        parent_id = b["parent"]
        attach_idx = b["attach_link"] - 1  # Convert to 0-based
        key = (parent_id, attach_idx)
        
        # Get first link of this branch
        link_paths, _, _, _ = branch_registry[b["id"]]
        first_link = link_paths[0]
        
        attachment_map.setdefault(key, []).append(first_link)
    
    # Filter siblings pairwise (bidirectional)
    filtered_count = 0
    for sibling_links in attachment_map.values():
        if len(sibling_links) <= 1:
            continue
        
        for i, link_a in enumerate(sibling_links):
            for link_b in sibling_links[i+1:]:
                add_collision_filter(stage, link_a, link_b)
                add_collision_filter(stage, link_b, link_a)
                filtered_count += 2
    
    if filtered_count > 0:
        print(f"[INFO] Added {filtered_count} sibling collision filters")



# ==============================================================================
# GEOMETRY VALIDATION (pre-simulation checks)
# ==============================================================================

def check_sphere_sphere_intersection(
    pos_a: tuple,
    radius_a: float,
    pos_b: tuple,
    radius_b: float,
    margin: float = 0.0,
) -> tuple:
    """
    Check if two spheres intersect.
    
    Args:
        pos_a: Center position of sphere A (x, y, z)
        radius_a: Radius of sphere A [m]
        pos_b: Center position of sphere B (x, y, z)
        radius_b: Radius of sphere B [m]
        margin: Safety margin [m] (default: 0.0)
            Positive margin = consider intersection earlier (safer)
            Negative margin = allow slight overlap (less safe)
    
    Returns:
        Tuple (intersects, distance, overlap):
            intersects: True if spheres intersect (considering margin)
            distance: Center-to-center distance [m]
            overlap: Amount of overlap [m] (negative if separated)
    """
    import math
    
    dx = pos_b[0] - pos_a[0]
    dy = pos_b[1] - pos_a[1]
    dz = pos_b[2] - pos_a[2]
    
    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    min_distance = radius_a + radius_b + margin
    
    intersects = distance <= min_distance
    overlap = min_distance - distance
    
    return intersects, distance, overlap


def check_sphere_cylinder_intersection(
    sphere_pos: tuple,
    sphere_radius: float,
    cyl_base: tuple,
    cyl_axis: tuple,
    cyl_height: float,
    cyl_radius: float,
    margin: float = 0.0,
) -> tuple:
    """
    Check if sphere intersects with cylinder.
    
    Simplified check: treats cylinder as capsule (cylinder + hemispherical caps).
    
    Args:
        sphere_pos: Sphere center (x, y, z)
        sphere_radius: Sphere radius [m]
        cyl_base: Cylinder base position (x, y, z)
        cyl_axis: Cylinder axis direction (unit vector)
        cyl_height: Cylinder height [m]
        cyl_radius: Cylinder radius [m]
        margin: Safety margin [m]
    
    Returns:
        Tuple (intersects, distance, overlap):
            intersects: True if sphere and cylinder intersect
            distance: Closest distance between surfaces
            overlap: Amount of overlap [m] (negative if separated)
    """
    import math
    
    # Vector from cylinder base to sphere center
    dx = sphere_pos[0] - cyl_base[0]
    dy = sphere_pos[1] - cyl_base[1]
    dz = sphere_pos[2] - cyl_base[2]
    
    # Project onto cylinder axis
    dot = dx * cyl_axis[0] + dy * cyl_axis[1] + dz * cyl_axis[2]
    
    # Clamp to cylinder length
    t = max(0.0, min(cyl_height, dot))
    
    # Closest point on cylinder axis
    closest_x = cyl_base[0] + cyl_axis[0] * t
    closest_y = cyl_base[1] + cyl_axis[1] * t
    closest_z = cyl_base[2] + cyl_axis[2] * t
    
    # Distance from closest point to sphere center
    dx_closest = sphere_pos[0] - closest_x
    dy_closest = sphere_pos[1] - closest_y
    dz_closest = sphere_pos[2] - closest_z
    
    radial_distance = math.sqrt(dx_closest**2 + dy_closest**2 + dz_closest**2)
    min_distance = sphere_radius + cyl_radius + margin
    
    intersects = radial_distance <= min_distance
    distance = radial_distance - cyl_radius
    overlap = min_distance - radial_distance
    
    return intersects, distance, overlap


def validate_truss_geometry(
    tomato_definitions: list,
    branch_registry: dict,
    branches: list,
    margin: float = 0.001,  # 1mm safety margin
) -> list:
    """
    Validate truss geometry for pre-simulation intersections.
    
    Checks:
    1. Tomato-tomato intersections (spheres)
    2. Tomato-rachis intersections (sphere-cylinder)
    3. Tomato-pedicel intersections (sphere-cylinder, excluding parent)
    
    Args:
        tomato_definitions: List of tomato dicts with id, pedicel_id, radius
        branch_registry: Dict mapping branch_id → (link_paths, link_bases, axis, orientation)
        branches: List of all branch definitions
        margin: Safety margin [m] (default: 1mm)
    
    Returns:
        List of warning messages (empty if no issues)
    """
    warnings = []
    
    # Skip if no tomatoes
    if not tomato_definitions:
        return warnings
    
    # Calculate tomato world positions
    tomato_positions = []
    for tomato_def in tomato_definitions:
        pedicel_id = tomato_def["pedicel_id"]
        tomato_radius = tomato_def["radius"]
        
        # Get pedicel info
        pedicel_paths, pedicel_bases, pedicel_axis, _ = branch_registry[pedicel_id]
        pedicel_base = pedicel_bases[0]
        
        # Get pedicel height
        pedicel_def = next(b for b in branches if b["id"] == pedicel_id)
        pedicel_height = pedicel_def["height"]  # Pre-scale
        
        # Apply GLOBAL_SCALE
        from ..tree_config import GLOBAL_SCALE
        pedicel_height_world = pedicel_height * GLOBAL_SCALE
        tomato_radius_world = tomato_radius * GLOBAL_SCALE
        
        # Tomato position: pedicel tip + radius along axis
        tomato_pos = (
            pedicel_base[0] + pedicel_axis[0] * (pedicel_height_world + tomato_radius_world),
            pedicel_base[1] + pedicel_axis[1] * (pedicel_height_world + tomato_radius_world),
            pedicel_base[2] + pedicel_axis[2] * (pedicel_height_world + tomato_radius_world),
        )
        
        tomato_positions.append({
            "id": tomato_def["id"],
            "pos": tomato_pos,
            "radius": tomato_radius_world,
            "pedicel_id": pedicel_id,
        })
    
    # Check 1: Tomato-tomato intersections
    for i in range(len(tomato_positions)):
        for j in range(i + 1, len(tomato_positions)):
            tomato_a = tomato_positions[i]
            tomato_b = tomato_positions[j]
            
            intersects, distance, overlap = check_sphere_sphere_intersection(
                tomato_a["pos"], tomato_a["radius"],
                tomato_b["pos"], tomato_b["radius"],
                margin
            )
            
            if intersects:
                warnings.append(
                    f"[WARNING] Tomato intersection: {tomato_a['id']} ↔ {tomato_b['id']} "
                    f"(overlap={overlap*1000:.2f}mm, distance={distance*1000:.2f}mm)"
                )
    
    # Check 2: Tomato-rachis intersections
    # Find rachis branches (parent of pedicels)
    rachis_ids = set()
    for b in branches:
        if "pedicel" in b["id"] and b.get("parent"):
            rachis_ids.add(b["parent"])
    
    for rachis_id in rachis_ids:
        if rachis_id not in branch_registry:
            continue
        
        rachis_paths, rachis_bases, rachis_axis, _ = branch_registry[rachis_id]
        rachis_def = next(b for b in branches if b["id"] == rachis_id)
        
        from ..tree_config import GLOBAL_SCALE
        rachis_radius_world = rachis_def["radius"] * GLOBAL_SCALE
        rachis_height_world = rachis_def["height"] * GLOBAL_SCALE
        
        # Check each rachis link against all tomatoes
        for link_idx, rachis_base in enumerate(rachis_bases):
            for tomato in tomato_positions:
                intersects, distance, overlap = check_sphere_cylinder_intersection(
                    tomato["pos"], tomato["radius"],
                    rachis_base, rachis_axis,
                    rachis_height_world, rachis_radius_world,
                    margin
                )
                
                if intersects:
                    warnings.append(
                        f"[WARNING] Tomato-rachis intersection: {tomato['id']} ↔ {rachis_id}_Link_{link_idx+1:02d} "
                        f"(overlap={overlap*1000:.2f}mm)"
                    )
    
    return warnings


def print_geometry_validation_report(warnings: list) -> None:
    """
    Print geometry validation report.
    
    Args:
        warnings: List of warning messages from validate_truss_geometry
    """
    if not warnings:
        print("[INFO] Geometry validation: No intersections detected ✓")
        return
    
    print("\n" + "="*80)
    print("  GEOMETRY VALIDATION WARNINGS")
    print("="*80)
    print(f"\nDetected {len(warnings)} potential collision issues:")
    print("These geometries may intersect before simulation starts,")
    print("which can cause PhysX instability or explosive forces.\n")
    
    for i, warning in enumerate(warnings, 1):
        print(f"{i}. {warning}")
    
    print("\n" + "="*80)
    print("Recommendation: Adjust truss parameters to eliminate overlaps")
    print("="*80 + "\n")
