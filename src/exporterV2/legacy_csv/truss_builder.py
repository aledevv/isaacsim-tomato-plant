"""
truss_builder.py - Truss-Specific Branch Construction

Builds rachis and pedicel branches from truss data (FruitsNode in CSV).
Similar to leaf structure but terminates with tomatoes (spheres) instead of leaflets.
"""

from typing import List, Dict
from pathlib import Path
from functools import lru_cache

TOMATO_DENSITY = 1000.0  # kg/m^3, close to water


def _fruit_layout(n_fruits: int) -> tuple[int, bool]:
    """
    Return the number of lateral fruit pairs and whether a terminal fruit exists.
    Even fruit counts are all lateral pairs; odd counts keep one terminal fruit.
    """
    n_fruits = max(int(n_fruits), 0)
    return n_fruits // 2, (n_fruits % 2) == 1


@lru_cache(maxsize=1)
def _load_tree_config():
    """Load tree_config without importing pxr-dependent USD modules."""
    try:
        from exporterV2.core import tree_config
        return tree_config
    except ImportError:
        import importlib.util

        config_path = Path(__file__).parent.parent / "core" / "tree_config.py"
        spec = importlib.util.spec_from_file_location("tree_config", config_path)
        tree_config = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(tree_config)
        return tree_config


def _pedicel_geometry(truss_dict: Dict, rachis_id: str, *, terminal: bool = False):
    """Return shared pedicel geometry and physics config for one truss."""
    tree_config = _load_tree_config()
    truss_geometry = tree_config.TrussGeometryConfig
    pedicel_length = truss_dict.get("pedicel_length", truss_geometry.PEDICEL_LENGTH)
    pedicel_radius_prescale = truss_dict.get(
        "pedicel_radius",
        truss_geometry.PEDICEL_RADIUS,
    )
    pedicel_r, clamped = tree_config.clamp_radius(pedicel_radius_prescale)

    if clamped:
        label = "terminal pedicel" if terminal else "pedicel"
        print(f"[WARNING] Truss {rachis_id} {label} radius clamped")

    return tree_config, pedicel_length, pedicel_r


def _make_pedicel_branch(
    tree_config,
    *,
    branch_id: str,
    rachis_id: str,
    attach_link: int,
    radius: float,
    height: float,
    tilt: float,
    rot: float,
) -> Dict:
    """Create one truss pedicel branch with the standard soft D6 settings."""
    return {
        "id": branch_id,
        "system": "truss",
        "parent": rachis_id,
        "attach_link": attach_link,
        "n_links": 1,
        "radius": radius,
        "height": height,
        "tilt": tilt,
        "rot": rot,
        "physics_profile": "truss",
        "joint_type": "d6",
        "bend_limit_deg": tree_config.TrussPhysicsConfig.PEDICEL_BEND_LIMIT_DEG,
        "drive_stiffness_scale": tree_config.TrussPhysicsConfig.PEDICEL_DRIVE_STIFFNESS_SCALE,
    }


def truss_rachis_to_branch(
    truss_dict: Dict,
    parent_trunk_id: str,
    rank: int,
    organ_index: int = 0,
) -> Dict:
    """
    Convert truss data to rachis branch definition.
    
    Creates a single articulated rachis branch that attaches directly to the trunk.
    No petiole (simplified structure compared to leaves).
    
    The rachis is subdivided into multiple links (one per pedicel pair + terminal).
    
    Args:
        truss_dict: Truss dict with keys:
            - rachis_length: Total length of rachis [m, pre-scale]
            - rachis_radius: Radius of rachis [m, pre-scale]
            - n_fruits: Number of fruits (determines n_links)
            - tilt_deg: Initial tilt from vertical [deg]
            - azimuth_deg: Rotation around trunk Z-axis [deg]
        parent_trunk_id: ID of trunk branch (e.g., "trunk")
        rank: Truss rank (attachment position on trunk)
        organ_index: Organ index (default: 0)
    
    Returns:
        Branch dict for rachis in BRANCHES format
    """
    tree_config = _load_tree_config()
    truss_geometry = tree_config.TrussGeometryConfig
    clamp_radius = tree_config.clamp_radius
    GLOBAL_SCALE = tree_config.GLOBAL_SCALE
    MIN_LINK_RADIUS_WORLD = tree_config.MIN_LINK_RADIUS_WORLD
    PHYLLOTAXIS = tree_config.PHYLLOTAXIS
    
    # Extract truss parameters with defaults
    n_fruits = truss_dict.get("n_fruits", 5)  # Default 5 fruits
    lateral_pairs, has_terminal = _fruit_layout(n_fruits)
    default_rachis_links = max(lateral_pairs + int(has_terminal), 1)
    rachis_length = truss_dict.get(
        "rachis_length",
        truss_geometry.RACHIS_SEGMENT_LENGTH * default_rachis_links,
    )
    rachis_radius_prescale = truss_dict.get("rachis_radius", truss_geometry.RACHIS_RADIUS)
    parent_rank = truss_dict.get("parent_rank", rank)
    
    # Orientation: use phyllotaxis if not specified
    tilt_deg = truss_dict.get("tilt_deg", None)
    if tilt_deg is None:
        tilt_deg = truss_geometry.INITIAL_TILT_DEG
    
    azimuth_deg = truss_dict.get("azimuth_deg", None)
    if azimuth_deg is None:
        # Use phyllotaxis (golden angle)
        azimuth_deg = (rank * PHYLLOTAXIS) % 360.0
    
    # Apply radius clamping
    rachis_r, clamped = clamp_radius(rachis_radius_prescale)
    
    if clamped:
        print(f"[WARNING] Truss rank={rank} organ_index={organ_index} rachis radius clamped: "
              f"{rachis_radius_prescale * GLOBAL_SCALE:.5f}m → {rachis_r * GLOBAL_SCALE:.5f}m "
              f"(world-space, min={MIN_LINK_RADIUS_WORLD}m)")
    
    n_rachis_links = max(lateral_pairs + int(has_terminal), 1)
    
    # Create unique ID
    truss_id_base = f"Truss_r{rank}_o{organ_index}"
    
    # Determine attach_link (1-based indexing)
    attach_link = parent_rank + 1
    
    # Create rachis branch
    rachis_branch = {
        "id": f"{truss_id_base}_rachis",
        "system": "truss",
        "parent": parent_trunk_id,
        "attach_link": attach_link,
        "n_links": n_rachis_links,
        "radius": rachis_r,
        "height": rachis_length / n_rachis_links,  # Distribute evenly
        "tilt": tilt_deg,
        "rot": azimuth_deg,
        "physics_profile": "truss",
    }
    
    return rachis_branch


def create_lateral_pedicels(
    truss_dict: Dict,
    rachis_id: str,
    rachis_n_links: int,
    rachis_radius: float,
) -> List[Dict]:
    """
    Create lateral pedicel branches (pairs) along the rachis.
    
    Each fruit pair creates two pedicel branches (left/right) alternating ±90° from rachis.
    Similar to petiolules in leaves, but for tomatoes.
    
    Args:
        truss_dict: Truss dict with keys:
            - n_fruits: Number of fruits (determines number of pairs)
            - pedicel_length: Length of each pedicel [m, pre-scale]
            - pedicel_radius: Radius of pedicels [m, pre-scale] (optional)
            - pedicel_angle: Inclination angle from rachis [deg] (optional)
        rachis_id: ID of parent rachis branch
        rachis_n_links: Number of links in the rachis
        rachis_radius: Reserved for API compatibility; current pedicel radius
            comes from TrussGeometryConfig or truss_dict["pedicel_radius"].
    
    Returns:
        List of pedicel branch dicts (2 per lateral pair)
    """
    _ = rachis_radius
    n_fruits = truss_dict.get("n_fruits", 5)
    lateral_pairs, _ = _fruit_layout(n_fruits)

    if lateral_pairs <= 0:
        return []  # Only terminal fruit, no lateral pairs

    tree_config, pedicel_length, pedicel_r = _pedicel_geometry(
        truss_dict,
        rachis_id,
        terminal=True,
    )
    pedicel_angle = truss_dict.get(
        "pedicel_angle",
        tree_config.TrussGeometryConfig.LATERAL_PEDICEL_CHORD_ANGLE_DEG
    )
    branches = []

    for j in range(lateral_pairs):
        attach_link_idx = min(j + 1, rachis_n_links)
        for suffix, rot in (("L", 90.0), ("R", 270.0)):
            branches.append(
                _make_pedicel_branch(
                    tree_config,
                    branch_id=f"{rachis_id}_pedicel_lat_{j}_{suffix}",
                    rachis_id=rachis_id,
                    attach_link=attach_link_idx,
                    radius=pedicel_r,
                    height=pedicel_length,
                    tilt=pedicel_angle,
                    rot=rot,
                )
            )

    return branches


def create_terminal_pedicel(
    truss_dict: Dict,
    rachis_id: str,
    rachis_n_links: int,
    rachis_radius: float,
) -> Dict:
    """
    Create terminal pedicel branch at the end of rachis.
    
    Attaches to the last link of the rachis, aligned coaxially (0° tilt, 0° rot).
    
    Args:
        truss_dict: Truss dict with keys:
            - pedicel_length: Length of pedicel [m, pre-scale]
            - pedicel_radius: Radius of pedicel [m, pre-scale] (optional)
        rachis_id: ID of parent rachis branch
        rachis_n_links: Number of links in the rachis
        rachis_radius: Reserved for API compatibility; current pedicel radius
            comes from TrussGeometryConfig or truss_dict["pedicel_radius"].
    
    Returns:
        Terminal pedicel branch dict
    """
    _ = rachis_radius
    tree_config, pedicel_length, pedicel_r = _pedicel_geometry(truss_dict, rachis_id)
    return _make_pedicel_branch(
        tree_config,
        branch_id=f"{rachis_id}_pedicel_term",
        rachis_id=rachis_id,
        attach_link=rachis_n_links,
        radius=pedicel_r,
        height=pedicel_length,
        tilt=0.0,
        rot=0.0,
    )


def truss_to_branch_config(
    truss_dict: Dict,
    parent_trunk_id: str,
    rank: int,
    organ_index: int = 0,
    *,
    include_pedicels: bool = True,
) -> List[Dict]:
    """
    Convert truss data to complete branch configuration.
    
    Creates:
    1. Rachis branch (articulated main stem)
    2. Lateral pedicel branches (pairs, alternating ±90°)
    3. Terminal pedicel branch (coaxial with rachis)
    
    Tomato spheres are NOT included here; terminal-body authoring handles them.
    
    Args:
        truss_dict: Truss dict with keys:
            - rachis_length: Total length of rachis [m, pre-scale]
            - rachis_radius: Radius of rachis [m, pre-scale]
            - n_fruits: Number of fruits
            - pedicel_length: Length of each pedicel [m, pre-scale]
            - pedicel_radius: Radius of pedicels [m, pre-scale] (optional)
            - pedicel_angle: Inclination angle from rachis [deg] (optional)
            - tilt_deg: Initial tilt from vertical [deg] (optional)
            - azimuth_deg: Rotation around trunk Z-axis [deg] (optional)
            - parent_rank: Attachment position on trunk (optional, defaults to rank)
        parent_trunk_id: ID of trunk branch (e.g., "trunk")
        rank: Truss rank (attachment position on trunk)
        organ_index: Organ index (default: 0)
    
    Returns:
        List of branch dicts: [rachis, pedicel_1L, pedicel_1R, ..., pedicel_term]
    """
    branches = []
    
    # Create rachis
    rachis_branch = truss_rachis_to_branch(
        truss_dict,
        parent_trunk_id,
        rank,
        organ_index
    )
    branches.append(rachis_branch)
    
    rachis_id = rachis_branch["id"]
    rachis_n_links = rachis_branch["n_links"]
    rachis_radius = rachis_branch["radius"]
    
    # Create lateral pedicels
    if include_pedicels:
        lateral_pedicels = create_lateral_pedicels(
            truss_dict,
            rachis_id,
            rachis_n_links,
            rachis_radius
        )
        branches.extend(lateral_pedicels)
    
    # Create terminal pedicel (if n_fruits > 0)
    n_fruits = truss_dict.get("n_fruits", 5)
    _, has_terminal = _fruit_layout(n_fruits)
    if include_pedicels and has_terminal:
        terminal_pedicel = create_terminal_pedicel(
            truss_dict,
            rachis_id,
            rachis_n_links,
            rachis_radius
        )
        branches.append(terminal_pedicel)
    
    return branches



# ==============================================================================
# TOMATO DEFINITIONS (for sphere attachment)
# ==============================================================================

def create_tomato_definitions(
    truss_dict: Dict,
    pedicel_ids: List[str],
) -> List[Dict]:
    """
    Create tomato sphere definitions for attachment to pedicel tips.
    
    Each tomato is a sphere that will be rigidly attached to a pedicel tip
    with a FixedJoint. USD authoring decides whether that joint remains in the
    articulation or uses the experimental native detachment settings.
    
    Args:
        truss_dict: Truss dict with keys:
            - tomato_radii: List of radii for each tomato [m, pre-scale]
            - tomato_masses: List of masses for each tomato [kg] (optional)
            - maturation: List of maturation states (0=unripe, 1=ripe) (optional)
        pedicel_ids: List of pedicel IDs to attach tomatoes to
            Order: [lat_0_L, lat_0_R, lat_1_L, lat_1_R, ..., term]
    
    Returns:
        List of tomato definition dicts with keys:
            - id: Unique ID for tomato
            - pedicel_id: Parent pedicel ID
            - radius: Sphere radius [m, pre-scale]
            - mass: Sphere mass [kg]
            - maturation: Maturation state (0.0=unripe, 1.0=ripe)
    """
    import math
    
    # Extract tomato parameters
    tomato_radii = truss_dict.get("tomato_radii", [])
    tomato_masses = truss_dict.get("tomato_masses", None)
    maturation_states = truss_dict.get("maturation", None)
    
    # If no radii specified, use defaults
    if not tomato_radii:
        n_fruits = truss_dict.get("n_fruits", 5)
        # Default: 3cm radius (medium tomato)
        tomato_radii = [0.03] * n_fruits
    
    # Drop invalid fruit radii before mass calculation. GroIMP can emit zeros
    # for fruits that are not physically present yet.
    valid_radii = []
    valid_indices = []
    for i, radius in enumerate(tomato_radii):
        if radius > 0.0:
            valid_indices.append(i)
            valid_radii.append(radius)
        else:
            print(f"[WARNING] Skipping tomato {i} with non-positive radius: {radius}")
    tomato_radii = valid_radii
    n_tomatoes = len(tomato_radii)

    if maturation_states is not None:
        maturation_states = [
            maturation_states[i] if i < len(maturation_states) else 0.0
            for i in valid_indices
        ]
    if tomato_masses is not None:
        tomato_masses = [
            tomato_masses[i] if i < len(tomato_masses) else None
            for i in valid_indices
        ]

    # Ensure we have enough pedicels for tomatoes
    if n_tomatoes > len(pedicel_ids):
        print(f"[WARNING] More tomatoes ({n_tomatoes}) than pedicels ({len(pedicel_ids)}). "
              f"Truncating to {len(pedicel_ids)} tomatoes.")
        tomato_radii = tomato_radii[:len(pedicel_ids)]
        n_tomatoes = len(pedicel_ids)
    
    def mass_from_radius(radius: float) -> float:
        return (4.0 / 3.0) * math.pi * (radius ** 3) * TOMATO_DENSITY

    if tomato_masses is None:
        tomato_masses = [mass_from_radius(radius) for radius in tomato_radii]
    else:
        tomato_masses = [
            mass if mass is not None and mass > 0.0
            else mass_from_radius(tomato_radii[i])
            for i, mass in enumerate(tomato_masses[:n_tomatoes])
        ]
    
    # Default maturation: all unripe
    if maturation_states is None:
        maturation_states = [0.0] * n_tomatoes
    
    # Create tomato definitions
    tomato_defs = []
    for i in range(n_tomatoes):
        tomato_defs.append({
            "id": f"{pedicel_ids[i]}_tomato",
            "pedicel_id": pedicel_ids[i],
            "radius": tomato_radii[i],
            "mass": tomato_masses[i],
            "maturation": maturation_states[i],
        })
    
    return tomato_defs


def truss_to_complete_config(
    truss_dict: Dict,
    parent_trunk_id: str,
    rank: int,
    organ_index: int = 0,
    *,
    include_pedicels: bool = True,
    include_tomatoes: bool = True,
) -> tuple:
    """
    Convert truss data to complete configuration including tomatoes.
    
    This is the main entry point for truss generation that includes both
    articulated branches (rachis + pedicels) and leaf nodes (tomatoes).
    
    Args:
        truss_dict: Complete truss dict (see truss_to_branch_config for details)
        parent_trunk_id: ID of trunk branch (e.g., "trunk")
        rank: Truss rank (attachment position on trunk)
        organ_index: Organ index (default: 0)
    
    Returns:
        Tuple (branches, tomatoes):
            branches: List of branch dicts for rachis and pedicels
            tomatoes: List of tomato definition dicts
    """
    # Generate branch structure
    branches = truss_to_branch_config(
        truss_dict,
        parent_trunk_id,
        rank,
        organ_index,
        include_pedicels=include_pedicels,
    )
    
    # Extract pedicel IDs (all branches except rachis)
    pedicel_ids = [b["id"] for b in branches[1:]]  # Skip rachis (first branch)
    
    # Generate tomato definitions
    tomatoes = []
    if include_pedicels and include_tomatoes:
        tomatoes = create_tomato_definitions(
            truss_dict,
            pedicel_ids
        )
    
    return branches, tomatoes
