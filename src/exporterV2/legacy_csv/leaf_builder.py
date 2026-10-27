"""
leaf_builder.py - Leaf-Specific Branch Construction

Builds petiole, rachis, and petiolule branches from leaf CSV data.
"""

from typing import List, Dict
from pathlib import Path
from functools import lru_cache

PETIOLULE_TIP_RADIUS_SCALE = 0.65

def _visual_segment(
    branch_id: str,
    length: float,
    radius: float,
    end_radius: float = None,
) -> Dict:
    """Preserve visual geometry independently of physics."""

    segment = {
        "source_id": branch_id,
        "length": length,
        "radius": radius,
    }

    if end_radius is not None:
        segment["end_radius"] = end_radius

    return segment


@lru_cache(maxsize=1)
def _load_tree_config():
    """Load tree_config in package and standalone execution modes."""
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


def calculate_leaf_orientation(leaf_dict: Dict) -> tuple:
    """
    Calculate leaf orientation from CSV data.
    
    Uses ccw_orientation if present (>1e-3), otherwise falls back to phyllotaxis.
    
    Args:
        leaf_dict: Leaf dict from load_leaves()
    
    Returns:
        Tuple (azimuth_deg, tilt_deg):
            azimuth_deg: Rotation around trunk Z-axis [deg]
            tilt_deg: Tilt from vertical [deg]
    """
    tree_config = _load_tree_config()
    
    ccw_orientation = leaf_dict.get("ccw_orientation", 0.0)
    angle_petiole = leaf_dict.get("angle_petiole", 90.0)
    rank = leaf_dict["rank"]
    
    # Use ccw_orientation if explicitly set (>1e-3), otherwise use phyllotaxis
    if abs(ccw_orientation) > 1e-3:
        azimuth_deg = ccw_orientation
    else:
        # Fallback to phyllotaxis (golden angle)
        azimuth_deg = (rank * tree_config.PHYLLOTAXIS) % 360.0
    
    tilt_deg = angle_petiole
    
    return azimuth_deg, tilt_deg


def leaf_to_petiole_rachis_branches(
    leaf_dict: Dict,
    parent_trunk_id: str,
    *,
    include_rachis: bool = True,
) -> List[Dict]:
    """
    Convert leaf CSV data to petiole and rachis branch definitions.
    
    Creates two branches:
    1. Petiole: attaches to trunk at parent_rank
    2. Rachis: attaches to top of petiole (if rachis_length > 0)
    
    Applies MIN_LINK_RADIUS_WORLD clamping to all radii.
    Rachis is subdivided into multiple links (one per petiolule attachment point).
    
    Args:
        leaf_dict: Leaf dict from load_leaves()
        parent_trunk_id: ID of trunk branch (e.g., "trunk")
    
    Returns:
        List of branch dicts: [petiole_branch, rachis_branch] or [petiole_branch] if no rachis
    """
    tree_config = _load_tree_config()
    
    clamp_radius = tree_config.clamp_radius
    GLOBAL_SCALE = tree_config.GLOBAL_SCALE
    MIN_LINK_RADIUS_WORLD = tree_config.MIN_LINK_RADIUS_WORLD
    
    rank = leaf_dict["rank"]
    organ_index = leaf_dict["organ_index"]
    parent_rank = leaf_dict["parent_rank"]
    
    # Calculate radii with clamping
    petiole_r_prescale = leaf_dict["diameter_petiole"] / 2.0
    petiole_r, petiole_clamped = clamp_radius(petiole_r_prescale)
    
    rachis_r_prescale = petiole_r_prescale * 0.6
    rachis_r, rachis_clamped = clamp_radius(rachis_r_prescale)
    
    # Log clamping warnings
    if petiole_clamped:
        print(f"[WARNING] Leaf rank={rank} organ_index={organ_index} petiole radius clamped: "
              f"{petiole_r_prescale * GLOBAL_SCALE:.5f}m → {petiole_r * GLOBAL_SCALE:.5f}m "
              f"(world-space, min={MIN_LINK_RADIUS_WORLD}m)")
    if rachis_clamped:
        print(f"[WARNING] Leaf rank={rank} organ_index={organ_index} rachis radius clamped: "
              f"{rachis_r_prescale * GLOBAL_SCALE:.5f}m → {rachis_r * GLOBAL_SCALE:.5f}m "
              f"(world-space, min={MIN_LINK_RADIUS_WORLD}m)")
    
    # Get order once (0=trunk, 1=lateral branch)
    order = leaf_dict.get("order", 0)
    
    # Calculate orientation based on order
    if order == 0:
        # Trunk leaves: use CSV data
        azimuth_deg, tilt_deg = calculate_leaf_orientation(leaf_dict)
    else:
        # Lateral branch leaves: oriented more coaxially with branch (upward)
        # Rotation is RELATIVE to parent branch axis (branch is already rotated 0°/180°)
        # So we always use same random range for both organ_index
        import random
        random.seed(rank * 1000 + organ_index)  # Deterministic per leaf
        
        # Random rotation between -90° and +90° (relative to branch axis)
        # This works for both organ_index because the parent branch is already rotated
        azimuth_deg = random.uniform(-90.0, 90.0)
        
        # Normalize to [0, 360)
        azimuth_deg = azimuth_deg % 360.0
        
        # tilt: ~35° to be more coaxial with 45° branch (pointing upward along branch)
        # Lower tilt = more aligned with branch axis
        tilt_deg = 35.0
    
    # Create unique IDs with organ_index AND order to distinguish trunk/lateral leaves
    if order == 0:
        leaf_id_base = f"Leaf_r{rank}_o{organ_index}"
    else:
        leaf_id_base = f"LatLeaf_r{rank}_o{organ_index}"
    
    # Determine attach_link based on order:
    # - order=0 (trunk leaves): attach to trunk link = parent_rank + 1
    # - order=1 (lateral branch leaves): attach to branch link = 1 (only link)
    if order == 0:
        attach_link = parent_rank + 1  # Trunk has multiple links
    else:
        attach_link = 1  # Lateral branches have only 1 link
    
    # Create petiole branch
    visual_axis_id = f"{leaf_id_base}_axis"
    petiole_id = f"{leaf_id_base}_petiole"
    petiole_branch = {
        "id": petiole_id,
        "system": "vegetative",
        "visual_axis_id": visual_axis_id,
        "visual_segments": [
            _visual_segment(petiole_id, leaf_dict["length_petiole"], petiole_r)
        ],
        "parent": parent_trunk_id,
        "attach_link": attach_link,
        "n_links": 1,
        "radius": petiole_r,
        "height": leaf_dict["length_petiole"],
        "tilt": tilt_deg,
        "rot": azimuth_deg,
    }
    
    branches = [petiole_branch]
    
    # Create rachis branch if length > 0
    rachis_length = leaf_dict["rachis_length"]
    if include_rachis and rachis_length > 1e-6:
        # Calculate number of rachis links based on lateral blade pairs
        # Each pair needs an attachment point + 1 for terminal
        blades_nr = leaf_dict["blades_nr"]
        lateral_pairs = blades_nr - 1  # Last blade is terminal
        
        # Rachis needs at least lateral_pairs links for attachment points
        # Plus we want smooth distribution, so use max(lateral_pairs, 1)
        n_rachis_links = max(lateral_pairs, 1)
        
        rachis_id = f"{leaf_id_base}_rachis"
        rachis_branch = {
            "id": rachis_id,
            "system": "vegetative",
            "visual_axis_id": visual_axis_id,
            "visual_segments": [
                _visual_segment(rachis_id, rachis_length, rachis_r)
            ],
            "parent": petiole_branch["id"],
            "attach_link": 1,  # Top of petiole
            "n_links": n_rachis_links,
            "radius": rachis_r,
            "height": rachis_length / n_rachis_links,  # Distribute evenly
            "tilt": 0.0,  # Continue in same direction as petiole
            "rot": 0.0,
        }
        branches.append(rachis_branch)
    
    return branches


def _generate_leaf_mesh_data(hw: float, L: float) -> dict:
    import math
    points = [[0.0, 0.0, 0.0]]  # 0: base (attachment)
    
    n_side = 8
    # Right side (CCW: base to tip)
    for i in range(1, n_side):
        t = i / n_side
        z = L * t
        x = hw * math.sin(math.pi * t) * (1.2 - 0.4 * t)
        points.append([x, 0.0, z])
        
    points.append([0.0, 0.0, L])  # Tip
    
    # Left side (CCW: tip back to base)
    for i in range(n_side - 1, 0, -1):
        t = i / n_side
        z = L * t
        x = hw * math.sin(math.pi * t) * (1.2 - 0.4 * t)
        points.append([-x, 0.0, z])

    num_triangles = len(points) - 2
    face_vertex_counts = [3] * num_triangles
    
    indices = []
    for i in range(1, len(points) - 1):
        indices.extend([0, i, i + 1])
        
    return {
        "points": points,
        "face_vertex_counts": face_vertex_counts,
        "indices": indices
    }


def create_lateral_petiolules(leaf_dict: Dict, rachis_id: str, petiole_radius: float) -> tuple[List[Dict], List[Dict]]:
    """
    Create lateral petiolule branches (pairs) along the rachis.
    
    Each lateral blade pair creates two petiolule branches (left/right) at 90° from rachis.
    The attach_link is calculated based on segment distribution along the rachis.
    
    Args:
        leaf_dict: Leaf dict from load_leaves()
        rachis_id: ID of parent rachis branch
        petiole_radius: Radius of petiole (pre-scale) for scaling petiolule radius
    
    Returns:
        Tuple (branches, terminal_bodies):
            branches: List of petiolule branch dicts (2 per lateral pair)
            terminal_bodies: List of leaf blade mesh dicts (2 per lateral pair)
    """
    tree_config = _load_tree_config()
    
    clamp_radius = tree_config.clamp_radius
    
    rank = leaf_dict["rank"]
    organ_index = leaf_dict["organ_index"]
    blades_nr = leaf_dict["blades_nr"]
    lateral_pairs = blades_nr - 1  # Last blade is terminal
    
    if lateral_pairs <= 0:
        return [], []  # No lateral blades
    
    # Calculate petiolule radius (40% of petiole)
    petiolule_r_prescale = petiole_radius * 0.4
    petiolule_r, clamped = clamp_radius(petiolule_r_prescale)
    
    if clamped:
        print(f"[WARNING] Leaf rank={rank} organ_index={organ_index} petiolule radius clamped")
    
    # Get inclination angles (default 90° if not in CSV)
    inclination_array = leaf_dict["inclination_segments"]
    
    # Get area arrays
    area_array = leaf_dict.get("area_m2blades", [])
    area_blades_total = leaf_dict.get("area_blades_total", 0.0)
    
    # Petiolule fixed length: 1cm
    petiolule_length = 0.01
    
    branches = []
    terminal_bodies = []
    
    import math
    
    # Create pairs
    for j in range(lateral_pairs):
        # Calculate area/length for this pair
        pair_area = area_array[j] if j < len(area_array) else (area_blades_total / blades_nr)
        lat_area = pair_area / 2.0
        lat_length = math.sqrt(lat_area / 0.6) if lat_area > 0 else 0.0
        lat_width = lat_length * 0.6
        
        hw = lat_width / 2.0
        L = lat_length
        mesh_data = _generate_leaf_mesh_data(hw, L) if L > 0 else None
        
        # Determine which link of the rachis to attach to (1-based)
        # Distribute evenly: if rachis has N links, pair j attaches to link (j+1)
        attach_link_idx = j + 1  # 1-based: first pair on link 1, second on link 2, etc.
        
        # Get inclination angle for this pair
        if j < len(inclination_array):
            inclination = inclination_array[j]
        else:
            inclination = 90.0  # Default perpendicular
        
        left_id = f"{rachis_id}_petiolule_lat_{j}_left"
        right_id = f"{rachis_id}_petiolule_lat_{j}_right"
        
        # Create left petiolule
        branches.append({
            "id": left_id,
            "leaflet_role": "left",
            "leaflet_id": left_id,
            "system": "vegetative",
            "visual_axis_id": left_id,
            "visual_segments": [
                _visual_segment(
                    left_id,
                    petiolule_length,
                    petiolule_r,
                    end_radius=(
                        petiolule_r
                        * PETIOLULE_TIP_RADIUS_SCALE
                    ),
                )
            ],
            "parent": rachis_id,
            "attach_link": attach_link_idx,
            "n_links": 1,
            "radius": petiolule_r,
            "height": petiolule_length,
            "tilt": inclination,
            "rot": 90.0,  # Left = +90° from rachis direction
        })
        
        if mesh_data:
            terminal_bodies.append({
                "id": f"{left_id}_blade",
                "kind": "leaf_blade",
                "shape": "mesh",
                "parent_branch_id": left_id,
                "mass": 0.005,  # 5g
                "roll": 90.0,  # Rotate 90° around local Z to align with branch plane
                **mesh_data
            })
        
        # Create right petiolule
        branches.append({
            "id": right_id,
            "leaflet_role": "right",
            "leaflet_id": right_id,
            "system": "vegetative",
            "visual_axis_id": right_id,
            "visual_segments": [
                _visual_segment(
                    right_id,
                    petiolule_length,
                    petiolule_r,
                    end_radius=(
                        petiolule_r
                        * PETIOLULE_TIP_RADIUS_SCALE
                    ),
                )
            ],
            "parent": rachis_id,
            "attach_link": attach_link_idx,
            "n_links": 1,
            "radius": petiolule_r,
            "height": petiolule_length,
            "tilt": inclination,
            "rot": 270.0,  # Right = -90° (or 270°) from rachis direction
        })
        
        if mesh_data:
            terminal_bodies.append({
                "id": f"{right_id}_blade",
                "kind": "leaf_blade",
                "shape": "mesh",
                "parent_branch_id": right_id,
                "mass": 0.005,  # 5g
                "roll": 90.0,  # Rotate 90° around local Z to align with branch plane
                **mesh_data
            })
    
    return branches, terminal_bodies


def create_terminal_petiolule(
    rachis_id: str,
    rachis_n_links: int,
    petiole_radius: float,
    leaf_dict: Dict,
    visual_axis_id: str = None,
) -> tuple[Dict, Dict]:
    """
    Create terminal petiolule branch at the end of rachis.
    
    Attaches to the last link of the rachis, aligned (0° tilt, 0° rot).
    
    Args:
        rachis_id: ID of parent rachis branch
        rachis_n_links: Number of links in the rachis
        petiole_radius: Radius of petiole (pre-scale) for scaling petiolule radius
        leaf_dict: Leaf dictionary containing area parameters and metadata
    
    Returns:
        Tuple (branch, terminal_body):
            branch: Terminal petiolule branch dict
            terminal_body: Leaf blade mesh dict
    """
    tree_config = _load_tree_config()
    
    clamp_radius = tree_config.clamp_radius
    
    rank = leaf_dict.get("rank", 0)
    organ_index = leaf_dict.get("organ_index", 0)
    
    # Calculate petiolule radius (40% of petiole)
    petiolule_r_prescale = petiole_radius * 0.4
    petiolule_r, clamped = clamp_radius(petiolule_r_prescale)
    
    if clamped:
        print(f"[WARNING] Leaf rank={rank} organ_index={organ_index} terminal petiolule radius clamped")
    
    # Calculate dimensions
    blades_nr = leaf_dict.get("blades_nr", 1)
    area_array = leaf_dict.get("area_m2blades", [])
    area_blades_total = leaf_dict.get("area_blades_total", 0.0)
    
    terminal_area = area_array[-1] if len(area_array) >= blades_nr else (area_blades_total / blades_nr if blades_nr > 0 else 4e-4)
    
    import math
    terminal_length = math.sqrt(terminal_area / 0.6) if terminal_area > 0 else 0.0
    terminal_width = terminal_length * 0.6
    
    hw = terminal_width / 2.0
    L = terminal_length
    mesh_data = _generate_leaf_mesh_data(hw, L) if L > 0 else None
    
    # Petiolule fixed length: 1cm
    petiolule_length = 0.01
    
    term_id = f"{rachis_id}_petiolule_term"
    if visual_axis_id is None:
        visual_axis_id = (
            f"{rachis_id[:-len('_rachis')]}_axis"
            if rachis_id.endswith("_rachis")
            else term_id
        )
    
    branch = {
        "id": term_id,
        "leaflet_role": "terminal",
        "leaflet_id": term_id,
        "system": "vegetative",
        "visual_axis_id": visual_axis_id,
        "visual_segments": [
            _visual_segment(
                term_id,
                petiolule_length,
                petiolule_r,
                end_radius=(
                    petiolule_r
                    * PETIOLULE_TIP_RADIUS_SCALE
                ),
            )
        ],
        "parent": rachis_id,
        "attach_link": rachis_n_links,  # Last link of rachis
        "n_links": 1,
        "radius": petiolule_r,
        "height": petiolule_length,
        "tilt": 0.0,  # Aligned with rachis
        "rot": 0.0,
    }
    
    terminal_body = {
        "id": f"{term_id}_blade",
        "kind": "leaf_blade",
        "shape": "mesh",
        "parent_branch_id": term_id,
        "mass": 0.005,  # 5g
    }
    if mesh_data:
        terminal_body.update(mesh_data)
        
    return branch, terminal_body
