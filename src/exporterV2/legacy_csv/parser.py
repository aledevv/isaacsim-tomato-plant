"""
parser.py - Generic CSV Parsing

Handles loading and filtering of groIMP CSV export data.
"""

import os
import json
import math
import pandas as pd
from datetime import datetime
from typing import List, Dict, Tuple
from pathlib import Path
from collections import defaultdict

from exporterV2.core import tree_config

TRUSS_GEOMETRY = tree_config.TrussGeometryConfig


def _read_csv_frame(csv_path: str) -> pd.DataFrame:
    """Read and normalize one GroIMP CSV export."""
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    dataframe = pd.read_csv(csv_path, skipinitialspace=True)
    dataframe.columns = dataframe.columns.str.strip()
    return dataframe


def _csv_frame(csv_path: str, dataframe: pd.DataFrame = None) -> pd.DataFrame:
    """Use a pipeline-owned frame or load one for a standalone loader call."""
    return dataframe if dataframe is not None else _read_csv_frame(csv_path)


def _effective_generation_settings(profile: dict) -> Dict[str, bool]:
    """Combine global debug switches with cultivar-profile permissions."""
    config = tree_config.OrganGenerationConfig
    lateral_profile = profile.get("lateral_branches", {}).get("enabled", True)
    trunk_leaf_profile = profile.get("trunk_leaves", {}).get("enabled", True)
    lateral_leaf_profile = profile.get("lateral_leaves", {}).get("enabled", True)
    truss_profile = profile.get("trusses", {}).get("enabled", True)

    lateral_branches = config.CREATE_LATERAL_BRANCHES and lateral_profile
    leaf_base = config.CREATE_LEAF_BRANCHES and config.CREATE_PETIOLES
    return {
        "lateral_branches": lateral_branches,
        "trunk_leaves": leaf_base and trunk_leaf_profile,
        "lateral_leaves": leaf_base and lateral_branches and lateral_leaf_profile,
        "petioles": leaf_base,
        "leaf_rachis": leaf_base and config.CREATE_LEAF_RACHIS,
        "petiolules": leaf_base and config.CREATE_LEAF_RACHIS and config.CREATE_PETIOLULES,
        "trusses": config.CREATE_TRUSSES and config.CREATE_TRUSS_RACHIS and truss_profile,
        "truss_rachis": config.CREATE_TRUSSES and config.CREATE_TRUSS_RACHIS and truss_profile,
        "pedicels": (
            config.CREATE_TRUSSES
            and config.CREATE_TRUSS_RACHIS
            and config.CREATE_PEDICELS
            and truss_profile
        ),
        "tomatoes": (
            config.CREATE_TRUSSES
            and config.CREATE_TRUSS_RACHIS
            and config.CREATE_PEDICELS
            and config.CREATE_TOMATOES
            and truss_profile
        ),
    }


def _info(message: str) -> None:
    """Print detailed plant-generation info only when enabled."""
    if tree_config.LoggingConfig.VERBOSE_PLANT_INFO:
        print(f"[INFO] {message}")


def _print_generation_settings(settings: Dict[str, bool]) -> None:
    values = ", ".join(
        f"{name}={'on' if enabled else 'off'}"
        for name, enabled in settings.items()
    )
    print(f"[CONFIG] Organ generation: {values}")


def _parse_float_array(val_str: str) -> List[float]:
    """
    Parse float arrays from CSV strings.
    Example: "0.1_0.2_0.3" -> [0.1, 0.2, 0.3]
    Empty or "0" returns empty list.
    """
    s = str(val_str).strip()
    if s in ("0", "0.0", "", "nan", "None"):
        return []
    return [float(x) for x in s.split("_")]


def _count_d6_joints(branches: List[Dict]) -> int:
    """Count only branches that still use D6 joints."""
    return sum(
        b.get("n_links", 1)
        for b in branches
        if b.get("joint_type", "d6").lower() != "fixed"
    )


def _filter_invalid_branches(branches: List[Dict]) -> List[Dict]:
    """
    Remove branches with invalid dimensions and descendants that depend on them.
    """
    valid = []
    removed_ids = set()

    for branch in branches:
        bid = branch["id"]
        parent = branch.get("parent")

        invalid_parent = parent in removed_ids
        invalid_geometry = (
            branch.get("n_links", 0) <= 0
            or branch.get("radius", 0.0) <= 0.0
            or branch.get("height", 0.0) <= 0.0
        )

        if invalid_parent or invalid_geometry:
            removed_ids.add(bid)
            reason = "parent was skipped" if invalid_parent else "invalid dimensions"
            print(f"[WARNING] Skipping branch '{bid}' ({reason})")
            continue

        valid.append(branch)

    return valid


def _normalize_terminal_bodies(tomatoes: List[Dict]) -> List[Dict]:
    """Convert truss tomato definitions to the generic terminal body schema."""
    terminal_bodies = []

    for tomato in tomatoes:
        radius = tomato.get("radius", 0.0)
        mass = tomato.get("mass", 0.0)
        parent_branch_id = tomato.get("pedicel_id") or tomato.get("parent_branch_id")

        if radius <= 0.0 or mass <= 0.0 or not parent_branch_id:
            print(f"[WARNING] Skipping terminal body '{tomato.get('id', '<unknown>')}' with invalid data")
            continue

        terminal_bodies.append({
            "id": tomato["id"],
            "kind": "tomato",
            "shape": "sphere",
            "parent_branch_id": parent_branch_id,
            "radius": radius,
            "mass": mass,
            "maturation": tomato.get("maturation", 0.0),
        })

    return terminal_bodies


def _filter_terminal_bodies(terminal_bodies: List[Dict], branches: List[Dict]) -> List[Dict]:
    """Keep only terminal bodies whose parent branch survived filtering/optimization."""
    branch_ids = {b["id"] for b in branches}
    filtered = []

    for body in terminal_bodies:
        parent_branch_id = body.get("parent_branch_id")
        if parent_branch_id not in branch_ids:
            print(
                f"[WARNING] Skipping terminal body '{body.get('id', '<unknown>')}' "
                f"because parent branch '{parent_branch_id}' is missing"
            )
            continue
        filtered.append(body)

    return filtered


def _truss_tilt_from_groimp_angle(truss_angle: float, n_fruits: int) -> float:
    """
    Convert GroIMP fruit_truss_angle to exporter tilt from vertical.

    The CSV angle bends each truss segment. V2 represents the rachis as a
    straight articulated chain, so use the average bend as the initial pose.
    """
    bend_steps = max(n_fruits - 2, 0) / 2.0
    tilt_deg = TRUSS_GEOMETRY.INITIAL_TILT_DEG + truss_angle * bend_steps

    return max(
        TRUSS_GEOMETRY.MIN_TILT_DEG,
        min(tilt_deg, TRUSS_GEOMETRY.MAX_TILT_DEG),
    )


def load_trunk_internodes(
    csv_path: str,
    day: int,
    plant_id: int = 1,
    *,
    _dataframe: pd.DataFrame = None,
) -> List[Dict]:
    """
    Load trunk internodes (order=0) from groIMP CSV export.
    
    Args:
        csv_path: Path to CSV file (e.g., graph_day_1.csv)
        day: Simulation day
        plant_id: Plant identifier (default: 1)
    
    Returns:
        List of internode dicts with fields: rank, organ_index, width_m, length
        Sorted by rank (ascending, from base to top)
    
    Raises:
        ValueError: If no trunk internodes found
        FileNotFoundError: If CSV file doesn't exist
    """
    df = _csv_frame(csv_path, _dataframe)
    
    # Filter for trunk internodes: order=0 (main stem)
    mask = (
        (df["day"] == day) &
        (df["plant_id"] == plant_id) &
        (df["organ_class"].str.strip() == "Internode") &
        (df["order"] == 0)
    )
    trunk_df = df[mask].copy()
    
    if trunk_df.empty:
        raise ValueError(f"No trunk internodes found for day={day}, plant_id={plant_id}, order=0")
    
    # Sort by rank (ascending: base to top)
    trunk_df = trunk_df.sort_values("rank")
    
    # Extract relevant fields
    internodes = []
    for _, row in trunk_df.iterrows():
        internodes.append({
            "rank": int(row["rank"]),
            "organ_index": int(row["organ_index"]),
            "width_m": float(row["internode_width_m"]),
            "length": float(row["length"]),
        })
    
    _info(f"Found {len(internodes)} trunk internodes for day {day}")
    
    return internodes


def load_lateral_branches(
    csv_path: str,
    day: int,
    plant_id: int = 1,
    profile: dict = None,
    *,
    _dataframe: pd.DataFrame = None,
) -> List[Dict]:
    """
    Load lateral branches (order=1) from groIMP CSV export with cultivar-specific filtering.
    
    Filtering logic is controlled by profile configuration.
    Default (tomato): Keep only organ_index 0 and 1 (opposite pairs).
    
    Args:
        csv_path: Path to CSV file (e.g., graph_day_1.csv)
        day: Simulation day
        plant_id: Plant identifier (default: 1)
        profile: Cultivar profile dict (default: None = no filtering)
    
    Returns:
        List of lateral branch dicts with fields: rank, organ_index, parent_rank, 
        width_m, length
        Sorted by parent_rank, then rank, then organ_index (ascending)
    
    Raises:
        FileNotFoundError: If CSV file doesn't exist
    """
    if profile is None:
        profile = {}  # No filtering by default
    df = _csv_frame(csv_path, _dataframe)
    
    # Filter for lateral branches: order=1 (attached to trunk internodes)
    mask = (
        (df["day"] == day) &
        (df["plant_id"] == plant_id) &
        (df["organ_class"].str.strip() == "Internode") &
        (df["order"] == 1)
    )
    branch_df = df[mask].copy()
    
    if branch_df.empty:
        print(f"[INFO] No lateral branches found for day={day}, plant_id={plant_id}")
        return []
    
    # Sort by parent_rank, rank, organ_index
    branch_df = branch_df.sort_values(["parent_rank", "rank", "organ_index"])
    
    # Group by (rank, parent_rank) to find opposite pairs
    branches_by_key = defaultdict(list)
    
    for _, row in branch_df.iterrows():
        branch_data = {
            "rank": int(row["rank"]),
            "organ_index": int(row["organ_index"]),
            "parent_rank": int(row["parent_rank"]),
            "width_m": float(row["internode_width_m"]),
            "length": float(row["length"]),
        }
        key = (branch_data["rank"], branch_data["parent_rank"])
        branches_by_key[key].append(branch_data)
    
    # Filter: keep only organ_indices specified in profile (e.g., [0, 1] for opposite pairs)
    # If no profile, keep all
    lateral_config = profile.get("lateral_branches", {})
    organ_indices_filter = lateral_config.get("organ_indices", None)
    
    filtered_branches = []
    n_pairs = 0
    n_singles = 0
    
    if organ_indices_filter:
        # Apply filtering
        for key, key_branches in sorted(branches_by_key.items()):
            for organ_idx in organ_indices_filter:
                branch = next((b for b in key_branches if b["organ_index"] == organ_idx), None)
                if branch:
                    filtered_branches.append(branch)
                    if organ_idx == organ_indices_filter[0]:
                        n_singles += 1
                    else:
                        n_pairs += 1
    else:
        # No filtering - keep all
        filtered_branches = [b for branches in branches_by_key.values() for b in branches]
        n_singles = len(filtered_branches)
    
    # Sort final list by parent_rank, rank, organ_index
    filtered_branches.sort(key=lambda x: (x["parent_rank"], x["rank"], x["organ_index"]))
    
    print(f"[INFO] Found {len(filtered_branches)} lateral branches for day {day} "
          f"({n_pairs} opposite pairs + {n_singles} singles)")
    
    return filtered_branches


def load_trusses(
    csv_path: str,
    day: int,
    plant_id: int = 1,
    order: int = 0,
    *,
    _dataframe: pd.DataFrame = None,
) -> List[Dict]:
    """
    Load trusses (fruit clusters) from groIMP CSV export.
    
    Args:
        csv_path: Path to CSV file (e.g., graph_day_96.csv)
        day: Simulation day
        plant_id: Plant identifier (default: 1)
        order: Truss order (0=trunk trusses, 1=lateral branch trusses)
    
    Returns:
        List of truss dicts with fields: rank, organ_index, parent_rank, order,
        fruit_nr, fruit_radii, fruit_age_dd, fruit_ripening_dd, truss_angle
        Sorted by rank, then organ_index (ascending)
    
    Raises:
        FileNotFoundError: If CSV file doesn't exist
    """
    df = _csv_frame(csv_path, _dataframe)
    
    # Filter for trusses (Fruits organ_class) with specified order
    mask = (
        (df["day"] == day) &
        (df["plant_id"] == plant_id) &
        (df["organ_class"].str.strip() == "Fruits") &
        (df["order"] == order)
    )
    truss_df = df[mask].copy()
    
    if truss_df.empty:
        order_str = "trunk" if order == 0 else f"lateral (order={order})"
        print(f"[INFO] No {order_str} trusses found for day={day}, plant_id={plant_id}")
        return []
    
    # Sort by rank, then organ_index
    truss_df = truss_df.sort_values(["rank", "organ_index"])
    
    # Extract truss data
    trusses = []
    for _, row in truss_df.iterrows():
        truss_data = {
            "rank": int(row["rank"]),
            "organ_index": int(row["organ_index"]),
            "parent_rank": int(row["parent_rank"]),
            "order": int(row["order"]),
            "fruit_nr": int(row["fruit_nr"]),
            "fruit_radii": _parse_float_array(row["fruit_radii"]),
            "fruit_age_dd": _parse_float_array(row["fruit_age_dd"]),
            "fruit_ripening_dd": float(row["fruit_ripening_dd"]),
            "truss_angle": float(row["fruit_truss_angle"]),
        }
        trusses.append(truss_data)
    
    order_str = "trunk" if order == 0 else f"lateral (order={order})"
    print(f"[INFO] Found {len(trusses)} {order_str} trusses for day {day}")
    
    return trusses


def load_leaves(
    csv_path: str,
    day: int,
    plant_id: int = 1,
    order: int = 0,
    *,
    _dataframe: pd.DataFrame = None,
) -> List[Dict]:
    """
    Load leaves from groIMP CSV export with opposite pair filtering.
    
    NOTE: Opposite pair filtering (180° difference) is specific to this tomato cultivar.
          Other cultivars may have different leaf arrangement patterns (phyllotaxis).
    
    Filtering logic:
    - Group leaves by rank
    - Keep both leaves if they form an opposite pair (180° difference in ccw_orientation)
    - Keep single leaves without opposite pair
    - Remove clones (same position)
    
    Args:
        csv_path: Path to CSV file (e.g., graph_day_1.csv)
        day: Simulation day
        plant_id: Plant identifier (default: 1)
        order: Leaf order (0=trunk leaves, 1=lateral branch leaves)
    
    Returns:
        List of leaf dicts with fields: rank, organ_index, parent_rank, order, length_petiole, 
        diameter_petiole, angle_petiole, ccw_orientation, rachis_length,
        blades_nr, segments_length_array, inclination_array
        Sorted by rank, then organ_index (ascending)
    
    Raises:
        FileNotFoundError: If CSV file doesn't exist
    """
    df = _csv_frame(csv_path, _dataframe)
    
    # Filter for leaves with specified order
    # For order=1 (lateral branch leaves): CULTIVAR-SPECIFIC filter for organ_index 0+1 only
    # (to match the lateral branch opposite pair filtering)
    if order == 1:
        mask = (
            (df["day"] == day) &
            (df["plant_id"] == plant_id) &
            (df["organ_class"].str.strip() == "Leaf") &
            (df["order"] == order) &
            (df["organ_index"].isin([0, 1]))  # CULTIVAR-SPECIFIC: opposite pairs only
        )
    else:
        # For order=0 (trunk leaves): NO organ_index filter - we need all to find opposite pairs
        mask = (
            (df["day"] == day) &
            (df["plant_id"] == plant_id) &
            (df["organ_class"].str.strip() == "Leaf") &
            (df["order"] == order)
        )
    leaf_df = df[mask].copy()
    
    if leaf_df.empty:
        order_str = "trunk" if order == 0 else f"lateral (order={order})"
        print(f"[INFO] No {order_str} leaves found for day={day}, plant_id={plant_id}")
        return []
    
    # Sort by rank, then organ_index
    leaf_df = leaf_df.sort_values(["rank", "organ_index"])
    
    # Group by rank to find opposite pairs
    leaves_by_rank = defaultdict(list)
    def _safe_float(val, default=0.0):
        try:
            f = float(val)
            return default if math.isnan(f) else f
        except (ValueError, TypeError):
            return default

    for _, row in leaf_df.iterrows():
        leaf_data = {
            "rank": int(row["rank"]),
            "organ_index": int(row["organ_index"]),
            "parent_rank": int(row["parent_rank"]),
            "order": int(row["order"]),
            "length_petiole": _safe_float(row["leaf_length_petiole"]),
            "diameter_petiole": _safe_float(row["leaf_diameter_petiole"]),
            "angle_petiole": _safe_float(row["leaf_angle_petiole"], 90.0),
            "ccw_orientation": _safe_float(row["leaf_ccw_orientation"]),
            "rachis_length": _safe_float(row["leaf_rachis_length"]),
            "blades_nr": int(row["leaf_blades_nr"]),
            "segments_length": _parse_float_array(row["leaf_segments_length"]),
            "inclination_segments": _parse_float_array(row["leaf_inclination_segments"]),
            "area_m2blades": _parse_float_array(row["leaf_area_m2blades"]),
            "area_blades_total": _safe_float(row.get("leaf_area_blades_total", 0.0)),
        }
        leaves_by_rank[leaf_data["rank"]].append(leaf_data)
    
    # Filter logic depends on order:
    # - order=0 (trunk leaves): Find opposite pairs (180° difference) or keep singles
    # - order=1 (lateral leaves): Keep all (already filtered to organ_index 0+1)
    #   NOTE: Both filtering strategies are cultivar-specific
    filtered_leaves = []
    n_pairs = 0
    n_singles = 0
    
    if order == 1:
        # Lateral branch leaves: keep all (no opposite pair filtering needed)
        # Already filtered to organ_index 0+1 at CSV load time
        filtered_leaves = list(leaves_by_rank.values())
        filtered_leaves = [leaf for rank_leaves in filtered_leaves for leaf in rank_leaves]
        n_singles = len(filtered_leaves)
    else:
        # Trunk leaves: apply opposite pair filtering
        for rank, rank_leaves in sorted(leaves_by_rank.items()):
            if len(rank_leaves) == 1:
                # Single leaf, always keep
                filtered_leaves.append(rank_leaves[0])
                n_singles += 1
            else:
                # Multiple leaves: find opposite pairs
                used = set()
                for i, leaf_i in enumerate(rank_leaves):
                    if i in used:
                        continue
                        
                    # Look for opposite pair
                    found_pair = False
                    for j, leaf_j in enumerate(rank_leaves):
                        if i >= j or j in used:
                            continue
                        
                        # Check if they form an opposite pair (180° difference)
                        angle_diff = abs(leaf_i["ccw_orientation"] - leaf_j["ccw_orientation"])
                        # Normalize to [0, 360)
                        angle_diff = angle_diff % 360
                        if angle_diff > 180:
                            angle_diff = 360 - angle_diff
                        
                        if abs(angle_diff - 180.0) < 1e-6:  # Exact 180° difference
                            # Found opposite pair
                            filtered_leaves.append(leaf_i)
                            filtered_leaves.append(leaf_j)
                            used.add(i)
                            used.add(j)
                            found_pair = True
                            n_pairs += 1
                            break
                    
                    # If no pair found, check for clones (same position)
                    if not found_pair and i not in used:
                        # Check if this is a clone of an already-added leaf
                        is_clone = False
                        for existing in filtered_leaves:
                            if (existing["rank"] == leaf_i["rank"] and
                                abs(existing["ccw_orientation"] - leaf_i["ccw_orientation"]) < 1e-6):
                                is_clone = True
                                break
                        
                        if not is_clone:
                            # Single leaf without pair, keep it
                            filtered_leaves.append(leaf_i)
                            used.add(i)
                            n_singles += 1
    
    # CULTIVAR-SPECIFIC: For lateral branch leaves (order=1), ensure both leaves in opposite pair exist
    # If only one leaf exists for a (rank, organ_index 0+1) pair, clone it for the missing organ_index
    if order == 1:
        # Group filtered leaves by rank
        leaves_by_rank_final = defaultdict(list)
        for leaf in filtered_leaves:
            leaves_by_rank_final[leaf["rank"]].append(leaf)
        
        cloned_leaves = []
        for rank, rank_leaves in leaves_by_rank_final.items():
            organ_indices = {leaf["organ_index"] for leaf in rank_leaves}
            
            # Check if we have both organ_index 0 and 1
            if 0 in organ_indices and 1 not in organ_indices:
                # Clone organ_index 0 → 1
                template = next(leaf for leaf in rank_leaves if leaf["organ_index"] == 0)
                cloned = template.copy()
                cloned["organ_index"] = 1
                cloned_leaves.append(cloned)
            elif 1 in organ_indices and 0 not in organ_indices:
                # Clone organ_index 1 → 0
                template = next(leaf for leaf in rank_leaves if leaf["organ_index"] == 1)
                cloned = template.copy()
                cloned["organ_index"] = 0
                cloned_leaves.append(cloned)
        
        filtered_leaves.extend(cloned_leaves)
        if cloned_leaves:
            print(f"[INFO] Cloned {len(cloned_leaves)} lateral leaves to complete opposite pairs")
    
    # Sort final list by rank, then organ_index
    filtered_leaves.sort(key=lambda x: (x["rank"], x["organ_index"]))
    
    order_str = "trunk" if order == 0 else f"lateral (order={order})"
    print(f"[INFO] Found {len(filtered_leaves)} {order_str} leaves for day {day} "
          f"({n_pairs} opposite pairs + {n_singles} singles)")
    
    return filtered_leaves


def lateral_branches_to_branch_config(lateral_branches: List[Dict], trunk_id: str = "trunk", profile: dict = None) -> List[Dict]:
    """
    Convert lateral branches to BRANCHES format.
    
    Orientation logic is controlled by profile configuration.
    Default (tomato): tilt=45°, base rot (0°/180°) + random jitter with collision check.
    
    Args:
        lateral_branches: List of lateral branch dicts from load_lateral_branches()
        trunk_id: ID of trunk branch (default: "trunk")
        profile: Cultivar profile dict (default: None = use defaults)
    
    Returns:
        List of branch dicts in BRANCHES format
    """
    if profile is None:
        profile = {}
    
    lateral_config = profile.get("lateral_branches", {})
    tilt_deg = lateral_config.get("tilt_deg", 45.0)
    rot_base_deg = lateral_config.get("rot_base_deg", [0.0, 180.0])
    rot_jitter_deg = lateral_config.get("rot_jitter_deg", 0.0)
    min_angle_sep = lateral_config.get("min_angle_separation_deg", 60.0)
    
    import random
    
    clamp_radius = tree_config.clamp_radius
    GLOBAL_SCALE = tree_config.GLOBAL_SCALE
    MIN_LINK_RADIUS_WORLD = tree_config.MIN_LINK_RADIUS_WORLD
    
    # Group by (rank, organ_index) to calculate averages
    grouped = defaultdict(list)
    
    for branch in lateral_branches:
        key = (branch["rank"], branch["organ_index"])
        grouped[key].append(branch)
    
    # Convert each group to BRANCHES format
    branches = []
    
    # Track rotations per parent_rank for anti-collision
    # Key: parent_rank, Value: list of rotations
    rotations_by_parent = defaultdict(list)
    
    for (rank, organ_index), group in sorted(grouped.items()):
        # Calculate averages
        avg_radius = sum(b["width_m"] / 2.0 for b in group) / len(group)
        avg_height = sum(b["length"] for b in group) / len(group)
        parent_rank = group[0]["parent_rank"]  # Same for all in group
        
        # Apply radius clamping
        radius_final, was_clamped = clamp_radius(avg_radius)
        
        if was_clamped:
            radius_world_original = avg_radius * GLOBAL_SCALE
            radius_world_clamped = radius_final * GLOBAL_SCALE
            print(f"[WARNING] Lateral branch rank={rank} organ_index={organ_index} radius clamped: "
                  f"{radius_world_original:.4f}m → {radius_world_clamped:.4f}m "
                  f"(min {MIN_LINK_RADIUS_WORLD}m at scale {GLOBAL_SCALE})")
        
        # Calculate rotation with jitter + anti-collision
        # Base rotation from profile
        if organ_index < len(rot_base_deg):
            rot_base = rot_base_deg[organ_index]
        else:
            rot_base = organ_index * 90.0
        
        # Add random jitter
        if rot_jitter_deg > 0:
            random.seed(rank * 1000 + organ_index)
            jitter = random.uniform(-rot_jitter_deg, rot_jitter_deg)
            rot_deg = (rot_base + jitter) % 360.0
        else:
            rot_deg = rot_base
        
        # Anti-collision check: avoid branches too close in angle
        # Check against branches on same parent_rank and adjacent ranks (±1)
        collision_check_ranks = [parent_rank, parent_rank - 1, parent_rank + 1]
        
        max_attempts = 10
        for attempt in range(max_attempts):
            collision_found = False
            
            for check_rank in collision_check_ranks:
                for existing_rot in rotations_by_parent.get(check_rank, []):
                    # Calculate shortest angular distance
                    angle_diff = abs(rot_deg - existing_rot)
                    angle_diff = min(angle_diff, 360.0 - angle_diff)
                    
                    if angle_diff < min_angle_sep:
                        # Collision detected! Adjust rotation
                        collision_found = True
                        # Shift by min_angle_sep + 5° for safety margin
                        rot_deg = (existing_rot + min_angle_sep + 5.0) % 360.0
                        break
                
                if collision_found:
                    break
            
            if not collision_found:
                break  # No collision, success!
            
            if attempt == max_attempts - 1:
                print(f"[WARNING] Branch rank={rank} organ_index={organ_index}: "
                      f"Could not fully resolve collision after {max_attempts} attempts, using rot={rot_deg:.1f}°")
        
        # Record this rotation for future collision checks
        rotations_by_parent[parent_rank].append(rot_deg)
        
        # Create BRANCHES format dict
        branch = {
            "id": f"Branch_r{rank}_o{organ_index}",
            "system": "vegetative",
            "parent": trunk_id,
            "attach_link": parent_rank + 1,  # 1-based indexing
            "n_links": len(group),
            "radius": radius_final,
            "height": avg_height,
            "tilt": tilt_deg,
            "rot": rot_deg,
        }
        branch["visual_axis_id"] = branch["id"]
        branch["visual_segments"] = [{
            "source_id": branch["id"],
            "length": avg_height * len(group),
            "radius": radius_final,
        }]
        
        branches.append(branch)
    
    return branches


def internodes_to_branch_config(internodes: List[Dict]) -> Dict:
    """
    Convert list of trunk internodes to BRANCHES format.
    
    Uses average radius and height across all internodes.
    Applies minimum radius clamping for PhysX stability.
    
    Args:
        internodes: List of trunk internode dicts from load_trunk_internodes()
    
    Returns:
        Branch dict in BRANCHES format (trunk configuration)
    """
    clamp_radius = tree_config.clamp_radius
    GLOBAL_SCALE = tree_config.GLOBAL_SCALE
    MIN_LINK_RADIUS_WORLD = tree_config.MIN_LINK_RADIUS_WORLD
    
    # Calculate averages
    avg_radius = sum(i["width_m"] / 2.0 for i in internodes) / len(internodes)
    avg_height = sum(i["length"] for i in internodes) / len(internodes)
    
    # Apply radius clamping
    radius_final, was_clamped = clamp_radius(avg_radius)
    
    if was_clamped:
        radius_world_original = avg_radius * GLOBAL_SCALE
        radius_world_clamped = radius_final * GLOBAL_SCALE
        print(f"[WARNING] Trunk radius clamped: {radius_world_original:.4f}m → {radius_world_clamped:.4f}m "
              f"(min {MIN_LINK_RADIUS_WORLD}m at scale {GLOBAL_SCALE})")
    
    # Create BRANCHES format dict
    branch = {
        "id": "trunk",
        "system": "vegetative",
        "visual_axis_id": "trunk",
        "visual_segments": [{
            "source_id": "trunk",
            "length": avg_height * len(internodes),
            "radius": radius_final,
        }],
        "parent": None,
        "attach_link": None,
        "n_links": len(internodes),
        "radius": radius_final,
        "height": avg_height,
        "tilt": 0.0,
        "rot": 0.0,
        "joint_type": (
            "fixed" if tree_config.PhysicsRuntimeConfig.RIGID_TRUNK else "d6"
        ),
    }
    
    return branch


def save_branches_json(
    branches: List[Dict],
    day: int,
    output_dir: str,
    internodes: List[Dict],
    csv_filename: str,
    terminal_bodies: List[Dict] = None,
    generation_settings: Dict[str, bool] = None,
    resolution_changes: List[Dict] = None,
) -> str:
    """
    Save BRANCHES configuration to JSON with metadata.
    
    Args:
        branches: List of branch dicts in BRANCHES format
        day: Simulation day
        output_dir: Base output directory (e.g., "output")
        internodes: Original internode data for metadata
        csv_filename: Source CSV filename
    
    Returns:
        Absolute path to saved JSON file
    """
    # Create output directory
    day_dir = Path(output_dir) / f"day_{day}"
    day_dir.mkdir(parents=True, exist_ok=True)
    
    output_path = day_dir / f"branches_v2_day_{day}.json"
    
    # Build JSON structure
    data = {
        "metadata": {
            "day": day,
            "plant_id": 1,  # Fixed to 1 as per requirements
            "generated_at": datetime.now().isoformat(),
            "source_csv": csv_filename,
            "n_branches": len(branches),
            "total_links": sum(b["n_links"] for b in branches),
            "d6_joints": _count_d6_joints(branches),
            "n_terminal_bodies": len(terminal_bodies or []),
            "global_scale": tree_config.GLOBAL_SCALE,
            "min_radius_world_m": tree_config.MIN_LINK_RADIUS_WORLD,
            "physics_runtime": {
                "rigid_trunk": tree_config.PhysicsRuntimeConfig.RIGID_TRUNK,
                "physics_hz": tree_config.PhysicsRuntimeConfig.PHYSICS_HZ,
                "solver_position_iterations": tree_config.PhysicsRuntimeConfig.SOLVER_POSITION_ITERATIONS,
                "solver_velocity_iterations": tree_config.PhysicsRuntimeConfig.SOLVER_VELOCITY_ITERATIONS,
                "terminal_body_solver_position_iterations": tree_config.PhysicsRuntimeConfig.TERMINAL_BODY_SOLVER_POSITION_ITERATIONS,
                "terminal_body_solver_velocity_iterations": tree_config.PhysicsRuntimeConfig.TERMINAL_BODY_SOLVER_VELOCITY_ITERATIONS,
                "enable_gpu_dynamics": tree_config.PhysicsRuntimeConfig.ENABLE_GPU_DYNAMICS,
            },
            "branch_resolution": {
                "max_links_per_branch": tree_config.BranchResolutionConfig.MAX_LINKS_PER_BRANCH,
                "capped_branch_count": len(resolution_changes or []),
            },
            "organ_generation": generation_settings or {},
        },
        "branches": branches
    }

    if terminal_bodies:
        data["terminal_bodies"] = terminal_bodies
    
    # Save with pretty formatting
    with open(output_path, "w") as f:
        json.dump(data, f, indent=2)
    
    return str(output_path.absolute())


def _append_leaf_system(
    leaf: Dict,
    parent_branch_id: str,
    generation_settings: Dict[str, bool],
    all_branches: List[Dict],
    terminal_bodies: List[Dict],
    leaf_to_petiole_rachis_branches,
    create_lateral_petiolules,
    create_terminal_petiolule,
) -> None:
    """Append one complete leaf system while preserving authoring order."""
    petiole_rachis = leaf_to_petiole_rachis_branches(
        leaf,
        parent_branch_id,
        include_rachis=generation_settings["leaf_rachis"],
    )
    all_branches.extend(petiole_rachis)

    if len(petiole_rachis) <= 1 or not generation_settings["petiolules"]:
        return

    rachis_branch = petiole_rachis[1]
    petiole_radius = petiole_rachis[0]["radius"]
    lateral_branches, lateral_bodies = create_lateral_petiolules(
        leaf,
        rachis_branch["id"],
        petiole_radius,
    )
    all_branches.extend(lateral_branches)
    terminal_bodies.extend(lateral_bodies)

    terminal_branch, terminal_body = create_terminal_petiolule(
        rachis_branch["id"],
        rachis_branch["n_links"],
        petiole_radius,
        leaf,
        visual_axis_id=rachis_branch.get("visual_axis_id"),
    )
    all_branches.append(terminal_branch)
    terminal_bodies.append(terminal_body)


def parse_csv_to_branches(
    day: int,
    plant_id: int = 1,
    profile: dict = None,
    include_terminal_bodies: bool = False,
    save_json: bool = True,
    csv_path: str | Path | None = None,
) -> Tuple[List[Dict], str]:
    """
    Complete pipeline: CSV → internodes + leaves → BRANCHES → JSON.
    
    Args:
        day: Simulation day
        plant_id: Plant identifier (default: 1)
        profile: Cultivar profile dict (default: None = load tomato default)
        include_terminal_bodies: If True, return terminal rigid body definitions
        save_json: If True, save the parsed configuration JSON
        csv_path: Optional explicit CSV input. The legacy repository path is
            retained when omitted.
    
    Returns:
        Tuple (branches_list, json_path) by default, or
        Tuple (branches_list, terminal_bodies, json_path) when include_terminal_bodies=True:
            branches_list: List of branch dicts in BRANCHES format (trunk + leaves)
            terminal_bodies: Optional tomato/terminal body definitions
            json_path: Absolute path to saved JSON file
    
    Raises:
        FileNotFoundError: If CSV file doesn't exist
        ValueError: If no trunk internodes found
    """
    # Load default tomato profile if none provided
    if profile is None:
        from exporterV2.legacy_csv.tomato_default import TOMATO_PROFILE
        profile = TOMATO_PROFILE
    generation_settings = _effective_generation_settings(profile)
    _print_generation_settings(generation_settings)
    # Import leaf_builder - handle both package and standalone execution
    try:
        from .leaf_builder import (
            leaf_to_petiole_rachis_branches,
            create_lateral_petiolules,
            create_terminal_petiolule,
        )
        from .truss_builder import truss_to_complete_config
    except ImportError:
        # Standalone execution
        import sys
        import os
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from leaf_builder import (
            leaf_to_petiole_rachis_branches,
            create_lateral_petiolules,
            create_terminal_petiolule,
        )
        from truss_builder import truss_to_complete_config
    
    # Build paths
    script_dir = Path(__file__).parent
    # From legacy_csv/ → exporterV2/ → src/ → project_root
    project_root = script_dir.parent.parent.parent
    if csv_path is None:
        csv_path = (
            project_root
            / "data"
            / "simulation_output"
            / "dynamic_output"
            / "graphs"
            / f"graph_day_{day}.csv"
        )
    else:
        csv_path = Path(csv_path).expanduser().resolve()
    output_dir = project_root / "output"
    dataframe = _read_csv_frame(str(csv_path))
    
    # Load trunk internodes
    print(f"[INFO] Loading trunk internodes...")
    internodes = load_trunk_internodes(
        str(csv_path), day, plant_id, _dataframe=dataframe
    )
    
    # Convert trunk to BRANCHES format
    trunk_branch = internodes_to_branch_config(internodes)
    all_branches = [trunk_branch]
    terminal_bodies = []
    
    # Load lateral branches (order=1)
    lateral_branches = []
    if generation_settings["lateral_branches"]:
        print(f"[INFO] Loading lateral branches...")
        lateral_branches = load_lateral_branches(
            str(csv_path),
            day,
            plant_id,
            profile=profile,
            _dataframe=dataframe,
        )
    else:
        print("[CONFIG] Skipping lateral branches and their descendants")
    
    if lateral_branches:
        print(f"[INFO] Processing {len(lateral_branches)} lateral branches...")
        lateral_branch_configs = lateral_branches_to_branch_config(lateral_branches, trunk_branch["id"], profile=profile)
        all_branches.extend(lateral_branch_configs)
        
        # Create mapping: (rank, organ_index) → branch_id for lateral branches
        lateral_branch_map = {
            (int(b["id"].split("_r")[1].split("_")[0]), 
             int(b["id"].split("_o")[1])): b["id"]
            for b in lateral_branch_configs
        }
    else:
        lateral_branch_map = {}
    
    # Load trunk leaves (order=0)
    trunk_leaves = []
    if generation_settings["trunk_leaves"]:
        print(f"[INFO] Loading trunk leaves...")
        trunk_leaves = load_leaves(
            str(csv_path), day, plant_id, order=0, _dataframe=dataframe
        )
    else:
        print("[CONFIG] Skipping trunk leaf systems")
    
    if trunk_leaves:
        print(f"[INFO] Processing {len(trunk_leaves)} trunk leaves...")
        for leaf in trunk_leaves:
            _append_leaf_system(
                leaf,
                trunk_branch["id"],
                generation_settings,
                all_branches,
                terminal_bodies,
                leaf_to_petiole_rachis_branches,
                create_lateral_petiolules,
                create_terminal_petiolule,
            )
    
    # Load lateral branch leaves (order=1)
    if lateral_branch_map and generation_settings["lateral_leaves"]:
        print(f"[INFO] Loading lateral branch leaves...")
        lateral_leaves = load_leaves(
            str(csv_path), day, plant_id, order=1, _dataframe=dataframe
        )
        
        if lateral_leaves:
            print(f"[INFO] Processing {len(lateral_leaves)} lateral branch leaves...")
            for leaf in lateral_leaves:
                # Find parent lateral branch: match by parent_rank AND organ_index
                # Leaf organ_index 0 → Branch organ_index 0
                # Leaf organ_index 1 → Branch organ_index 1
                parent_key = (leaf["parent_rank"], leaf["organ_index"])
                
                parent_branch_id = lateral_branch_map.get(parent_key)
                
                if not parent_branch_id:
                    print(f"[WARNING] Could not find parent lateral branch for leaf rank={leaf['rank']}, "
                          f"organ_index={leaf['organ_index']}, parent_rank={leaf['parent_rank']}")
                    continue
                
                _append_leaf_system(
                    leaf,
                    parent_branch_id,
                    generation_settings,
                    all_branches,
                    terminal_bodies,
                    leaf_to_petiole_rachis_branches,
                    create_lateral_petiolules,
                    create_terminal_petiolule,
                )
    elif lateral_branch_map:
        print("[CONFIG] Skipping lateral leaf systems")

    # Load trunk trusses (order=0). Truss morphology is adapter-specific to GroIMP.
    trunk_trusses = []
    if generation_settings["trusses"]:
        print(f"[INFO] Loading trunk trusses...")
        trunk_trusses = load_trusses(
            str(csv_path), day, plant_id, order=0, _dataframe=dataframe
        )
    else:
        print("[CONFIG] Skipping trusses and their descendants")

    if trunk_trusses:
        print(f"[INFO] Processing {len(trunk_trusses)} trunk trusses...")
        max_trunk_link = trunk_branch["n_links"]

        for truss in trunk_trusses:
            fruit_radii = [r for r in truss["fruit_radii"] if r > 0.0]
            if not fruit_radii:
                print(
                    f"[WARNING] Skipping truss rank={truss['rank']} organ_index={truss['organ_index']} "
                    f"because it has no positive fruit radii"
                )
                continue

            ripening_dd = truss.get("fruit_ripening_dd", 0.0)
            maturation = []
            for age_dd in truss.get("fruit_age_dd", []):
                if ripening_dd > 0.0:
                    maturation.append(max(0.0, min(age_dd / ripening_dd, 1.0)))
                else:
                    maturation.append(0.0)

            n_fruits = min(truss["fruit_nr"], len(fruit_radii))
            lateral_pairs = n_fruits // 2
            has_terminal = (n_fruits % 2) == 1
            rachis_links = max(lateral_pairs + int(has_terminal), 1)

            truss_dict = {
                "n_fruits": n_fruits,
                "parent_rank": truss["parent_rank"],
                "rachis_length": TRUSS_GEOMETRY.RACHIS_SEGMENT_LENGTH * rachis_links,
                "rachis_radius": TRUSS_GEOMETRY.RACHIS_RADIUS,
                "pedicel_length": TRUSS_GEOMETRY.PEDICEL_LENGTH,
                "pedicel_radius": TRUSS_GEOMETRY.PEDICEL_RADIUS,
                "tilt_deg": _truss_tilt_from_groimp_angle(truss["truss_angle"], n_fruits),
                "azimuth_deg": (truss["rank"] * profile.get("phyllotaxis_deg", 137.5)) % 360.0,
                "tomato_radii": fruit_radii,
                "maturation": maturation,
            }

            truss_branches, tomatoes = truss_to_complete_config(
                truss_dict,
                parent_trunk_id=trunk_branch["id"],
                rank=truss["rank"],
                organ_index=truss["organ_index"],
                include_pedicels=generation_settings["pedicels"],
                include_tomatoes=generation_settings["tomatoes"],
            )

            if truss_branches:
                rachis = truss_branches[0]
                if rachis["attach_link"] > max_trunk_link:
                    print(
                        f"[WARNING] Truss '{rachis['id']}' attach_link={rachis['attach_link']} "
                        f"exceeds trunk links ({max_trunk_link}); clamping to {max_trunk_link}"
                    )
                    rachis["attach_link"] = max_trunk_link

            all_branches.extend(truss_branches)
            terminal_bodies.extend(_normalize_terminal_bodies(tomatoes))
    
    all_branches = _filter_invalid_branches(all_branches)
    all_branches, resolution_changes = tree_config.limit_branch_resolution(all_branches)
    print(
        f"[CONFIG] Branch resolution: max="
        f"{tree_config.BranchResolutionConfig.MAX_LINKS_PER_BRANCH}, "
        f"capped={len(resolution_changes)}"
    )
    terminal_bodies = _filter_terminal_bodies(terminal_bodies, all_branches)

    # Calculate stats
    total_links = sum(b["n_links"] for b in all_branches)
    d6_joints = _count_d6_joints(all_branches)
    print(
        f"[INFO] Total branches: {len(all_branches)}, total links: {total_links}, "
        f"D6 joints: {d6_joints}, terminal bodies: {len(terminal_bodies)}"
    )
    
    if d6_joints > tree_config.MAX_N_JOINTS:
        print(f"[WARNING] D6 joints ({d6_joints}) exceed PhysX budget ({tree_config.MAX_N_JOINTS})")
        print(f"[WARNING] Run with --optimize or reduce dynamic branch complexity")
    
    # Save to JSON
    json_path = None
    if save_json:
        json_path = save_branches_json(
            all_branches,
            day,
            str(output_dir),
            internodes,
            f"graph_day_{day}.csv",
            terminal_bodies=terminal_bodies,
            generation_settings=generation_settings,
            resolution_changes=resolution_changes,
        )

    if include_terminal_bodies:
        return all_branches, terminal_bodies, json_path

    return all_branches, json_path



# ==============================================================================
# STANDALONE TEST
# ==============================================================================

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="CSV to BRANCHES Parser")
    parser.add_argument("--day", type=int, required=True, help="Simulation day")
    parser.add_argument("--plant-id", type=int, default=1, help="Plant ID (default: 1)")
    args = parser.parse_args()
    
    print("=" * 80)
    print("  CSV to BRANCHES Parser")
    print("=" * 80)
    
    branches, json_path = parse_csv_to_branches(args.day, args.plant_id)
    
    print(f"\nDay: {args.day}")
    print(f"Plant ID: {args.plant_id}")
    print(f"Trunk links: {branches[0]['n_links']}")
    print(f"Average radius: {branches[0]['radius']:.4f}m (pre-scale)")
    print(f"Average height: {branches[0]['height']:.4f}m (pre-scale)")
    print(f"JSON saved: {json_path}")
    print("=" * 80)
