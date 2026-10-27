"""
Tomato cultivar configuration for groIMP CSV parser.

This profile contains all tomato-specific filtering and orientation logic.
Other cultivars would have different values here.
"""

TOMATO_PROFILE = {
    "name": "Tomato Default",
    "description": "Standard tomato plant with opposite pair branching",
    
    # Lateral branches configuration
    "lateral_branches": {
        "enabled": True,
        "organ_indices": [0, 1],           # Keep only opposite pairs (organ_index 0 and 1)
        "tilt_deg": 45.0,                  # Fixed tilt from trunk (vertical = 0°)
        "rot_base_deg": [0.0, 180.0],      # Base rotation: organ_0=0°, organ_1=180°
        "rot_jitter_deg": 45.0,            # Random jitter: ±45° from base
        "min_angle_separation_deg": 60.0,  # Anti-collision: min angle between branches
    },
    
    # Trunk leaves configuration
    "trunk_leaves": {
        "enabled": True,
        "filter_strategy": "opposite_pairs_180deg",  # Keep leaves forming 180° pairs
        "pair_angle_threshold_deg": 1e-6,            # Tolerance for 180° detection
        "phyllotaxis_deg": 137.5,                    # Golden angle fallback (if no ccw_orientation)
        "clone_detection_threshold_deg": 1e-6,       # Detect duplicate leaves at same position
    },
    
    # Lateral branch leaves configuration
    "lateral_leaves": {
        "enabled": True,
        "organ_indices": [0, 1],           # Same as lateral branches (opposite pairs)
        "clone_missing": True,              # Clone missing leaf if only 1 in pair exists
        "tilt_deg": 35.0,                   # More coaxial with 45° branch (upward orientation)
        "rot_range_deg": (-90.0, 90.0),     # Random range relative to branch axis
        "rot_seed_formula": "rank * 1000 + organ_index",  # Deterministic randomness
    },

    # General CSV parsing
    "csv": {
        "day_column": "day",
        "plant_id_column": "plant_id",
        "organ_class_column": "organ_class",
        "order_column": "order",
    },
}
