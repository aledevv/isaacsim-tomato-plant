"""
Simple plant profile - for testing and validation.

A minimal plant with:
- No lateral branches
- All trunk leaves (no filtering)
- Simple phyllotaxis-based leaf orientation

Use this to test the generic pipeline without cultivar-specific logic.
"""

SIMPLE_PLANT_PROFILE = {
    "name": "Simple Plant",
    "description": "Single trunk with all leaves, no lateral branches",
    
    # No lateral branches
    "lateral_branches": {
        "enabled": False,
    },
    
    # Keep all trunk leaves, no filtering
    "trunk_leaves": {
        "enabled": True,
        "filter_strategy": "keep_all",  # No opposite pair filtering
        "phyllotaxis_deg": 137.5,        # Use phyllotaxis for all
    },
    
    # No lateral leaves (no lateral branches)
    "lateral_leaves": {
        "enabled": False,
    },
}
