"""
exporterV2 - plant skeleton -> articulated USDA for Isaac Sim

- exporter.py / cli.py: the pipeline (``uv run python -m exporterV2``)
- skeleton_branches.py: skeleton nodes -> branch chains
- visuals/: where every mesh generator is (stems, leaves, trusses, fruits)
- leaf_shapes/: baked leaflet assets and the live leaf-shape worker
- core/: generic rigid-link / joint / mesh builder
"""

# Re-export core functionality for convenience (tree_config only, no pxr)
from .core import (
    BRANCHES,
    GLOBAL_SCALE,
    MAX_N_JOINTS,
    MIN_LINK_RADIUS_WORLD,
    TrussGeometryConfig,
    print_tree_summary,
    validate_branches,
)

# Lazy imports for USD/physics (require pxr)
def __getattr__(name):
    if name == "build_stage":
        from .core.usd import build_stage
        return build_stage
    elif name == "apply_physx_scene_settings":
        from .core.physics import apply_physx_scene_settings
        return apply_physx_scene_settings
    elif name == "export_plant":
        from .exporter import export_plant
        return export_plant
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")

__all__ = [
    "build_stage",
    "BRANCHES",
    "GLOBAL_SCALE",
    "MAX_N_JOINTS",
    "MIN_LINK_RADIUS_WORLD",
    "TrussGeometryConfig",
    "print_tree_summary",
    "validate_branches",
    "apply_physx_scene_settings",
    "export_plant",
]

__version__ = "2.2.0"
