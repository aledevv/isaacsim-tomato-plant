"""Optional smooth skinning backend for ExporterV2 vegetative branches."""

from .adapter import branch_system, partition_branches, resolve_vegetative_graph
from .axis import build_visual_axes
from .builder import build_skinned_vegetative_structure
from .model import BranchData, BranchSpec, VisualAxisData, VisualProfile, VisualSegment
from .runtime import SkinningRuntime

__all__ = [
    "SkinningRuntime",
    "BranchData",
    "BranchSpec",
    "VisualAxisData",
    "VisualProfile",
    "VisualSegment",
    "branch_system",
    "build_skinned_vegetative_structure",
    "build_visual_axes",
    "partition_branches",
    "resolve_vegetative_graph",
]
