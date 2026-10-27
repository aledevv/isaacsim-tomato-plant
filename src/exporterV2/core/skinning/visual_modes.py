"""Compatibility exports for the non-UsdSkel visual modes."""

from .visual_rigid import author_rigid_visual_axis
from .visual_segmented import author_segmented_visual_axis
from .visual_static import author_static_visual_axis


__all__ = (
    "author_rigid_visual_axis",
    "author_segmented_visual_axis",
    "author_static_visual_axis",
)
