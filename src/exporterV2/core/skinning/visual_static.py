"""Static smooth-mesh visual mode for vegetative axes."""

from pxr import UsdGeom

from ..usd.materials import get_or_create_tomato_stem_material
from .mesh import _axis_color, author_plain_mesh, build_axis_tube_data
from .model import VisualAxisData


def author_static_visual_axis(stage, axis: VisualAxisData) -> None:
    """Author a world-space smooth tube without a UsdSkel runtime."""
    UsdGeom.Xform.Define(stage, axis.visual_root_path)
    points, face_counts, face_indices, _, _ = build_axis_tube_data(axis)
    author_plain_mesh(
        stage,
        f"{axis.visual_root_path}/StaticMesh",
        points,
        face_counts,
        face_indices,
        _axis_color(axis),
        material=get_or_create_tomato_stem_material(stage),
    )
