"""Single-link rigid visual mode for vegetative axes."""

from pxr import Gf

from ..usd.materials import get_or_create_tomato_stem_material
from .mesh import _axis_color, author_plain_mesh, build_axis_tube_data, link_rest_world
from .model import VisualAxisData


def author_rigid_visual_axis(stage, axis: VisualAxisData) -> None:
    """Attach a one-bone smooth tube directly below its PhysX rigid link."""
    if len(axis.link_paths) != 1:
        raise ValueError(
            f"Rigid visual axis '{axis.axis_id}' must have exactly one physics link"
        )

    world_points, face_counts, face_indices, _, _ = build_axis_tube_data(axis)
    world_to_link = link_rest_world(axis, 0).GetInverse()
    local_points = [
        Gf.Vec3f(*world_to_link.Transform(Gf.Vec3d(*point)))
        for point in world_points
    ]
    author_plain_mesh(
        stage,
        f"{axis.link_paths[0]}/VisualMesh",
        local_points,
        face_counts,
        face_indices,
        _axis_color(axis),
        material=get_or_create_tomato_stem_material(stage),
    )
