"""Data model of ``plant_skeleton/1.0``: one plant as a graph of segments.

Every node is one straight segment of the plant (an internode, a petiole, a
rachis segment or a pedicel). Leaflets and fruits hang from nodes. Positions
are in metres, Z is up and the plant base sits at the origin.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, TypeAlias


SKELETON_SCHEMA_VERSION = "plant_skeleton/1.0"

# Allowed node kinds and the parent kinds each one may hang from.
NODE_PARENT_KINDS: dict[str, frozenset[str | None]] = {
    "stem": frozenset({None, "stem"}),
    "branch": frozenset({"stem", "branch"}),
    "petiole": frozenset({"stem", "branch"}),
    "leaf_rachis": frozenset({"petiole", "leaf_rachis"}),
    "truss_rachis": frozenset({"stem", "branch", "truss_rachis"}),
    "pedicel": frozenset({"truss_rachis"}),
}
NODE_KINDS = tuple(NODE_PARENT_KINDS)
LEAFLET_SIDES = ("left", "right", "terminal")
LEAFLET_HOST_KINDS = frozenset({"petiole", "leaf_rachis"})
FRUIT_HOST_KINDS = frozenset({"pedicel"})

Vector3: TypeAlias = tuple[float, float, float]
Matrix3: TypeAlias = tuple[
    tuple[float, float, float],
    tuple[float, float, float],
    tuple[float, float, float],
]


@dataclass(frozen=True)
class SkeletonMetadata:
    plant_id: int = 1
    source: str = "unknown"
    description: str = ""
    time: float | None = None


@dataclass(frozen=True)
class SkeletonNode:
    """One straight segment.

    ``frame`` is optional: a 3x3 rotation written as rows whose columns are
    the local X (left), Y (up) and Z (along the segment) axes in world space.
    When it is missing the exporter derives one from the direction.
    ``axis`` optionally groups consecutive nodes into one branch; when it is
    missing the exporter follows single same-kind children.
    ``organ`` optionally names the biological organ (for example a leaf) the
    node belongs to; it is only used for bookkeeping in the USD.
    """

    id: str
    parent: str | None
    kind: str
    start: Vector3
    end: Vector3
    radius: float
    frame: Matrix3 | None = None
    axis: str | None = None
    organ: str | None = None


@dataclass(frozen=True)
class SkeletonLeaflet:
    """A leaflet: a small stalk (petiolule) from ``start`` to ``end`` with a
    blade generated at its end. ``node`` is the petiole or leaf rachis it
    grows from. ``inclination_deg`` is the stalk angle used by the default
    "v2-aesthetic" pose; ``mass_kg`` is added to the host body."""

    id: str
    node: str
    side: str
    start: Vector3
    end: Vector3
    radius: float
    frame: Matrix3 | None = None
    inclination_deg: float | None = None
    mass_kg: float = 0.0


@dataclass(frozen=True)
class SkeletonFruit:
    """A tomato at the tip of a pedicel. ``ripeness`` goes from 0 (green) to
    1 (red). ``center`` is only used by the "canonical" pose."""

    id: str
    node: str
    radius: float
    ripeness: float = 0.0
    center: Vector3 | None = None


@dataclass(frozen=True)
class PlantSkeleton:
    nodes: tuple[SkeletonNode, ...]
    leaflets: tuple[SkeletonLeaflet, ...] = ()
    fruits: tuple[SkeletonFruit, ...] = ()
    metadata: SkeletonMetadata = field(default_factory=SkeletonMetadata)
    schema_version: str = SKELETON_SCHEMA_VERSION

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def node(self, node_id: str) -> SkeletonNode:
        for node in self.nodes:
            if node.id == node_id:
                return node
        raise KeyError(node_id)

    @property
    def root(self) -> SkeletonNode:
        return next(node for node in self.nodes if node.parent is None)
