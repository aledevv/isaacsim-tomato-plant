"""Structural checks for ``plant_skeleton/1.0``.

``validate_skeleton`` raises one error listing every problem it found, so a
converter author can fix them all in one pass.
"""

from __future__ import annotations

import math
from typing import Iterable

from .geometry import frame_error, norm, segment_length, sub
from .model import (
    FRUIT_HOST_KINDS,
    LEAFLET_HOST_KINDS,
    LEAFLET_SIDES,
    NODE_KINDS,
    NODE_PARENT_KINDS,
    SKELETON_SCHEMA_VERSION,
    PlantSkeleton,
)

# A child normally starts where its parent ends. Larger gaps are reported as
# warnings because the joint is placed at the parent's end anyway.
ATTACHMENT_WARNING_DISTANCE_M = 1e-3


class SkeletonValidationError(ValueError):
    def __init__(self, errors: Iterable[str]):
        self.errors = tuple(errors)
        lines = "\n  - ".join(self.errors)
        super().__init__(f"invalid plant skeleton:\n  - {lines}")


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _duplicates(values: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    repeated: set[str] = set()
    for value in values:
        if value in seen:
            repeated.add(value)
        seen.add(value)
    return sorted(repeated)


def skeleton_errors(skeleton: PlantSkeleton) -> list[str]:
    errors: list[str] = []
    if skeleton.schema_version != SKELETON_SCHEMA_VERSION:
        errors.append(
            f"schema_version must be {SKELETON_SCHEMA_VERSION!r}, got {skeleton.schema_version!r}"
        )
    if skeleton.metadata.plant_id < 1:
        errors.append("metadata.plant_id must be 1 or greater")
    if not skeleton.nodes:
        return [*errors, "the skeleton has no nodes"]

    all_ids = [node.id for node in skeleton.nodes]
    all_ids += [leaflet.id for leaflet in skeleton.leaflets]
    all_ids += [fruit.id for fruit in skeleton.fruits]
    for duplicate in _duplicates(all_ids):
        errors.append(f"id {duplicate!r} is used more than once")
    for item_id in all_ids:
        if not item_id or not item_id.strip():
            errors.append("ids must be non-empty strings")

    nodes = {node.id: node for node in skeleton.nodes}
    roots = [node.id for node in skeleton.nodes if node.parent is None]
    if len(roots) != 1:
        errors.append(f"exactly one node must have parent null, found {roots}")

    for node in skeleton.nodes:
        where = f"node {node.id!r}"
        if node.kind not in NODE_PARENT_KINDS:
            errors.append(f"{where} has unknown kind {node.kind!r}; allowed: {list(NODE_KINDS)}")
            continue
        parent = nodes.get(node.parent) if node.parent is not None else None
        if node.parent is not None and parent is None:
            errors.append(f"{where} has missing parent {node.parent!r}")
        parent_kind = parent.kind if parent is not None else None
        if (node.parent is None or parent is not None) and parent_kind not in NODE_PARENT_KINDS[node.kind]:
            allowed = sorted(str(kind) for kind in NODE_PARENT_KINDS[node.kind])
            errors.append(
                f"{where} of kind {node.kind!r} cannot hang from {parent_kind!r}; allowed parents: {allowed}"
            )
        if not _finite((*node.start, *node.end, node.radius)):
            errors.append(f"{where} has non-finite numbers")
            continue
        if node.radius <= 0.0:
            errors.append(f"{where} radius must be positive")
        if segment_length(node.start, node.end) <= 1e-9:
            errors.append(f"{where} has zero length (start == end)")
            continue
        if node.frame is not None:
            problem = frame_error(node.frame, sub(node.end, node.start))
            if problem:
                errors.append(f"{where} frame: {problem}")

    # Cycles and reachability from the root.
    for node in skeleton.nodes:
        seen = {node.id}
        current = node.parent
        while current is not None and current in nodes:
            if current in seen:
                errors.append(f"node {node.id!r} is part of a parent cycle")
                break
            seen.add(current)
            current = nodes[current].parent

    stem_children: dict[str | None, list[str]] = {}
    for node in skeleton.nodes:
        if node.kind == "stem":
            stem_children.setdefault(node.parent, []).append(node.id)
    for parent_id, children in stem_children.items():
        if parent_id is not None and len(children) > 1:
            errors.append(
                f"the stem must be one chain, but node {parent_id!r} has stem children {children}; "
                "use kind 'branch' for side shoots"
            )

    for leaflet in skeleton.leaflets:
        where = f"leaflet {leaflet.id!r}"
        host = nodes.get(leaflet.node)
        if host is None:
            errors.append(f"{where} references missing node {leaflet.node!r}")
        elif host.kind not in LEAFLET_HOST_KINDS:
            errors.append(f"{where} must hang from a petiole or leaf_rachis, not {host.kind!r}")
        if leaflet.side not in LEAFLET_SIDES:
            errors.append(f"{where} side must be one of {list(LEAFLET_SIDES)}")
        if not _finite((*leaflet.start, *leaflet.end, leaflet.radius, leaflet.mass_kg)):
            errors.append(f"{where} has non-finite numbers")
            continue
        if leaflet.radius <= 0.0 or segment_length(leaflet.start, leaflet.end) <= 1e-9:
            errors.append(f"{where} needs positive radius and length")
            continue
        if leaflet.mass_kg < 0.0:
            errors.append(f"{where} mass_kg cannot be negative")
        if leaflet.frame is not None:
            problem = frame_error(leaflet.frame, sub(leaflet.end, leaflet.start))
            if problem:
                errors.append(f"{where} frame: {problem}")

    fruit_hosts: dict[str, str] = {}
    for fruit in skeleton.fruits:
        where = f"fruit {fruit.id!r}"
        host = nodes.get(fruit.node)
        if host is None:
            errors.append(f"{where} references missing node {fruit.node!r}")
        elif host.kind not in FRUIT_HOST_KINDS:
            errors.append(f"{where} must hang from a pedicel, not {host.kind!r}")
        if fruit.node in fruit_hosts:
            errors.append(f"pedicel {fruit.node!r} carries two fruits ({fruit_hosts[fruit.node]!r}, {fruit.id!r})")
        fruit_hosts[fruit.node] = fruit.id
        if not math.isfinite(fruit.radius) or fruit.radius <= 0.0:
            errors.append(f"{where} radius must be positive")
        if not 0.0 <= fruit.ripeness <= 1.0:
            errors.append(f"{where} ripeness must be between 0 and 1")
    return errors


def skeleton_warnings(skeleton: PlantSkeleton) -> list[str]:
    """Soft issues that still export, such as children far from their parent."""

    nodes = {node.id: node for node in skeleton.nodes}
    warnings = []
    for node in skeleton.nodes:
        parent = nodes.get(node.parent) if node.parent else None
        if parent is None:
            continue
        gap = norm(sub(node.start, parent.end))
        if gap > ATTACHMENT_WARNING_DISTANCE_M:
            warnings.append(
                f"node {node.id!r} starts {gap * 1000:.1f} mm away from the end of its parent {parent.id!r}"
            )
    return warnings


def validate_skeleton(skeleton: PlantSkeleton) -> PlantSkeleton:
    errors = skeleton_errors(skeleton)
    if errors:
        raise SkeletonValidationError(errors)
    return skeleton
