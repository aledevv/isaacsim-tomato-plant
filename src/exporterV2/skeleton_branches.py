"""Turn a ``plant_skeleton/1.0`` graph into the branch list the USD core builds.

The USD core (``core/``) works with *branches*: chains of rigid links joined
by D6 or fixed joints. This module groups skeleton nodes into those chains,
attaches the physics values from ``config/physics.yaml`` and the visual
choices from ``config/visuals.yaml``, and turns leaflets and fruits into
visual records. It never reads source-specific data; whatever produced the
skeleton handled that.

How nodes become branches:

* every ``stem`` node forms the single ``trunk`` chain;
* nodes sharing an explicit ``axis`` form one chain;
* otherwise a node continues its parent's chain when it has the same kind and
  is the parent's only child of that kind;
* a branch starts at the end of its parent node (``attach_link``).
"""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
import re
from typing import Any

from plant_skeleton import PlantSkeleton, SkeletonNode, validate_skeleton
from plant_skeleton.geometry import (
    closest_fraction,
    derive_frame,
    frame_columns,
    segment_length,
    sub,
)

from .config import PhysicsConfig, VisualsConfig


# Internal branch kinds used by the core and the joint-budget optimizer.
BRANCH_KIND = {
    "stem": "stem",
    "branch": "lateral_branch",
    "petiole": "leaf_petiole",
    "leaf_rachis": "leaf_rachis",
    "truss_rachis": "truss_rachis",
    "pedicel": "pedicel",
}
# ``autotom:role`` written on every rigid link.
LINK_ROLE = {
    "stem": "internode",
    "branch": "internode",
    "petiole": "petiole",
    "leaf_rachis": "leaf_rachis",
    "truss_rachis": "truss_rachis",
    "pedicel": "pedicel",
}
LEAFLET_ROLE = {"left": "petiolule_left", "right": "petiolule_right", "terminal": "rachis_terminal"}

# Mesh sampling of the organic branch generator (core/skinning).
VISUAL_PROFILES = {
    "realistic": {"radial_segments": 20, "axial_spacing_m": 0.005, "radius_transition_samples": 9},
    "performance": {"radial_segments": 12, "axial_spacing_m": 0.012, "radius_transition_samples": 5},
}

# Historical V2 look of pedicels: tilted away from the truss, alternating
# left/right; with an odd count the last one points straight on.
PEDICEL_TILT_DEG = 56.0


class SkeletonBranchesError(ValueError):
    pass


@dataclass(frozen=True)
class BranchPlan:
    """Everything the exporter needs to author one plant."""

    branches: tuple[dict[str, Any], ...]
    rigid_leaf_visuals: tuple[dict[str, Any], ...]
    terminal_bodies: tuple[dict[str, Any], ...]
    attachment_map: tuple[dict[str, Any], ...]
    pose: str
    visual_quality: str
    fruit_mode: str


def _safe(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_]", "_", value)


def _matmul3(left, right) -> list[list[float]]:
    return [[sum(float(left[r][k]) * float(right[k][c]) for k in range(3)) for c in range(3)] for r in range(3)]


def _rest_frame(rotation, start) -> list[list[float]]:
    """4x4 row-major frame: rotation columns X, Y, Z plus the start point."""

    rows = [[float(rotation[r][c]) for c in range(3)] + [float(start[r])] for r in range(3)]
    rows.append([0.0, 0.0, 0.0, 1.0])
    return rows


def appendage_rotation(host_rotation, *, tilt_deg: float, rotation_deg: float) -> list[list[float]]:
    """Historical V2 recipe: spin around the host axis, then tilt away from it."""

    tilt = math.radians(float(tilt_deg))
    spin = math.radians(float(rotation_deg))
    rotate_z = ((math.cos(spin), -math.sin(spin), 0.0), (math.sin(spin), math.cos(spin), 0.0), (0.0, 0.0, 1.0))
    rotate_x = ((1.0, 0.0, 0.0), (0.0, math.cos(tilt), math.sin(tilt)), (0.0, -math.sin(tilt), math.cos(tilt)))
    return _matmul3(host_rotation, _matmul3(rotate_z, rotate_x))


class _Graph:
    def __init__(self, skeleton: PlantSkeleton):
        self.skeleton = skeleton
        self.nodes = {node.id: node for node in skeleton.nodes}
        self.order = {node.id: index for index, node in enumerate(skeleton.nodes)}
        self.children: dict[str, list[str]] = {node.id: [] for node in skeleton.nodes}
        for node in skeleton.nodes:
            if node.parent is not None:
                self.children[node.parent].append(node.id)
        self._frames: dict[str, tuple] = {}

    def frame(self, node_id: str):
        """The node's rotation, derived from its parent when not given."""

        if node_id in self._frames:
            return self._frames[node_id]
        chain = []
        current: str | None = node_id
        while current is not None and current not in self._frames:
            chain.append(current)
            current = self.nodes[current].parent
        for item in reversed(chain):
            node = self.nodes[item]
            if node.frame is not None:
                self._frames[item] = node.frame
            else:
                reference = None
                if node.parent is not None:
                    reference = frame_columns(self._frames[node.parent])[0]
                self._frames[item] = derive_frame(sub(node.end, node.start), reference)
        return self._frames[node_id]


def _chains(graph: _Graph) -> list[list[str]]:
    """Group nodes into chains, parents before children."""

    nodes = graph.nodes
    chain_of: dict[str, int] = {}
    chains: list[list[str]] = []

    def continues(node: SkeletonNode) -> bool:
        if node.parent is None:
            return False
        parent = nodes[node.parent]
        if node.kind == "stem":
            return True
        if node.axis is not None or parent.axis is not None:
            return node.axis is not None and node.axis == parent.axis
        if parent.kind != node.kind:
            return False
        same_kind = [child for child in graph.children[parent.id] if nodes[child].kind == node.kind and nodes[child].axis is None]
        return len(same_kind) == 1

    # Visit parent-first in file order.
    pending = [graph.skeleton.root.id]
    visited = []
    while pending:
        current = pending.pop(0)
        visited.append(current)
        pending[0:0] = sorted(graph.children[current], key=graph.order.__getitem__)
    for node_id in visited:
        node = nodes[node_id]
        if continues(node) and node.parent in chain_of:
            index = chain_of[node.parent]
            if chains[index][-1] != node.parent:
                raise SkeletonBranchesError(
                    f"axis {node.axis!r} branches at node {node.parent!r}; an axis must be one unbranched chain"
                )
            chains[index].append(node_id)
            chain_of[node_id] = index
        else:
            if node.axis is not None and any(nodes[chain[0]].axis == node.axis for chain in chains):
                raise SkeletonBranchesError(
                    f"axis {node.axis!r} is split: node {node_id!r} does not continue it"
                )
            chain_of[node_id] = len(chains)
            chains.append([node_id])
    # Keep the order of the input file; the core resolves parents in any order.
    return sorted(chains, key=lambda chain: graph.order[chain[0]])


def _branch_ids(graph: _Graph, chains: list[list[str]]) -> list[str]:
    """Readable, unique USD names.

    The joint-budget techniques (core/optimizations) recognise trusses by
    name: a truss is ``<base>_rachis`` and its pedicels ``<base>_pedicel_NN``.
    core/skinning only adds terminal forks to ids containing ``petiole``.
    """

    nodes = graph.nodes
    owner = {node_id: index for index, chain in enumerate(chains) for node_id in chain}
    bases: list[str] = []
    for chain in chains:
        first = nodes[chain[0]]
        name = _safe(first.id)
        if first.kind == "stem":
            base = "trunk"
        elif first.kind == "branch":
            base = f"Branch_{name}"
        elif first.kind == "petiole":
            base = "Leaf_" + name.removesuffix("_petiole")
        elif first.kind == "leaf_rachis" and nodes[first.parent].kind == "petiole":
            base = bases[owner[first.parent]]
        elif first.kind == "leaf_rachis":
            base = f"Leaf_{name}"
        elif first.kind == "truss_rachis":
            base = f"Truss_{name}"
        else:
            base = ""  # pedicels are named after their truss below
        bases.append(base)

    ids: list[str] = []
    used: set[str] = set()
    pedicel_count: dict[int, int] = {}
    for index, chain in enumerate(chains):
        first = nodes[chain[0]]
        suffix = {"petiole": "_petiole", "leaf_rachis": "_rachis", "truss_rachis": "_rachis"}.get(first.kind, "")
        candidate = bases[index] + suffix
        if first.kind == "pedicel":
            truss = owner[first.parent]
            pedicel_count[truss] = pedicel_count.get(truss, 0) + 1
            candidate = f"{bases[truss]}_pedicel_{pedicel_count[truss]:02d}"
        unique, number = candidate, 2
        while unique in used:
            unique, number = f"{candidate}_{number}", number + 1
        used.add(unique)
        ids.append(unique)
    return ids


def _visual_profile(visuals: VisualsConfig) -> dict[str, Any]:
    profile = dict(VISUAL_PROFILES[visuals.quality])
    if not visuals.stems["junction_bulge"]:
        profile["junction_bulge_amplitude"] = 0.0
    return profile


def lengthen_pedicels(skeleton: PlantSkeleton, scale: float) -> PlantSkeleton:
    """Pedicels ``scale`` times longer from their base; fruit centres follow."""

    if scale == 1.0:
        return skeleton
    shifts = {}
    nodes = []
    for node in skeleton.nodes:
        if node.kind == "pedicel":
            end = tuple(a + (b - a) * scale for a, b in zip(node.start, node.end))
            shifts[node.id] = tuple(e - b for e, b in zip(end, node.end))
            node = replace(node, end=end)
        nodes.append(node)
    fruits = tuple(
        replace(fruit, center=tuple(c + d for c, d in zip(fruit.center, shifts[fruit.node])))
        if fruit.center is not None and fruit.node in shifts
        else fruit
        for fruit in skeleton.fruits
    )
    return replace(skeleton, nodes=tuple(nodes), fruits=fruits)


def build_branch_plan(skeleton: PlantSkeleton, physics: PhysicsConfig, visuals: VisualsConfig) -> BranchPlan:
    """Group the skeleton into branches and attach physics and visuals."""

    validate_skeleton(skeleton)
    skeleton = lengthen_pedicels(skeleton, float(visuals.trusses["pedicel_length_scale"]))
    graph = _Graph(skeleton)
    nodes = graph.nodes
    chains = _chains(graph)
    branch_ids = _branch_ids(graph, chains)
    pose = visuals.pose
    profile = _visual_profile(visuals)

    location: dict[str, tuple[str, int]] = {}
    for branch_id, chain in zip(branch_ids, chains):
        for index, node_id in enumerate(chain, start=1):
            location[node_id] = (branch_id, index)

    # Pedicel v2 poses depend on their order along the truss.
    pedicel_pose: dict[str, tuple[float, float]] = {}
    pedicels_by_truss: dict[str, list[str]] = {}
    for node in skeleton.nodes:
        if node.kind == "pedicel":
            pedicels_by_truss.setdefault(location[node.parent][0], []).append(node.id)
    for pedicels in pedicels_by_truss.values():
        odd = len(pedicels) % 2 == 1
        for index, node_id in enumerate(pedicels):
            if odd and index == len(pedicels) - 1:
                pedicel_pose[node_id] = (0.0, 0.0)
            else:
                pedicel_pose[node_id] = (PEDICEL_TILT_DEG, 90.0 if index % 2 == 0 else 270.0)

    rest_frames: dict[str, list[list[float]]] = {}
    source_frames: dict[str, list[list[float]]] = {}
    for node in skeleton.nodes:
        source = _rest_frame(graph.frame(node.id), node.start)
        source_frames[node.id] = source
        rest_frames[node.id] = source
        if node.kind == "pedicel" and pose == "v2-aesthetic":
            tilt, spin = pedicel_pose[node.id]
            rotation = appendage_rotation(graph.frame(node.parent), tilt_deg=tilt, rotation_deg=spin)
            rest_frames[node.id] = _rest_frame(rotation, node.start)

    visual_axis: dict[str, str] = {}
    for branch_id, chain in zip(branch_ids, chains):
        first = nodes[chain[0]]
        visual_axis[branch_id] = branch_id
        if first.kind == "petiole":
            visual_axis[branch_id] = branch_id.removesuffix("_petiole") + "_axis"
        if first.kind == "leaf_rachis" and nodes[first.parent].kind == "petiole":
            parent_branch = location[first.parent][0]
            siblings = [child for child in graph.children[first.parent] if nodes[child].kind == "leaf_rachis"]
            if siblings and siblings[0] == first.id:
                visual_axis[branch_id] = visual_axis[parent_branch]

    branches = []
    attachment_map = []
    for branch_id, chain in zip(branch_ids, chains):
        first = nodes[chain[0]]
        settings = physics.segment(first.kind)
        specs = []
        for node_id in chain:
            node = nodes[node_id]
            specs.append(
                {
                    "id": _safe(node.id),
                    "canonical_node_id": node.id,
                    "canonical_organ_id": node.organ or node.id,
                    "canonical_axis_id": node.id,
                    "axis_role": LINK_ROLE[node.kind],
                    "represented_organ_ids": [node.organ or node.id],
                    "length": segment_length(node.start, node.end),
                    "radius": float(node.radius),
                    "rest_frame": rest_frames[node_id],
                    "source_rest_frame": source_frames[node_id],
                }
            )
        lengths = [spec["length"] for spec in specs]
        radii = [spec["radius"] for spec in specs]
        branch = {
            "id": branch_id,
            "kind": BRANCH_KIND[first.kind],
            "system": "vegetative",
            "visual_axis_id": visual_axis[branch_id],
            "visual_segments": [
                {"source_id": spec["id"], "length": spec["length"], "radius": spec["radius"]} for spec in specs
            ],
            "visual_profile": dict(profile),
            "parent": None,
            "attach_link": None,
            "n_links": len(specs),
            "radius": sum(radii) / len(radii),
            "height": sum(lengths) / len(lengths),
            "tilt": 0.0,
            "rot": 0.0,
            "joint_type": settings["joint"],
            "attachment_joint_type": settings["joint"],
            "young_modulus": float(settings["young_modulus"]),
            "damping_ratio": float(settings["damping_ratio"]),
            "density": float(settings["density"]),
            "bend_limit_deg": float(settings["bend_limit_deg"]),
            "drive_stiffness_scale": float(settings["drive_stiffness_scale"]),
            "link_specs": specs,
            "source_parent_node_id": first.parent,
        }
        if first.parent is not None:
            parent_branch, attach_link = location[first.parent]
            branch.update(parent=parent_branch, attach_link=attach_link, attach_frac=1.0)
            attachment_map.append(
                {
                    "kind": branch["kind"],
                    "branch_id": branch_id,
                    "root_node_id": first.id,
                    "parent_branch_id": parent_branch,
                    "parent_node_id": first.parent,
                    "attach_link": attach_link,
                    "link_node_ids": list(chain),
                }
            )
        if first.kind in ("truss_rachis", "pedicel"):
            component = "rachis" if first.kind == "truss_rachis" else "pedicel"
            branch["physics_profile"] = "truss"
            branch["truss_component"] = component
            thickness = float(visuals.trusses[f"{component}_thickness"])
            branch["visual_radius_scale"] = thickness
            for segment in branch["visual_segments"]:
                segment["radius"] *= thickness
            if component == "rachis":
                branch["visual_style"] = "historical_truss_rachis"
            elif visuals.pedicels["gravity_curve"]:
                branch["visual_style"] = "historical_pedicel"
        if first.kind == "petiole" and not visuals.stems["terminal_forks"]:
            branch["disable_terminal_fork"] = True
        branches.append(branch)

    leaf_visuals = _leaf_visuals(skeleton, graph, pose)
    terminal_bodies = _fruit_bodies(skeleton, rest_frames, physics, location, pose, float(visuals.fruits["size_scale"]))
    return BranchPlan(
        branches=tuple(branches),
        rigid_leaf_visuals=tuple(leaf_visuals),
        terminal_bodies=tuple(terminal_bodies),
        attachment_map=tuple(attachment_map),
        pose=pose,
        visual_quality=visuals.quality,
        fruit_mode=physics.fruits["mode"],
    )


def _leaf_visuals(skeleton: PlantSkeleton, graph: _Graph, pose: str) -> list[dict[str, Any]]:
    records = []
    for leaflet in skeleton.leaflets:
        host = graph.nodes[leaflet.node]
        _distance, fraction = closest_fraction(leaflet.start, host.start, host.end)
        host_rotation = graph.frame(host.id)
        source_rotation = leaflet.frame or derive_frame(sub(leaflet.end, leaflet.start), frame_columns(host_rotation)[0])
        source = _rest_frame(source_rotation, leaflet.start)
        rest = source
        if pose == "v2-aesthetic":
            if leaflet.side == "terminal":
                tilt, spin = 0.0, 0.0
            else:
                tilt = 90.0 if leaflet.inclination_deg is None else leaflet.inclination_deg
                spin = 90.0 if leaflet.side == "left" else 270.0
            rest = _rest_frame(appendage_rotation(host_rotation, tilt_deg=tilt, rotation_deg=spin), leaflet.start)
        role = LEAFLET_ROLE[leaflet.side]
        records.append(
            {
                "id": f"Leaflet_{_safe(leaflet.id)}",
                "node_id": host.id,
                "organ_id": host.organ or host.id,
                "axis_id": leaflet.id,
                "role": role,
                "leaflet_role": leaflet.side,
                "leaflet_id": leaflet.id,
                "host_axis_id": host.id,
                "host_fraction": float(fraction),
                "rest_frame": rest,
                "source_rest_frame": source,
                "appendage_pose_mode": pose,
                "length": segment_length(leaflet.start, leaflet.end),
                "radius": float(leaflet.radius),
                "aggregated_mass_kg": float(leaflet.mass_kg),
                "mass_source": "skeleton_leaflet_mass" if leaflet.mass_kg > 0.0 else "none",
            }
        )
    return records


def _fruit_bodies(
    skeleton, rest_frames, physics: PhysicsConfig, location, pose: str, size_scale: float = 1.0
) -> list[dict[str, Any]]:
    fruits = physics.fruits
    mode = fruits["mode"]
    if mode == "none":
        return []
    physical = mode == "physical"
    detachable = physical and bool(fruits["detachable"])
    bodies = []
    for fruit in skeleton.fruits:
        source_radius = float(fruit.radius)
        radius = source_radius * size_scale  # visuals.fruits.size_scale is cosmetic
        body = {
            "id": f"Fruit_{_safe(fruit.id)}",
            "shape": "sphere",
            "parent_branch_id": location[fruit.node][0],
            "radius": radius,
            # Mass always comes from the skeleton radius, not the drawn size.
            "mass": (4.0 / 3.0) * math.pi * source_radius**3 * float(fruits["density"]),
            "maturation": float(fruit.ripeness),
            "physical": physical,
            "detachment_enabled": detachable,
            "exclude_from_articulation": detachable,
            "break_force": float(fruits["break_force_n"]) if detachable else None,
            "canonical_node_id": fruit.node,
            "canonical_organ_id": fruit.id,
            "canonical_primitive_id": fruit.id,
            "parent_axis_id": fruit.node,
            "rest_frame": rest_frames[fruit.node],
            "source_center": None if fruit.center is None else list(fruit.center),
        }
        if pose == "canonical" and fruit.center is not None:
            body["rest_center"] = list(fruit.center)
        bodies.append(body)
    return bodies
