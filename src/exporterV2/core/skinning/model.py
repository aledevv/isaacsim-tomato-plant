"""Data models shared by the vegetative skinning backend."""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional

from pxr import Gf


@dataclass(frozen=True)
class VisualProfile:
    radial_segments: int = 20
    axial_spacing_m: float = 0.005
    skin_blend_half_width_m: float = 0.010

    radius_transition_half_width_m: float = 0.025
    radius_transition_max_fraction: float = 0.45
    radius_transition_samples: int = 9

    junction_bulge_amplitude: float = 0.18
    junction_bulge_sigma: float = 0.04
    root_parent_fraction: float = 0.92
    root_shoulder_amplitude: float = 0.18


@dataclass(frozen=True)
class BranchSpec:
    """Physical and visual discretization for one straight V2 branch."""

    physics_links: int
    radius: float
    inner_radius: float
    link_height: float
    density: float
    young_modulus: float
    colliders_per_link: int = 2
    collider_radius_scale: float = 0.90
    collider_length_scale: float = 0.92
    visual: VisualProfile = VisualProfile()


@dataclass(frozen=True)
class PhysicsGains:
    """Degree-based angular drive gains used by USD/PhysX."""

    stiffness: float
    damping: float
    attachment_stiffness: float
    attachment_damping: float


@dataclass
class BranchData:
    """A V2 branch resolved into world-space rest-pose data."""

    definition: Dict[str, Any]
    spec: BranchSpec
    branch_id: str
    parent_id: Optional[str]
    n_links: int
    radius: float
    inner_radius: float
    link_height: float
    start: Gf.Vec3d
    axis: Gf.Vec3d
    orientation: Gf.Quatf
    link_bases: List[Gf.Vec3d]
    link_orientations: List[Gf.Quatf]
    link_lengths: List[float]
    link_radii: List[float]
    link_collider_radii: List[float]
    link_masses: List[float]
    link_metadata: List[Dict[str, Any]]
    link_paths: List[str]
    physics_root_path: str
    visual_root_path: str
    skel_root_path: str
    skeleton_path: str
    animation_path: str
    mesh_path: str
    mass: float
    gains: PhysicsGains
    joint_type: str
    attachment_joint_type: str
    bend_limit_deg: Optional[float]
    locked_joints: bool
    parent_link_index: Optional[int] = None
    attachment_local_pos0: Optional[Gf.Vec3f] = None
    attachment_local_rot0: Optional[Gf.Quatf] = None
    centered_terminal: bool = False
    centered_terminal_host: bool = False
    explicit_link_poses: bool = False

    @property
    def total_length(self) -> float:
        return sum(self.link_lengths)

    def as_registry_entry(self):
        """Return the tuple consumed by the legacy truss/terminal builders."""
        return (
            list(self.link_paths),
            list(self.link_bases),
            Gf.Vec3d(self.axis),
            Gf.Quatf(self.orientation),
        )


@dataclass(frozen=True)
class VisualSegment:
    """One botanical radius span in world-space meters."""

    source_id: str
    start_arc: float
    length: float
    radius: float

    # Optional visual-only radius at the distal end.
    # None preserves the previous constant-radius behaviour.
    end_radius: Optional[float] = None

    @property
    def end_arc(self) -> float:
        return self.start_arc + self.length

    @property
    def distal_radius(self) -> float:
        return (
            self.radius
            if self.end_radius is None
            else self.end_radius
        )


@dataclass
class VisualAxisData:
    """One continuous visual tube driven by one or more physical branches."""

    axis_id: str
    members: List[BranchData]
    member_offsets: Dict[str, float]
    member_lengths: Dict[str, float]
    visual_segments: List[VisualSegment]
    link_paths: List[str]
    link_bases: List[Gf.Vec3d]
    link_orientations: List[Gf.Quatf]
    bone_starts: List[float]
    bone_lengths: List[float]
    start: Gf.Vec3d
    axis: Gf.Vec3d
    orientation: Gf.Quatf
    total_length: float
    visual_root_path: str
    skel_root_path: str
    skeleton_path: str
    animation_path: str
    mesh_path: str
    parent_radius: Optional[float]
    attachment_arcs: List[float]

    @property
    def definition(self) -> Dict[str, Any]:
        return self.members[0].definition

    @property
    def profile(self) -> VisualProfile:
        return self.members[0].spec.visual
