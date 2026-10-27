"""
tree_config.py - Tree Model Configuration

Configuration and physics helpers for recursive tree articulation.

Physics (Euler-Bernoulli beam theory):
    A  = pi (ro^2 - ri^2)        [m^2]
    I  = pi (ro^4 - ri^4) / 4    [m^4]
    K  = E * I / L               [N*m/rad]
    D  = 2*zeta * sqrt(K * J)    [N*m*s/rad]
    M  = rho * A * h             [kg]

All dimensions in BRANCHES are PRE-scale (meters at GLOBAL_SCALE=1).
The generator multiplies by GLOBAL_SCALE before building USD or computing physics.

--- BRANCHES list format ---

Each dict describes one chain (trunk or branch). Fields:

  id          (str)        Unique identifier for this chain.
  system      (str)        ``vegetative`` or ``truss``. Older definitions
                           fall back to physics_profile for classification.
  parent      (str|None)   id of the parent chain, or None for the trunk.
  attach_link (int|None)   1-based index of the parent link to attach to.
                           None when parent is None.
                           1 = first (bottom) link, n_links = top link.
  n_links     (int)        Number of rigid segments in this chain.
  radius      (float)      Cylinder radius [m, pre-scale].
  inner_radius(float)      Optional hollow-section inner radius [m, pre-scale].
  young_modulus(float)     Optional branch-specific Young's modulus [Pa].
  density     (float)      Optional branch-specific density [kg/m^3].
  height      (float)      Cylinder height per link [m, pre-scale].
  tilt        (float)      Tilt angle away from parent local-Z axis [deg].
  rot         (float)      Azimuthal rotation around parent local-Z axis [deg].
  joint_type  (str)        Optional joint mode: d6, fixed, d6_planar, or revolute_planar.
  bend_limit_deg (float)   Optional branch-specific D6 angular limit [deg].
  drive_stiffness_scale (float) Optional positive multiplier for D6 stiffness.

Run standalone to verify physics:
    python -m exporterV2.tree_config
"""

import math

MAX_N_JOINTS = 250  # D6-joint budget for stable Isaac Sim runs


class PhysicsRuntimeConfig:
    """Runtime PhysX defaults used by the exporter entry points."""

    # RigidTrunk feature flag: keep the main stem stable for fruit-heavy tests.
    RIGID_TRUNK = True
    PHYSICS_HZ = 480
    SOLVER_POSITION_ITERATIONS = 32
    SOLVER_VELOCITY_ITERATIONS = 4
    TERMINAL_BODY_SOLVER_POSITION_ITERATIONS = 32
    TERMINAL_BODY_SOLVER_VELOCITY_ITERATIONS = 1
    STABILIZED_TERMINAL_BODY_SOLVER_POSITION_ITERATIONS = 64
    STABILIZED_TERMINAL_BODY_SOLVER_VELOCITY_ITERATIONS = 4
    ENABLE_GPU_DYNAMICS = True


class BranchResolutionConfig:
    """Initial spatial-resolution limits applied before optimization."""

    MAX_LINKS_PER_BRANCH = 10


class OrganGenerationConfig:
    """Global debug switches for CSV-derived organ hierarchies."""

    CREATE_LATERAL_BRANCHES = True

    CREATE_LEAF_BRANCHES = True
    CREATE_PETIOLES = True
    CREATE_LEAF_RACHIS = True
    CREATE_PETIOLULES = True

    CREATE_TRUSSES = True
    CREATE_TRUSS_RACHIS = True
    CREATE_PEDICELS = True
    CREATE_TOMATOES = True


class LoggingConfig:
    """Console logging controls."""

    # Print detailed CSV parsing / plant-component information.
    VERBOSE_PLANT_INFO = False


class OutputConfig:
    """Console output controls."""

    STEP_1_VERBOSE = False   # USD generation / geometry details

    TERMINAL_GEOMETRY_WARNINGS_VERBOSE = False # True = print every geometry warning, False = print only one compact summary

# ==============================================================================
# GLOBAL SCALE & PHYSICS CONSTANTS
# ==============================================================================

GLOBAL_SCALE = 2.0      # All raw dimensions are multiplied by this

BEND_LIMIT_DEG = 30.0   # +/- deg soft limit on rotX/rotY joint drives
GAP            = 0.0  # Gap between adjacent links [m, pre-scale]

# Phyllotaxis angle (golden angle) for leaf and fruit positioning
# Used when CSV doesn't provide explicit ccw_orientation
PHYLLOTAXIS = 137.5  # [deg]

# Minimum link radius for PhysX stability (post-scale, in world units)
# Links with radius below this threshold (after scaling) may cause numerical
# instability in the articulation solver. Value determined empirically.
MIN_LINK_RADIUS_WORLD = 0.002  # [m] 2mm minimum for PhysX stability


class TrussGeometryConfig:
    """
    Geometry constants for adapter-generated tomato trusses.

    Length and radius values are pre-scale dimensions and are multiplied by
    GLOBAL_SCALE when USD geometry is built.
    """
    INITIAL_TILT_DEG = 45.0
    MIN_TILT_DEG = 45.0
    MAX_TILT_DEG = 95.0
    RACHIS_SEGMENT_LENGTH = 0.020
    RACHIS_RADIUS = 0.00075
    PEDICEL_LENGTH = 0.015
    PEDICEL_RADIUS = 0.0005
    LATERAL_PEDICEL_CHORD_ANGLE_DEG = 56.0
    # PlantState keeps the GroIMP length in metadata but authors a slightly
    # longer V2 pedicel so large tomato visuals do not crowd the truss rachis.
    # Mesh, collider, joint endpoint and tomato all follow this scale.
    PLANT_STATE_PEDICEL_LENGTH_SCALE = 3.0


class BioConfig:
    """Biological parameters for plant tissue."""
    YOUNG_MODULUS = 50.64e6  # [Pa] tomato stalk, Gao et al. 2024 (Agriculture 14(4):531)
    DAMPING_RATIO = 0.8     # Critically damped (zeta=1.0) to prevent 30s oscillations
    PLANT_DENSITY = 1000.0   # [kg/m^3] plant tissue density


class TrussPhysicsConfig:
    """
    Custom physics parameters for truss structures (rachis + pedicels).
    
    Trusses have different mechanical properties than stems/branches:
    - Higher stiffness to prevent excessive drooping
    - Higher damping to reduce oscillations
    - Custom minimum K to handle thin pedicels
    """
    # Manual tuning surface for PlantState trusses.  The exporter deliberately
    # does not auto-select these values: adjust rachis and pedicel stiffness or
    # damping independently here, then regenerate the USDA for comparison.
    # Day-160 fruit-free calibration approved in GUI (2026-08-25).
    RACHIS_YOUNG_MODULUS = 20.0e9  # [Pa]
    PEDICEL_YOUNG_MODULUS = 4.0e9  # [Pa]
    RACHIS_DAMPING_RATIO = 4.0
    PEDICEL_DAMPING_RATIO = 4.0
    # Compatibility aliases for legacy BRANCHES without component-specific data.
    YOUNG_MODULUS = RACHIS_YOUNG_MODULUS
    DAMPING_RATIO = RACHIS_DAMPING_RATIO
    # Moderate solver adaptation selected against the final day-160 plant.
    # This intentionally accepts a harder terminal-body mass ratio instead of
    # hiding it behind the former extremely dense vegetative supports.
    # Do not raise this to repair detachable-fruit instability: at day 160 the
    # added self-weight is transferred to the lateral support branches.
    PLANT_DENSITY = 2000.0  # [kg/m^3]
    MIN_K = 0.001             # [N·m/rad] Minimum stiffness for thin pedicels
    PEDICEL_BEND_LIMIT_DEG = 25.0
    # Third manual tuning control for pedicel angular drives.  Keep this
    # separate from Young's modulus so their response can be compared directly.
    PEDICEL_DRIVE_STIFFNESS_SCALE = 0.2
    # Breakable tomato detachment remains enabled for truss tests. Initial
    # compenetrating tomato pairs are filtered separately to avoid solver
    # repulsion spikes breaking the joints at startup.
    TOMATO_DETACHMENT_ENABLED = True
    # This is comfortably above the static weight of the generated tomatoes,
    # so gravity alone should not break the joint when detachment is enabled.
    TOMATO_DETACHMENT_BREAK_FORCE_N = 6.0
    # Keep tomato bodies outside the articulation while detachment is active.
    TOMATO_DETACHMENT_EXCLUDE_FROM_ARTICULATION = True
    # When the fixed joint is excluded, keep tomatoes outside the articulation
    # root and attach them as regular rigid bodies through the terminal joint.
    TOMATO_DETACHMENT_BODY_PARENT_PATH = "/World/TerminalBodies"
    # Filter tomato pairs that start overlapped, preventing contact impulses
    # from breaking detachable joints during initialization.
    FILTER_TERMINAL_BODY_PAIR_OVERLAPS = True


# ==============================================================================
# PLANT COLORS
# ==============================================================================

class PlantColors:
    """
    Display colors for plant organs in Isaac Sim.
    
    All colors are (R, G, B) tuples in linear float [0.0, 1.0].
    
    Tomato color is computed at runtime via the ripening interpolation:
        color = lerp(TOMATO_UNRIPE, TOMATO_RIPE, maturation)
    where maturation ∈ [0.0, 1.0] from CSV fruit_age_dd / fruit_ripening_dd.
    """

    # Stems and lateral branches — light green (#436c3a)
    STEM = (0.263, 0.424, 0.227)

    # Petiole and rachis — slightly darker green
    PETIOLE = (0.263, 0.424, 0.227)

    # Petiolules — same as petiole
    PETIOLULE = (0.263, 0.424, 0.227)

    # Leaf blades — deep dark green (#325928)
    LEAF_BLADE = (0.196, 0.349, 0.157)

    # Truss rachis — olive green
    TRUSS_RACHIS = (0.263, 0.424, 0.227)

    # Pedicels — medium green
    PEDICEL = (0.263, 0.424, 0.227)

    # Tomato unripe color (maturation=0.0)
    TOMATO_UNRIPE = (0.25, 0.65, 0.08)

    # Tomato ripe color (maturation=1.0)
    TOMATO_RIPE = (0.90, 0.17, 0.08)

    @classmethod
    def tomato_color(cls, maturation: float) -> tuple:
        """Interpolate between TOMATO_UNRIPE and TOMATO_RIPE based on maturation [0..1]."""
        t = max(0.0, min(1.0, maturation))
        r = cls.TOMATO_UNRIPE[0] + t * (cls.TOMATO_RIPE[0] - cls.TOMATO_UNRIPE[0])
        g = cls.TOMATO_UNRIPE[1] + t * (cls.TOMATO_RIPE[1] - cls.TOMATO_UNRIPE[1])
        b = cls.TOMATO_UNRIPE[2] + t * (cls.TOMATO_RIPE[2] - cls.TOMATO_UNRIPE[2])
        return (r, g, b)




BRANCHES = [
    {
        "id"         : "trunk",
        "system"     : "vegetative",
        "parent"     : None,
        "attach_link": None,
        "n_links"    : 5,
        "radius"     : 0.10,   # 10 cm -> 1.0 m world
        "height"     : 0.20,   # 20 cm -> 2.0 m world
        "tilt"       : 0.0,
        "rot"        : 0.0,
        "joint_type" : "fixed" if PhysicsRuntimeConfig.RIGID_TRUNK else "d6",
    },
    {
        "id"         : "branchA",
        "system"     : "vegetative",
        "parent"     : "trunk",
        "attach_link": 3,      # attaches to trunk link 3 (1-based)
        "n_links"    : 4,
        "radius"     : 0.03,   # 3 cm -> 0.3 m world
        "height"     : 0.15,   # 15 cm -> 1.5 m world
        "tilt"       : 45.0,
        "rot"        : 0.0,
    },
    {
        "id"         : "subA1",
        "system"     : "vegetative",
        "parent"     : "branchA",
        "attach_link": 2,      # attaches to branchA link 2 (1-based)
        "n_links"    : 3,
        "radius"     : 0.005,  # 0.5 cm -> 5 cm world
        "height"     : 0.10,   # 10 cm -> 1.0 m world
        "tilt"       : 40.0,
        "rot"        : 90.0,
    },
]


# ==============================================================================
# PHYSICS HELPERS
# ==============================================================================

def scaled(value: float) -> float:
    """Apply GLOBAL_SCALE to a pre-scale dimension."""
    return value * GLOBAL_SCALE


def clamp_radius(radius_prescale: float) -> tuple[float, bool]:
    """
    Clamp radius to minimum world-space value for PhysX stability.
    
    PhysX articulations with very thin links can become numerically unstable.
    This function ensures the radius (after scaling) meets the minimum threshold.
    
    Args:
        radius_prescale: Radius in pre-scale units [m]
    
    Returns:
        Tuple (clamped_radius_prescale, was_clamped):
            clamped_radius_prescale: Adjusted radius in pre-scale units [m]
            was_clamped: True if clamping was applied, False otherwise
    
    Example:
        >>> GLOBAL_SCALE = 2.0
        >>> MIN_LINK_RADIUS_WORLD = 0.004
        >>> clamp_radius(0.001)  # 0.001 * 2.0 = 0.002m < 0.004m
        (0.002, True)  # Clamped to 0.002 pre-scale (0.004m world)
        >>> clamp_radius(0.005)  # 0.005 * 2.0 = 0.010m > 0.004m
        (0.005, False)  # No clamping needed
    """
    radius_world = radius_prescale * GLOBAL_SCALE
    
    if radius_world < MIN_LINK_RADIUS_WORLD:
        clamped_prescale = MIN_LINK_RADIUS_WORLD / GLOBAL_SCALE
        return (clamped_prescale, True)
    
    return (radius_prescale, False)


def _remap_attachment(
    attach_link: int,
    attach_frac: float,
    old_n_links: int,
    new_n_links: int,
) -> tuple[int, float]:
    """Preserve a child's normalized axial attachment after parent resampling."""
    if not 1 <= attach_link <= old_n_links:
        raise ValueError(
            f"attach_link={attach_link} is outside [1, {old_n_links}]"
        )
    if not 0.0 <= attach_frac <= 1.0:
        raise ValueError(f"attach_frac={attach_frac} is outside [0, 1]")

    axial_fraction = (attach_link - 1 + attach_frac) / old_n_links
    if axial_fraction >= 1.0:
        return new_n_links, 1.0

    new_position = axial_fraction * new_n_links
    zero_based_link = math.floor(new_position)
    return zero_based_link + 1, new_position - zero_based_link


def limit_branch_resolution(
    branches: list[dict],
    max_links: int | None = None,
    *,
    verbose: bool = True,
) -> tuple[list[dict], list[dict]]:
    """
    Cap every chain while preserving total length and child attachment height.

    This is a pre-optimization upper bound. Optimizers remain free to reduce
    ``n_links`` further. Input dictionaries are never modified.

    Returns:
        ``(limited_branches, changes)`` where each change records the branch
        id and its old/new resolution.
    """
    if max_links is None:
        max_links = BranchResolutionConfig.MAX_LINKS_PER_BRANCH
    if not isinstance(max_links, int) or max_links <= 0:
        raise ValueError("max_links must be a positive integer")

    limited = [branch.copy() for branch in branches]
    children_by_parent: dict[str, list[dict]] = {}
    for branch in limited:
        parent_id = branch.get("parent")
        if parent_id is not None:
            children_by_parent.setdefault(parent_id, []).append(branch)

    changes = []
    for branch in limited:
        old_n_links = branch.get("n_links", 0)
        if old_n_links <= max_links:
            continue

        old_height = branch.get("height", 0.0)
        new_n_links = max_links
        branch["n_links"] = new_n_links
        branch["height"] = old_height * old_n_links / new_n_links

        remapped_children = 0
        for child in children_by_parent.get(branch["id"], []):
            new_link, new_frac = _remap_attachment(
                child["attach_link"],
                child.get("attach_frac", 1.0),
                old_n_links,
                new_n_links,
            )
            child["attach_link"] = new_link
            child["attach_frac"] = new_frac
            remapped_children += 1

        change = {
            "branch_id": branch["id"],
            "old_n_links": old_n_links,
            "new_n_links": new_n_links,
            "children_remapped": remapped_children,
        }
        changes.append(change)
        if verbose:
            print(
                f"[CONFIG] Capped branch '{branch['id']}': "
                f"{old_n_links} -> {new_n_links} links "
                f"({remapped_children} children remapped)"
            )

    return limited, changes


def compute_cross_section_area(radius: float, inner_radius: float = 0.0) -> float:
    """Cross-sectional area for a solid or hollow circular stem [m^2]."""
    if inner_radius < 0.0:
        raise ValueError("inner_radius must be non-negative")
    if inner_radius >= radius:
        raise ValueError("inner_radius must be smaller than radius")
    return math.pi * (radius**2 - inner_radius**2)


def compute_mass(
    radius: float,
    height: float,
    density: float = BioConfig.PLANT_DENSITY,
    inner_radius: float = 0.0,
) -> float:
    """
    Cylindrical segment mass [kg].
    
    Args:
        radius: Cylinder radius in world-unit meters
        height: Cylinder height in world-unit meters
        density: Material density [kg/m^3]
        inner_radius: Hollow-section inner radius in world-unit meters
    
    Returns:
        Mass in kilograms
    """
    return density * compute_cross_section_area(radius, inner_radius) * height


def compute_second_moment(radius: float, inner_radius: float = 0.0) -> float:
    """Second moment of area for a solid or hollow circular stem [m^4]."""
    if inner_radius < 0.0:
        raise ValueError("inner_radius must be non-negative")
    if inner_radius >= radius:
        raise ValueError("inner_radius must be smaller than radius")
    return math.pi * (radius**4 - inner_radius**4) / 4.0


def compute_flexural_rigidity(
    radius: float,
    young_modulus: float,
    inner_radius: float = 0.0,
) -> float:
    """Compute flexural rigidity (EI) [N*m^2]."""
    I = compute_second_moment(radius, inner_radius)
    return young_modulus * I


def compute_hinge_stiffness_rad(len_L: float, EI_L: float, len_R: float, EI_R: float) -> float:
    """
    Compute rotational hinge stiffness in series [N*m/rad].
    Represents half of the left link and half of the right link in bending.
    """
    compliance = (0.5 * len_L / EI_L) + (0.5 * len_R / EI_R)
    return 1.0 / compliance


def compute_moment_of_inertia(
    radius: float,
    height: float,
    mass: float,
    inner_radius: float = 0.0,
) -> float:
    """
    Moment of inertia of a solid cylinder about an axis through one end.
    Uses parallel-axis theorem.
    """
    radial_term = radius**2 + inner_radius**2
    J_center = mass * (3.0 * radial_term + height**2) / 12.0
    J_pivot = J_center + mass * (height / 2.0)**2
    return J_pivot


def calculate_physics_params(
    radius: float,
    height: float,
    mass: float,
    legacy_physics: bool = False,
    young_modulus: float = BioConfig.YOUNG_MODULUS,
    damping_ratio: float | None = None,
    inner_radius: float = 0.0,
):
    """
    Compute spring constant K and damper D for a cylindrical link.
    
    Args:
        radius: Cylinder radius in world-unit meters
        height: Cylinder height in world-unit meters
        mass: Cylinder mass in kilograms
        young_modulus: Structural Young's modulus [Pa]
        damping_ratio: Optional damping ratio override
        inner_radius: Hollow-section inner radius in world-unit meters
    
    Returns:
        Tuple (K, D):
            K: USD angular spring constant [N*m/degree]
            D: USD angular damping coefficient [N*m*s/degree]
    """
    I = compute_second_moment(radius, inner_radius)
    K = (young_modulus * I) / height
    
    J = compute_moment_of_inertia(radius, height, mass, inner_radius)
    if damping_ratio is None:
        damping_ratio = 0.1 if legacy_physics else BioConfig.DAMPING_RATIO
    D = 2.0 * damping_ratio * math.sqrt(K * J)

    # Isaac Sim expects stiffness and damping w.r.t. degrees (not radians)
    rad_to_deg = math.pi / 180.0
    K_deg = K * rad_to_deg
    D_deg = D * rad_to_deg

    return K_deg, D_deg


def calculate_truss_physics_params(
    radius: float,
    height: float,
    mass: float,
    *,
    young_modulus: float | None = None,
    damping_ratio: float | None = None,
):
    """
    Compute spring constant K and damper D for truss components (rachis + pedicels).
    
    Uses custom TrussPhysicsConfig with higher stiffness and damping to:
    - Prevent excessive drooping
    - Reduce oscillations
    - Handle thin pedicels (r ~ 2-3mm)
    
    Args:
        radius: Cylinder radius in world-unit meters
        height: Cylinder height in world-unit meters
        mass: Cylinder mass in kilograms
    
    Returns:
        Tuple (K, D):
            K: Spring constant [N*m/rad]
            D: Damping coefficient [N*m*s/rad]
    """
    if young_modulus is None:
        young_modulus = TrussPhysicsConfig.YOUNG_MODULUS
    if damping_ratio is None:
        damping_ratio = TrussPhysicsConfig.DAMPING_RATIO
    I = compute_second_moment(radius)
    K = (young_modulus * I) / height
    
    # Apply minimum stiffness for thin pedicels
    if K < TrussPhysicsConfig.MIN_K:
        K = TrussPhysicsConfig.MIN_K
    
    J = compute_moment_of_inertia(radius, height, mass)
    D = 2.0 * damping_ratio * math.sqrt(K * J)

    # Isaac Sim expects stiffness and damping w.r.t. degrees (not radians)
    rad_to_deg = math.pi / 180.0
    K_deg = K * rad_to_deg
    D_deg = D * rad_to_deg

    return K_deg, D_deg


# ==============================================================================
# VALIDATION
# ==============================================================================

def validate_branches(branches: list, skip_limit_check: bool = False) -> None:
    """
    Validate the BRANCHES list and raise ValueError with a clear message on any issue.

    Checks:
      - No duplicate ids
      - Exactly one root (parent=None)
      - Every parent id exists in the list
      - attach_link is within [1, parent.n_links] for non-root branches
      - Total D6 joint count <= MAX_N_JOINTS (unless skip_limit_check=True)
    
    Args:
        branches: List of branch definitions to validate
        skip_limit_check: If True, skip the link count limit check
    
    Raises:
        ValueError: If validation fails with descriptive error message
    """
    ids = [b["id"] for b in branches]

    # Check for duplicate ids
    seen = set()
    for bid in ids:
        if bid in seen:
            raise ValueError(f"[tree_config] Duplicate branch id: '{bid}'")
        seen.add(bid)

    # Check root count
    roots = [b for b in branches if b.get("parent") is None]
    if len(roots) == 0:
        raise ValueError("[tree_config] No root branch found (parent=None). Add one trunk.")
    if len(roots) > 1:
        root_ids = [r["id"] for r in roots]
        raise ValueError(f"[tree_config] Multiple root branches: {root_ids}. Only one is allowed.")

    # Build id -> n_links map for parent range check
    id_to_nlinks = {b["id"]: b["n_links"] for b in branches}

    # Validate each branch
    for b in branches:
        for key in ("n_links", "radius", "height"):
            if b.get(key, 0) <= 0:
                raise ValueError(
                    f"[tree_config] Branch '{b['id']}' has invalid {key}={b.get(key)}. "
                    f"Expected a positive value."
                )

        link_specs = b.get("link_specs")
        if link_specs is not None:
            if not isinstance(link_specs, list) or len(link_specs) != b["n_links"]:
                raise ValueError(
                    f"[tree_config] Branch '{b['id']}' link_specs must contain "
                    f"exactly n_links={b['n_links']} entries."
                )
            spec_ids = [spec.get("id") for spec in link_specs]
            if any(not isinstance(value, str) or not value for value in spec_ids):
                raise ValueError(
                    f"[tree_config] Branch '{b['id']}' link_specs require non-empty ids."
                )
            if len(set(spec_ids)) != len(spec_ids):
                raise ValueError(
                    f"[tree_config] Branch '{b['id']}' has duplicate link_spec ids."
                )
            pose_flags = [spec.get("rest_frame") is not None for spec in link_specs]
            if any(pose_flags) and not all(pose_flags):
                raise ValueError(
                    f"[tree_config] Branch '{b['id']}' cannot mix explicit and legacy link poses."
                )
            for index, spec in enumerate(link_specs):
                if float(spec.get("length", 0.0)) <= 0.0:
                    raise ValueError(
                        f"[tree_config] Branch '{b['id']}' link_spec {index} has invalid length."
                    )
                if float(spec.get("radius", 0.0)) <= 0.0:
                    raise ValueError(
                        f"[tree_config] Branch '{b['id']}' link_spec {index} has invalid radius."
                    )
                frame = spec.get("rest_frame")
                if frame is not None:
                    _validate_link_rest_frame(b["id"], index, frame)
            if all(pose_flags):
                _validate_explicit_link_continuity(b["id"], link_specs)

        if b.get("parent") is None:
            continue  # Root branch - skip parent checks

        parent_id = b["parent"]
        if parent_id not in id_to_nlinks:
            raise ValueError(
                f"[tree_config] Branch '{b['id']}' references unknown parent '{parent_id}'. "
                f"Known ids: {ids}"
            )

        attach = b.get("attach_link")
        if attach is None:
            raise ValueError(
                f"[tree_config] Branch '{b['id']}' has a parent but no 'attach_link'. "
                f"Set attach_link to an integer in [1, {id_to_nlinks[parent_id]}]."
            )
        if not isinstance(attach, int):
            raise ValueError(
                f"[tree_config] Branch '{b['id']}' attach_link must be an int, "
                f"got {type(attach).__name__}."
            )
        parent_nlinks = id_to_nlinks[parent_id]
        if not (1 <= attach <= parent_nlinks):
            raise ValueError(
                f"[tree_config] Branch '{b['id']}' attach_link={attach} is out of range "
                f"[1, {parent_nlinks}] for parent '{parent_id}'."
            )

    # Check D6 joint budget. Fixed branches are intentionally excluded because
    # they are much cheaper for Isaac Sim than D6 articulation joints.
    total = sum(
        b["n_links"]
        for b in branches
        if b.get("joint_type", "d6").lower() != "fixed"
    )
    if total > MAX_N_JOINTS and not skip_limit_check:
        raise ValueError(
            f"[tree_config] D6 joint count {total} exceeds PhysX articulation budget of {MAX_N_JOINTS}. "
            f"Run the optimizer or reduce D6 branches."
        )


def _validate_link_rest_frame(branch_id: str, index: int, frame) -> None:
    if (
        not isinstance(frame, (list, tuple))
        or len(frame) != 4
        or any(not isinstance(row, (list, tuple)) or len(row) != 4 for row in frame)
    ):
        raise ValueError(
            f"[tree_config] Branch '{branch_id}' link_spec {index} rest_frame must be 4x4."
        )
    values = [float(value) for row in frame for value in row]
    if not all(math.isfinite(value) for value in values):
        raise ValueError(
            f"[tree_config] Branch '{branch_id}' link_spec {index} rest_frame must be finite."
        )
    if any(abs(float(frame[3][column])) > 1e-9 for column in range(3)) or not math.isclose(
        float(frame[3][3]), 1.0, abs_tol=1e-9
    ):
        raise ValueError(
            f"[tree_config] Branch '{branch_id}' link_spec {index} rest_frame is not homogeneous."
        )
    columns = [
        [float(frame[row][column]) for row in range(3)]
        for column in range(3)
    ]
    for column in columns:
        norm = math.sqrt(sum(value * value for value in column))
        if not math.isclose(norm, 1.0, abs_tol=1e-7):
            raise ValueError(
                f"[tree_config] Branch '{branch_id}' link_spec {index} rotation is not orthonormal."
            )
    for left in range(3):
        for right in range(left + 1, 3):
            dot = sum(a * b for a, b in zip(columns[left], columns[right]))
            if abs(dot) > 1e-7:
                raise ValueError(
                    f"[tree_config] Branch '{branch_id}' link_spec {index} rotation is not orthonormal."
                )


def _validate_explicit_link_continuity(branch_id: str, specs: list) -> None:
    tolerance = 1e-6
    for index, (previous, current) in enumerate(zip(specs, specs[1:]), start=1):
        frame = previous["rest_frame"]
        length = float(previous["length"])
        expected = [
            float(frame[row][3]) + float(frame[row][2]) * length
            for row in range(3)
        ]
        actual = [float(current["rest_frame"][row][3]) for row in range(3)]
        error = math.sqrt(sum((a - b) ** 2 for a, b in zip(expected, actual)))
        if error > tolerance:
            raise ValueError(
                f"[tree_config] Branch '{branch_id}' link_specs {index - 1}->{index} "
                f"are discontinuous by {error:.6g}m."
            )


# ==============================================================================
# SUMMARY PRINTER
# ==============================================================================

def print_tree_summary(branches=None) -> None:
    """Print physics parameters for every branch in the list."""
    if branches is None:
        branches = BRANCHES

    validate_branches(branches)

    print()
    print("=" * 88)
    print(f"  Tree Config  |  GLOBAL_SCALE={GLOBAL_SCALE}  |  E={BioConfig.YOUNG_MODULUS:.2e} Pa  |  zeta={BioConfig.DAMPING_RATIO}")
    print("=" * 88)
    hdr = (f"  {'id':<12} {'parent':<12} {'alink':>5} {'r(m)':>8} {'h(m)':>8} "
           f"{'mass(kg)':>9} {'K(N*m/r)':>12} {'D(N*m*s)':>12} {'T(s)':>7}")
    print(hdr)
    print("-" * 88)

    total_d6_joints = 0
    for b in branches:
        r_w  = scaled(b["radius"])
        h_w  = scaled(b["height"])
        m    = compute_mass(r_w, h_w)
        K, D = calculate_physics_params(r_w, h_w, m)
        T    = 2.0 * math.pi * math.sqrt(m / K) if K > 0 else float("inf")
        al   = str(b.get("attach_link") or "-")
        pa   = b.get("parent") or "-"
        print(f"  {b['id']:<12} {pa:<12} {al:>5} {r_w:>8.4f} {h_w:>8.4f} "
              f"{m:>9.3f} {K:>12.2f} {D:>12.4f} {T:>7.4f}")
        if b.get("joint_type", "d6").lower() != "fixed":
            total_d6_joints += b["n_links"]

    print("-" * 88)
    status = "OK" if total_d6_joints <= MAX_N_JOINTS else "EXCEEDS LIMIT"
    print(f"  D6 joints: {total_d6_joints}  (PhysX budget: {MAX_N_JOINTS})  [{status}]")
    print()


if __name__ == "__main__":
    print_tree_summary()
