"""Author terminal fruit and leaf bodies for Exporter V2 stages."""

import math

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from ..physics import apply_physx_rigid_body_solver_settings
from ..tree_config import OutputConfig, PlantColors, TrussPhysicsConfig, scaled
from .collision import (
    add_collision_filter,
    check_sphere_cylinder_intersection,
    check_sphere_sphere_intersection,
)
from .geometry import create_sphere_rigid_body, create_static_mesh
from .joints import create_fixed_joint_to_tip
from .materials import get_or_create_tomato_fruit_material
from .pedicel_geometry import create_gravity_elbow_mesh, sample_gravity_elbow


def _resolve_terminal_body_attachment(body: dict, stem_path: str):
    detachment_enabled = (
        TrussPhysicsConfig.TOMATO_DETACHMENT_ENABLED
        and body.get("detachment_enabled", True)
    )
    exclude_from_articulation = body.get(
        "exclude_from_articulation",
        TrussPhysicsConfig.TOMATO_DETACHMENT_EXCLUDE_FROM_ARTICULATION,
    )
    break_force = (
        body.get("break_force", TrussPhysicsConfig.TOMATO_DETACHMENT_BREAK_FORCE_N)
        if detachment_enabled
        else None
    )
    body_parent_path = body.get("parent_path")
    if body_parent_path is None:
        body_parent_path = (
            getattr(
                TrussPhysicsConfig,
                "TOMATO_DETACHMENT_BODY_PARENT_PATH",
                "/World/TerminalBodies",
            )
            if exclude_from_articulation
            else stem_path
        )

    return (
        detachment_enabled,
        break_force,
        exclude_from_articulation,
        body_parent_path,
    )


def validate_terminal_body_clearance(
    terminal_body_records,
    branch_registry,
    branches,
    margin: float = 0.002,
    stage=None,
    apply_filters: bool = False,
    filter_terminal_body_pairs: bool = False,
    branch_defs=None,
):
    """Report initial overlaps and optionally filter only those collision pairs."""
    terminal_body_records = [
        record
        for record in terminal_body_records
        if record.get("physical", True)
        # Explicit PlantState terminal bodies are audited later from their
        # actual authored sphere/capsule colliders.  The legacy approximation
        # assumes one straight axis per branch and is invalid for canonical
        # per-link frames.
        and record.get("canonical_primitive_id") is None
    ]
    if not terminal_body_records:
        return

    warnings = []
    filtered_pairs = set()
    if branch_defs is None:
        branch_defs = {branch["id"]: branch for branch in branches}

    def maybe_filter(path_a: str, path_b: str) -> None:
        if not (apply_filters and stage and path_a and path_b):
            return
        key = tuple(sorted((path_a, path_b)))
        if key in filtered_pairs:
            return
        add_collision_filter(stage, path_a, path_b)
        add_collision_filter(stage, path_b, path_a)
        filtered_pairs.add(key)

    for index, body_a in enumerate(terminal_body_records):
        for body_b in terminal_body_records[index + 1 :]:
            intersects, distance, overlap = check_sphere_sphere_intersection(
                body_a["pos"],
                body_a["radius"],
                body_b["pos"],
                body_b["radius"],
                margin,
            )
            if intersects:
                warnings.append(
                    f"terminal bodies '{body_a['id']}' and '{body_b['id']}' overlap "
                    f"by {overlap * 1000.0:.1f}mm (distance={distance * 1000.0:.1f}mm)"
                )
                if filter_terminal_body_pairs:
                    maybe_filter(body_a.get("path"), body_b.get("path"))

    for body in terminal_body_records:
        parent_branch_id = body["parent_branch_id"]
        immediate_parent_path = branch_registry[parent_branch_id][0][-1]

        for branch_id, (link_paths, link_bases, axis, _) in branch_registry.items():
            branch_def = branch_defs[branch_id]
            branch_radius = scaled(branch_def["radius"])
            branch_height = scaled(branch_def["height"])

            for link_index, link_base in enumerate(link_bases):
                link_path = link_paths[link_index]
                if link_path == immediate_parent_path:
                    continue

                intersects, _, overlap = check_sphere_cylinder_intersection(
                    body["pos"],
                    body["radius"],
                    link_base,
                    axis,
                    branch_height,
                    branch_radius,
                    margin,
                )
                if intersects:
                    warnings.append(
                        f"terminal body '{body['id']}' intersects "
                        f"'{branch_id}_Link_{link_index + 1:02d}' "
                        f"by {overlap * 1000.0:.1f}mm"
                    )
                    maybe_filter(body.get("path"), link_path)

    if warnings:
        if OutputConfig.TERMINAL_GEOMETRY_WARNINGS_VERBOSE:
            print("\n" + "=" * 80)
            print("  TERMINAL BODY GEOMETRY WARNINGS")
            print("=" * 80)
            for warning in warnings[:25]:
                print(f"[WARNING] {warning}")
            if len(warnings) > 25:
                print(
                    f"[WARNING] ... {len(warnings) - 25} "
                    "additional geometry warnings omitted"
                )
            if filtered_pairs:
                print(
                    f"[INFO] Added {len(filtered_pairs) * 2} "
                    "terminal-body collision filters"
                )
            print("=" * 80 + "\n")
        else:
            print(
                f"[WARNING] Terminal body geometry: {len(warnings)} intersections "
                f"detected, {len(filtered_pairs)} collision pairs filtered"
            )
    elif OutputConfig.STEP_1_VERBOSE:
        print("[INFO] Terminal body geometry validation: no intersections detected")


def build_terminal_bodies(stage, stem_path, terminal_bodies, branch_registry, branch_defs):
    """Author terminal bodies and return sphere records used for validation."""
    terminal_body_records = []

    for body in terminal_bodies:
        shape = body.get("shape", "sphere")
        if shape not in ("sphere", "mesh"):
            print(
                f"[WARNING] Skipping terminal body '{body.get('id')}' "
                f"with unsupported shape '{shape}'"
            )
            continue

        parent_branch_id = body.get("parent_branch_id")
        if parent_branch_id not in branch_registry:
            print(
                f"[WARNING] Skipping terminal body '{body.get('id')}' because parent "
                f"branch '{parent_branch_id}' was not built"
            )
            continue

        mass = body.get("mass", 0.0)
        if shape == "sphere":
            radius = scaled(body.get("radius", 0.0))
            if radius <= 0.0 or mass <= 0.0:
                print(
                    f"[WARNING] Skipping terminal body '{body.get('id')}' "
                    "with invalid radius or mass"
                )
                continue
            child_offset = radius
        else:
            if mass <= 0.0:
                print(
                    f"[WARNING] Skipping terminal body '{body.get('id')}' with invalid mass"
                )
                continue
            child_offset = 0.0

        parent_paths, parent_bases, parent_axis, parent_orientation = branch_registry[
            parent_branch_id
        ]
        parent_definition = branch_defs[parent_branch_id]
        parent_specs = parent_definition.get("link_specs") or ()
        parent_height = scaled(
            parent_specs[-1]["length"]
            if parent_specs
            else parent_definition["height"]
        )
        parent_link_path = parent_paths[-1]
        parent_base = parent_bases[-1]
        explicit_center = body.get("rest_center")
        body_pos = (
            Gf.Vec3d(*(scaled(float(value)) for value in explicit_center))
            if explicit_center is not None
            else parent_base + parent_axis * (parent_height + child_offset)
        )
        explicit_orientation = None
        if body.get("rest_frame") is not None:
            from ..skinning.adapter import _quat_from_column_rotation

            explicit_orientation = _quat_from_column_rotation(body["rest_frame"])
        physical_orientation = explicit_orientation or parent_orientation
        is_pedicel = (
            "pedicel" in parent_branch_id.lower()
            or branch_defs[parent_branch_id].get("kind") == "pedicel"
        )
        if shape == "sphere" and is_pedicel and explicit_center is None:
            parent_rotation = Gf.Rotation(parent_orientation)
            gravity_local = parent_rotation.GetInverse().TransformDir(
                Gf.Vec3d(0.0, 0.0, -1.0)
            )
            centers, tangents = sample_gravity_elbow(
                parent_height, parent_branch_id, gravity_local
            )
            visual_overlap = 0.002
            tomato_center_local = centers[-1] + Gf.Vec3d(
                *tangents[-1]
            ).GetNormalized() * (radius - visual_overlap)
            body_pos = parent_base + parent_rotation.TransformDir(
                tomato_center_local
            )

        if shape == "sphere" and body.get("physical", True) is False:
            # Child of the pedicel tip link, so the tomato follows it when the
            # plant bends; no rigid body, collider or mass.
            visual_root = UsdGeom.Xform.Define(
                stage, f"{parent_link_path}/{body['id']}"
            )
            rest_world = Gf.Matrix4d(1.0)
            rest_world.SetTransform(
                Gf.Rotation(Gf.Quatd(explicit_orientation or Gf.Quatf(1.0))),
                body_pos,
            )
            link_world = UsdGeom.Xformable(
                stage.GetPrimAtPath(parent_link_path)
            ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            visual_root.AddTransformOp().Set(rest_world * link_world.GetInverse())
            sphere = UsdGeom.Sphere.Define(stage, f"{visual_root.GetPath()}/Sphere")
            sphere.CreateRadiusAttr().Set(radius)
            sphere.CreateDisplayColorAttr().Set(
                [PlantColors.tomato_color(body.get("maturation", 0.0))]
            )
            material = get_or_create_tomato_fruit_material(
                stage, body.get("maturation", 0.0)
            )
            from pxr import UsdShade

            UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(material)
            prim = visual_root.GetPrim()
            prim.CreateAttribute(
                "autotom:entityKind", Sdf.ValueTypeNames.String, custom=True
            ).Set("terminal_visual")
            for source, attribute in (
                ("canonical_organ_id", "autotom:canonicalOrganId"),
                ("canonical_primitive_id", "autotom:canonicalPrimitiveId"),
            ):
                if body.get(source) is not None:
                    prim.CreateAttribute(
                        attribute, Sdf.ValueTypeNames.String, custom=True
                    ).Set(str(body[source]))
            terminal_body_records.append(
                {
                    "id": body["id"],
                    "path": str(visual_root.GetPath()),
                    "parent_branch_id": parent_branch_id,
                    "pos": tuple(float(value) for value in body_pos),
                    "radius": radius,
                    "physical": False,
                    "exclude_from_articulation": False,
                }
            )
            continue
        (
            detachment_enabled,
            break_force,
            exclude_from_articulation,
            body_parent_path,
        ) = _resolve_terminal_body_attachment(body, stem_path)
        if body_parent_path != stem_path:
            UsdGeom.Xform.Define(stage, body_parent_path)

        if shape == "sphere":
            maturation = body.get("maturation", 0.0)
            tomato_material = get_or_create_tomato_fruit_material(stage, maturation)
            local_pos0 = None
            local_pos1 = None
            local_rot1 = None

            if is_pedicel:
                cylinder = UsdGeom.Cylinder.Get(stage, f"{parent_link_path}/Cylinder")
                if cylinder:
                    cylinder.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
                    pedicel_filtered = UsdPhysics.FilteredPairsAPI(cylinder.GetPrim())
                    if not pedicel_filtered:
                        pedicel_filtered = UsdPhysics.FilteredPairsAPI.Apply(
                            cylinder.GetPrim()
                        )
                    pedicel_filtered.GetFilteredPairsRel().AddTarget("/World/Stem")

                parent_world_to_local = parent_orientation.GetInverse()
                parent_rotation = Gf.Rotation(parent_world_to_local)
                gravity_local = parent_rotation.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
                centers, tangents = sample_gravity_elbow(
                    parent_height, parent_branch_id, gravity_local
                )
                if not stage.GetPrimAtPath(
                    f"{parent_link_path}/GravityElbowPedicelVisual"
                ):
                    create_gravity_elbow_mesh(
                        stage,
                        parent_link_path,
                        centers,
                        tangents,
                        scaled(branch_defs[parent_branch_id]["radius"]),
                        parent_branch_id,
                    )

                tip_local = centers[-1]
                terminal_down_local = Gf.Vec3d(*tangents[-1]).GetNormalized()
                visual_overlap = 0.002
                parent_fwd_rotation = Gf.Rotation(parent_orientation)
                local_pos0 = Gf.Vec3f(*tip_local)
                if explicit_center is None:
                    tomato_center_local = tip_local + terminal_down_local * (
                        radius - visual_overlap
                    )
                    body_pos = parent_base + parent_fwd_rotation.TransformDir(
                        tomato_center_local
                    )
                    local_pos1 = Gf.Vec3f(
                        *(-terminal_down_local * (radius - visual_overlap))
                    )
                else:
                    tip_world = parent_base + parent_fwd_rotation.TransformDir(tip_local)
                    child_rotation = Gf.Rotation(Gf.Quatd(physical_orientation))
                    local_pos1 = Gf.Vec3f(
                        *child_rotation.GetInverse().TransformDir(tip_world - body_pos)
                    )
                    # FixedJoint frames must coincide in world space at the
                    # canonical rest pose.  A GroIMP sphere frame is generally
                    # not aligned with its pedicel, so identity/identity would
                    # inject a large angular correction and can break the 6 N
                    # historical attachment on mature plants.
                    local_rot1 = Gf.Quatf(
                        physical_orientation.GetInverse() * parent_orientation
                    )

            authored_orientation = physical_orientation
            body_path = create_sphere_rigid_body(
                stage,
                body_parent_path,
                body["id"],
                radius,
                body_pos,
                mass,
                orientation=authored_orientation,
                color=PlantColors.tomato_color(maturation),
                material=tomato_material,
            )
            body_prim = stage.GetPrimAtPath(body_path)
            body_prim.CreateAttribute(
                "autotom:entityKind", Sdf.ValueTypeNames.String, custom=True
            ).Set("terminal_body")
            body_prim.CreateAttribute(
                "autotom:role", Sdf.ValueTypeNames.String, custom=True
            ).Set("fruit")
            for source, attribute in (
                ("canonical_organ_id", "autotom:canonicalOrganId"),
                ("canonical_primitive_id", "autotom:canonicalPrimitiveId"),
            ):
                if body.get(source) is not None:
                    body_prim.CreateAttribute(
                        attribute, Sdf.ValueTypeNames.String, custom=True
                    ).Set(str(body[source]))

            if is_pedicel and explicit_center is None:
                tomato_prim = stage.GetPrimAtPath(body_path)
                tomato_filtered = UsdPhysics.FilteredPairsAPI(tomato_prim)
                if not tomato_filtered:
                    tomato_filtered = UsdPhysics.FilteredPairsAPI.Apply(tomato_prim)
                tomato_filtered.GetFilteredPairsRel().AddTarget("/World/Stem")

            if exclude_from_articulation:
                apply_physx_rigid_body_solver_settings(stage, body_path)

            create_fixed_joint_to_tip(
                stage,
                parent_link_path,
                body_path,
                parent_height=parent_height,
                child_offset=child_offset,
                joint_name="TerminalBodyFixedJoint",
                break_force=break_force,
                exclude_from_articulation=exclude_from_articulation,
                local_pos0=local_pos0,
                local_pos1=local_pos1,
                local_rot1=local_rot1,
            )
            terminal_body_records.append(
                {
                    "id": body["id"],
                    "path": body_path,
                    "parent_branch_id": parent_branch_id,
                    "pos": (body_pos[0], body_pos[1], body_pos[2]),
                    "radius": radius,
                    "exclude_from_articulation": exclude_from_articulation,
                    "physical": True,
                    "canonical_organ_id": body.get("canonical_organ_id"),
                    "canonical_primitive_id": body.get("canonical_primitive_id"),
                    "source_center": body.get("source_center"),
                    "source_frame": body.get("source_frame", body.get("rest_frame")),
                    "authored_frame": body.get("rest_frame"),
                    "authored_orientation": [
                        float(authored_orientation.GetReal()),
                        *(
                            float(value)
                            for value in authored_orientation.GetImaginary()
                        ),
                    ],
                }
            )

            if OutputConfig.STEP_1_VERBOSE:
                break_force_label = (
                    "disabled" if break_force is None else f"{break_force:.2f}N"
                )
                print(
                    f"[INFO] terminal body '{body['id']}': sphere r={radius:.3f}m, "
                    f"parent='{parent_branch_id}', body_parent='{body_parent_path}', "
                    f"detachment={'enabled' if detachment_enabled else 'disabled'}, "
                    f"break_force={break_force_label}"
                )
        else:
            roll_deg = body.get("roll", 0.0)
            if roll_deg != 0.0:
                half = math.radians(roll_deg) / 2.0
                local_rot = Gf.Quatf(
                    math.cos(half), Gf.Vec3f(0.0, 0.0, math.sin(half))
                )
            else:
                local_rot = None

            create_static_mesh(
                stage,
                parent_link_path,
                body["id"],
                points=body.get("points", []),
                indices=body.get("indices", []),
                face_vertex_counts=body.get("face_vertex_counts", []),
                local_pos=Gf.Vec3d(0, 0, parent_height),
                local_rot=local_rot,
                color=PlantColors.LEAF_BLADE,
            )

            if OutputConfig.STEP_1_VERBOSE:
                print(
                    f"[INFO] terminal body '{body['id']}': mesh, "
                    f"parent='{parent_branch_id}', body_parent='{body_parent_path}'"
                )

    return terminal_body_records


def filter_external_terminal_body_collisions(
    stage, terminal_body_records, branch_registry, branch_defs
):
    """Filter detached fruit against its pedicel and rachis chains."""
    for record in terminal_body_records:
        if not record.get("physical", True):
            continue
        if not record.get("exclude_from_articulation"):
            continue

        tomato_path = record["path"]
        parent_branch_id = record["parent_branch_id"]
        if parent_branch_id in branch_registry:
            for link_path in branch_registry[parent_branch_id][0]:
                add_collision_filter(stage, tomato_path, link_path)

        parent_def = branch_defs.get(parent_branch_id, {})
        grandparent_id = parent_def.get("parent")
        if grandparent_id and grandparent_id in branch_registry:
            for link_path in branch_registry[grandparent_id][0]:
                add_collision_filter(stage, tomato_path, link_path)
