"""End-to-end exports. Leaves use the ``simple`` generator so no worker starts,
except the baked-leaf tests, which reuse the committed default set."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re

import pytest
from pxr import Usd, UsdGeom, UsdPhysics

from exporterV2 import cli
from exporterV2.config import physics_config_from_dict, visuals_config_from_dict
from exporterV2.exporter import ExportError, export_plant
from plant_skeleton import load_skeleton


PROJECT_ROOT = Path(__file__).resolve().parents[3]
SMALL_PLANT = PROJECT_ROOT / "input" / "examples" / "small_plant.json"
DAY_160 = PROJECT_ROOT / "input" / "skeleton.json"
REFERENCE = Path(__file__).parent / "data" / "day160_reference.json"
SIMPLE_LEAVES = {"leaves": {"generator": "simple"}}


def _export(tmp_path, skeleton_path=SMALL_PLANT, physics=None, visuals=None, name="plant.usda"):
    return export_plant(
        load_skeleton(skeleton_path),
        tmp_path / name,
        physics_config_from_dict(physics),
        visuals_config_from_dict({**SIMPLE_LEAVES, **(visuals or {})}),
    )


def test_small_hand_written_plant(tmp_path):
    result = _export(tmp_path)
    authored = result.manifest["authored"]
    assert result.manifest["errors"] == []
    assert authored["rigid_bodies"] == 16
    assert authored["fixed_joints"] == 4  # the stem
    assert authored["d6_joints"] == 12
    assert authored["leaflets"] == authored["leaf_blades"] == 6
    assert authored["fruit_spheres"] == 3
    assert authored["terminal_forks"] == 1  # leaf B ends the stem
    stage = Usd.Stage.Open(str(result.usd_path))
    stem = stage.GetPrimAtPath("/World/Stem")
    assert stem.GetAttribute("autotom:skeletonSchema").Get() == "plant_skeleton/1.0"
    assert stem.GetAttribute("autotom:fruitMode").Get() == "visual"
    assert stem.GetAttribute("autotom:globalScale").Get() == 2.0
    nodes = {prim.GetAttribute("autotom:skeletonNodeId").Get() for prim in stage.Traverse()}
    assert {"stem1", "shoot2", "pedicel3"} <= nodes


def test_welded_stem_is_one_continuous_mesh(tmp_path):
    result = _export(tmp_path)
    assert result.manifest["authored"]["organic_meshes"] == 16 - 3  # 4 stem links -> 1 mesh
    stage = Usd.Stage.Open(str(result.usd_path))
    stem_meshes = [prim for prim in stage.Traverse() if "/trunk/" in str(prim.GetPath()) and prim.GetName().startswith("OrganicVisual_")]
    assert [prim.GetName() for prim in stem_meshes] == ["OrganicVisual_01"]


def test_bending_stem_keeps_one_mesh_per_link(tmp_path):
    result = _export(tmp_path, physics={"segments": {"stem": {"joint": "d6"}}})
    authored = result.manifest["authored"]
    assert result.manifest["errors"] == []
    assert authored["d6_joints"] == 12 + 3
    assert authored["organic_meshes"] == 16


def test_physical_fruits_become_rigid_bodies(tmp_path):
    result = _export(tmp_path, physics={"fruits": {"mode": "physical"}})
    assert result.manifest["authored"]["rigid_bodies"] == 16 + 3


def test_global_scale_changes_every_length(tmp_path):
    two = _export(tmp_path, name="two.usda")
    three = _export(tmp_path, physics={"global_scale": 3.0}, name="three.usda")
    for a, b in zip(two.manifest["poses"], three.manifest["poses"]):
        assert b["length"] == pytest.approx(a["length"] * 1.5)
        assert b["start"] == pytest.approx([value * 1.5 for value in a["start"]])


def test_too_many_joints_asks_for_optimization(tmp_path):
    with pytest.raises(ExportError, match="joints.optimize"):
        _export(tmp_path, physics={"joints": {"joint_target": 5, "joint_max": 8}})
    result = _export(tmp_path, physics={"joints": {"joint_target": 8, "joint_max": 8, "optimize": "full"}})
    assert result.manifest["authored"]["post_merge"]["d6_joints"] <= 8


def test_cli_defaults_and_errors(tmp_path, capsys):
    output = tmp_path / "cli.usda"
    visuals = tmp_path / "visuals.yaml"
    visuals.write_text("leaves:\n  generator: simple\n")
    assert cli.main(["--input", str(SMALL_PLANT), "--visuals", str(visuals), "--output", str(output)]) == 0
    assert output.is_file() and output.with_suffix(".manifest.json").is_file()
    assert "16 segments -> 16 rigid bodies" in capsys.readouterr().out
    assert cli.build_argument_parser().parse_args([]).input == PROJECT_ROOT / "input" / "skeleton.json"
    broken = tmp_path / "broken.json"
    broken.write_text('{"schema_version": "plant_skeleton/1.0", "nodes": []}')
    assert cli.main(["--input", str(broken), "--output", str(output)]) == 1
    assert "has no nodes" in capsys.readouterr().err


@pytest.fixture(scope="module")
def day160(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("day160")
    # The reference was exported at the old 30 MPa vegetative modulus; pin it
    # so this stays a parity check of the exporter, not of the defaults.
    vegetative = {"young_modulus": 30.0e6}
    segments = {kind: vegetative for kind in ("stem", "branch", "petiole", "leaf_rachis")}
    # The reference also keeps the skeleton's pedicel lengths.
    return _export(
        tmp,
        DAY_160,
        physics={"fruits": {"mode": "none"}, "segments": segments},
        visuals={"trusses": {"pedicel_length_scale": 1.0}},
    )


def test_day_160_matches_the_reference_export(day160):
    """Same rigid links, poses, masses and joint gains as the reference export of
    the default plant (made with the earlier exporter, before the skeleton format)."""

    reference = json.loads(REFERENCE.read_text())
    authored = day160.manifest["authored"]
    expected_counts = reference["counts"]
    assert authored["rigid_bodies"] == expected_counts["rigid_bodies"]
    assert authored["fixed_joints"] == expected_counts["fixed_joints"]
    assert authored["d6_joints"] == expected_counts["d6_joints"]
    assert authored["capsule_colliders"] == expected_counts["capsule_colliders"]
    # The welded stem is now one continuous mesh instead of one per link.
    stem_links = sum("/trunk/" in pose["body_path"] for pose in day160.manifest["poses"])
    merged = stem_links - 1
    assert authored["organic_meshes"] == expected_counts["organic_meshes"] - merged
    assert authored["leaf_blades"] == expected_counts["leaf_blades"]
    assert authored["petiolule_meshes"] == expected_counts["petiolule_visual_meshes"]
    assert authored["terminal_forks"] == expected_counts["terminal_fork_shoots"]
    assert authored["total_meshes"] == expected_counts["total_meshes"] - merged
    assert authored["truss_visuals"] == reference["truss_visuals"]

    poses = sorted(pose["start"] + pose["direction"] for pose in day160.manifest["poses"])
    for ours, theirs in zip(poses, reference["links_start_direction"]):
        assert ours == pytest.approx(theirs, abs=1e-8)

    masses = sorted(body["mass_kg"] for body in day160.manifest["physics"]["body_masses"])
    # USD stores masses as 32-bit floats.
    assert masses == pytest.approx(reference["body_masses_kg"], rel=1e-6)

    usda = day160.usd_path.read_text()
    stiffness = sorted(float(x) for x in re.findall(r"drive:rot[XY]:physics:stiffness = ([0-9.e+-]+)", usda))
    damping = sorted(float(x) for x in re.findall(r"drive:rot[XY]:physics:damping = ([0-9.e+-]+)", usda))
    assert stiffness == pytest.approx(reference["drive_stiffness"], rel=1e-6)
    assert damping == pytest.approx(reference["drive_damping"], rel=1e-6)


def test_leaflets_hang_from_their_rachis(day160):
    stage = Usd.Stage.Open(str(day160.usd_path))
    for prim in stage.Traverse():
        if prim.GetAttribute("autotom:entityKind").Get() != "rigid_leaf_visual":
            continue
        assert prim.GetParent().GetParent().HasAPI(UsdPhysics.RigidBodyAPI)
        assert prim.GetChild("LeafBlade").IsA(UsdGeom.Mesh)  # simple generator


@pytest.mark.parametrize(
    "visuals",
    [
        {"stems": {"junction_bulge": False}},
        {"stems": {"terminal_forks": False}},
        {"pedicels": {"gravity_curve": False}},
        {"quality": "performance"},
        {"pose": "canonical"},
    ],
)
def test_every_visual_switch_exports(tmp_path, visuals):
    result = _export(tmp_path, visuals=visuals)
    assert result.manifest["errors"] == []
    assert result.manifest["authored"]["rigid_bodies"] == 16


def test_fruit_size_and_calyx_are_visual_only(tmp_path):
    def spheres(result):
        stage = Usd.Stage.Open(str(result.usd_path))
        return {
            prim.GetParent().GetName(): (UsdGeom.Sphere(prim).GetRadiusAttr().Get(), prim.GetParent().GetChild("Calyx").IsValid())
            for prim in stage.Traverse()
            if prim.IsA(UsdGeom.Sphere)
        }

    physical = {"fruits": {"mode": "physical"}}
    default = _export(tmp_path, physics=physical, name="default.usda")
    true_size = _export(tmp_path, physics=physical, visuals={"fruits": {"size_scale": 1.0, "calyx": False}}, name="true.usda")
    assert default.manifest["authored"]["calyces"] == 3
    assert true_size.manifest["authored"]["calyces"] == 0
    for name, (radius, has_calyx) in spheres(default).items():
        full_radius, full_calyx = spheres(true_size)[name]
        assert radius == pytest.approx(full_radius * 0.5)
        assert has_calyx and not full_calyx
    masses = lambda result: sorted(body["mass_kg"] for body in result.manifest["physics"]["body_masses"])
    assert masses(default) == pytest.approx(masses(true_size))


BAKED_DEFAULT_SET = {"leaves": {"generator": "baked", "one_per_leaflet": False}}


def test_baked_leaves_reference_assets_and_droop(tmp_path):
    result = _export(tmp_path, visuals=BAKED_DEFAULT_SET)
    assert result.manifest["errors"] == []
    assert result.manifest["authored"]["leaf_blades"] == 6
    baked = result.manifest["metadata"]["baked_leaves"]
    assert baked["unique"] is False and len(baked["leaves"]) == 6
    stage = Usd.Stage.Open(str(result.usd_path))
    blades = [prim for prim in stage.Traverse() if prim.GetName() == "LeafBlade"]
    assert len(blades) == 6
    for prim in blades:
        assert prim.IsInstance() and prim.GetAttribute("autotom:leafAsset").Get().startswith("leaf_")
        droop = prim.GetAttribute("autotom:leafDroopDeg").Get()
        assert 12.0 <= droop <= 32.0
        # The blade (+Y of the asset) leaves the petiolule tip tipped down by
        # about the droop angle relative to the petiolule.
        blade = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        root = UsdGeom.Xformable(prim.GetParent()).ComputeLocalToWorldTransform(0)
        forward = blade.TransformDir((0, 1, 0)).GetNormalized()
        stalk = root.TransformDir((0, 0, 1)).GetNormalized()
        assert math.degrees(math.acos(max(-1.0, min(1.0, forward * stalk)))) == pytest.approx(droop, abs=1e-3)
    no_droop = _export(
        tmp_path,
        visuals={"leaves": {**BAKED_DEFAULT_SET["leaves"], "droop_deg": 0.0, "droop_jitter_deg": 0.0}},
        name="flat.usda",
    )
    stage = Usd.Stage.Open(str(no_droop.usd_path))
    assert {prim.GetAttribute("autotom:leafDroopDeg").Get() for prim in stage.Traverse() if prim.GetName() == "LeafBlade"} == {0.0}


def test_one_leaf_per_leaflet_from_the_default_set(tmp_path):
    result = _export(tmp_path, visuals={"leaves": {"generator": "baked"}})
    leaves = result.manifest["metadata"]["baked_leaves"]
    assert leaves["unique"] is True
    assert len(set(leaves["leaves"].values())) == 6


def test_stem_colour(tmp_path):
    color = [0.1, 0.2, 0.05]
    result = _export(tmp_path, visuals={"stems": {"color": color}})
    stage = Usd.Stage.Open(str(result.usd_path))
    stems = [prim for prim in stage.Traverse() if prim.GetName().startswith("OrganicVisual_")]
    assert stems
    for prim in stems + [prim for prim in stage.Traverse() if prim.GetName() == "GravityElbowPedicelVisual"]:
        assert list(UsdGeom.Mesh(prim).GetDisplayColorAttr().Get()[0]) == pytest.approx(color)
    plain = _export(tmp_path, visuals={"stems": {"color": None}}, name="plain.usda")
    stage = Usd.Stage.Open(str(plain.usd_path))
    first = next(prim for prim in stage.Traverse() if prim.GetName().startswith("OrganicVisual_"))
    assert list(UsdGeom.Mesh(first).GetDisplayColorAttr().Get()[0]) != pytest.approx(color)


def test_branch_collisions(day160, tmp_path):
    stage = Usd.Stage.Open(str(day160.usd_path))
    assert stage.GetPrimAtPath("/World/Stem").GetAttribute("physxArticulation:enabledSelfCollisions").Get() is True
    assert day160.manifest["collisions"]["branch_contacts"] > 10000
    assert day160.manifest["collisions"]["active_at_rest"] == []
    off = _export(tmp_path, physics={"collisions": {"branches": False}})
    stage = Usd.Stage.Open(str(off.usd_path))
    assert stage.GetPrimAtPath("/World/Stem").GetAttribute("physxArticulation:enabledSelfCollisions").Get() is False
    assert off.manifest["collisions"]["branch_contacts"] is None


def test_pedicel_length_scale_moves_fruit_along_pedicel():
    from plant_skeleton import load_skeleton
    from exporterV2.skeleton_branches import lengthen_pedicels

    skeleton = load_skeleton(DAY_160)
    longer = lengthen_pedicels(skeleton, 1.75)
    for old, new in zip(skeleton.nodes, longer.nodes):
        assert old.start == new.start
        if old.kind == "pedicel":
            old_length = math.dist(old.start, old.end)
            assert math.isclose(math.dist(new.start, new.end), 1.75 * old_length, rel_tol=1e-9)
        else:
            assert old.end == new.end
    assert lengthen_pedicels(skeleton, 1.0) is skeleton


def test_visual_fruits_hang_from_their_pedicel_link(tmp_path):
    result = _export(tmp_path)
    stage = Usd.Stage.Open(str(result.usd_path))
    fruits = [prim for prim in stage.Traverse() if prim.GetAttribute("autotom:entityKind").Get() == "terminal_visual"]
    assert fruits
    for fruit in fruits:
        link = fruit.GetParent()
        assert link.HasAPI(UsdPhysics.RigidBodyAPI)
        assert "pedicel" in link.GetName()
        assert not fruit.HasAPI(UsdPhysics.RigidBodyAPI)
