"""The pre-generated leaflet set loads, picks deterministically and references at scale."""
import json

import pytest
from pxr import Usd, UsdGeom

from exporterV2.leaf_shapes.baked import DEFAULT_SET, load_baked_leaf_set, reference_baked_leaf

pytestmark = pytest.mark.skipif(not (DEFAULT_SET / "manifest.json").is_file(), reason="no baked set")


def test_default_set_is_complete_and_closed():
    leaf_set = load_baked_leaf_set()
    assert leaf_set.length_m == pytest.approx(0.06)
    leaf = leaf_set.leaves[0]
    stage = Usd.Stage.Open(str(leaf_set.path(leaf)))
    mesh = UsdGeom.Mesh(stage.GetDefaultPrim().GetChild("Blade"))
    indices = list(mesh.GetFaceVertexIndicesAttr().Get())
    edges = {}
    for i in range(0, len(indices), 3):
        a, b, c = indices[i:i + 3]
        for u, v in ((a, b), (b, c), (c, a)):
            assert (u, v) not in edges, "inconsistent winding"
            edges[(u, v)] = True
    assert all((v, u) in edges for u, v in edges), "mesh is not closed"
    assert UsdGeom.Subset.ValidateFamily(mesh, UsdGeom.Tokens.face, "materialBind")[0]
    for side in ("top", "bottom"):
        for kind in ("albedo", "normal"):
            assert (leaf_set.folder / f"textures/{leaf['file'][:-5]}_{side}_{kind}.png").is_file()


def test_pick_is_deterministic_and_reference_is_scaled(tmp_path):
    leaf_set = load_baked_leaf_set()
    first = leaf_set.pick("L1/left", "left", plant_id="p", seed=42)
    assert first == leaf_set.pick("L1/left", "left", plant_id="p", seed=42)
    picks = {leaf_set.pick(f"L{i}", "terminal", plant_id="p", seed=42)["file"] for i in range(40)}
    assert len(picks) > 10
    stage = Usd.Stage.CreateNew(str(tmp_path / "scene.usda"))
    prim = reference_baked_leaf(stage, "/Leaf", leaf_set, first, 0.045)
    stage.Save()
    bound = UsdGeom.BBoxCache(0, ["default"]).ComputeWorldBound(prim).ComputeAlignedRange()
    tip_y = json.loads((leaf_set.folder / "manifest.json").read_text())["length_m"] * 0.75
    assert bound.GetMax()[1] == pytest.approx(tip_y, abs=0.004)
    assert prim.GetAttribute("autotom:leafAsset").Get() == first["file"]


def test_default_set_gives_unique_leaves_when_large_enough():
    from exporterV2.leaf_shapes import baked

    size = len(baked.load_baked_leaf_set().leaves)
    assert baked.unique_leaf_set(size, 42).folder == baked.DEFAULT_SET
    assert baked.unique_leaf_set(size + 1, 42) is None
    assert baked.unique_leaf_set(size, 7) is None
