"""Load the pre-baked leaflet assets; no generation at export or Isaac startup.

A baked set is a folder with ``manifest.json``, one ``leaf_NNN.usda`` per leaflet and their
textures. Each asset is in metres, Z up, attachment at the origin, tip towards +Y, upper side
towards +Z, built at ``manifest["length_m"]`` base-tip length. See docs/11_baked_leaf_assets.md.
"""
from dataclasses import dataclass
import json
import os
from pathlib import Path

from .seeds import leaflet_seed

PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SET = PROJECT_ROOT / "assets/leaves/default"


@dataclass(frozen=True)
class BakedLeafSet:
    folder: Path
    manifest: dict

    @property
    def length_m(self):
        return float(self.manifest["length_m"])

    @property
    def leaves(self):
        return self.manifest["leaves"]

    def path(self, leaf):
        return self.folder / leaf["file"]

    def pick(self, structural_id, role, *, plant_id, seed):
        """The asset for one leaflet: stable for a given skeleton identity and global seed."""
        return self.leaves[leaflet_seed(seed, plant_id, structural_id, role) % len(self.leaves)]


def load_baked_leaf_set(folder=None):
    folder = Path(folder).expanduser().resolve() if folder is not None else DEFAULT_SET
    manifest_path = folder / "manifest.json"
    if not manifest_path.is_file():
        raise ValueError(
            f"no baked leaf set at {folder} (see src/exporterV2/docs/11_baked_leaf_assets.md)"
        )
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("schema") != 1 or not manifest.get("leaves"):
        raise ValueError(f"unsupported or empty baked leaf manifest: {manifest_path}")
    missing = [leaf["file"] for leaf in manifest["leaves"] if not (folder / leaf["file"]).is_file()]
    if missing:
        raise ValueError(f"baked leaf set {folder} is missing {missing[:3]}")
    return BakedLeafSet(folder, manifest)


def unique_leaf_set(count, seed):
    """The default set when it was baked with ``seed`` and has at least ``count`` leaflets, so
    every leaflet can get its own asset; else None (assets are then reused across leaflets)."""
    leaf_set = load_baked_leaf_set()
    settings = leaf_set.manifest.get("settings", {})
    if settings.get("seed") == seed and len(leaf_set.leaves) >= count:
        return leaf_set
    if settings.get("seed") != seed:
        print(f"[WARNING] the default leaf set was baked with seed {settings.get('seed')} "
              f"(leaves.seed is {seed}): leaves are reused across leaflets")
    else:
        print(f"[WARNING] {count} leaflets but {len(leaf_set.leaves)} baked leaves with seed "
              f"{settings.get('seed')} in {DEFAULT_SET}: reusing them across leaflets")
    return None


@dataclass(frozen=True)
class BakedLeaves:
    """The baked asset chosen for every leaflet of one plant."""
    leaf_set: BakedLeafSet
    assets: dict  # leaflet id -> manifest entry
    unique: bool  # True when every leaflet has its own asset

    def for_leaf(self, leaflet_id):
        return self.assets[leaflet_id]

    def manifest(self):
        return {"folder": str(self.leaf_set.folder), "unique": self.unique,
                "settings": self.leaf_set.manifest.get("settings", {}),
                "leaves": {key: leaf["file"] for key, leaf in sorted(self.assets.items())}}


def prepare_baked_leaves(leaflets, *, plant_id, seed, one_per_leaflet=True, folder=None):
    """Pick an asset for each ``(leaflet_id, role)``.

    ``folder`` set: use that baked set, reused across leaflets. Otherwise, with
    ``one_per_leaflet``, every leaflet gets its own asset from the default set when it is large
    enough: leaflets are ordered by their leaflet seed and leaflet ``k`` gets ``leaf_k``, so the
    result does not depend on the skeleton's node order. Otherwise (or with
    ``one_per_leaflet: false``) the default set is reused across leaflets.
    """
    keys = {}
    for leaflet_id, role in leaflets:
        keys.setdefault(leaflet_id, leaflet_seed(seed, plant_id, leaflet_id, role))
    leaf_set = None
    if folder is None and one_per_leaflet and keys:
        leaf_set = unique_leaf_set(len(keys), seed)
    if leaf_set is not None:
        order = sorted(keys, key=keys.get)
        return BakedLeaves(leaf_set, {key: leaf_set.leaves[k] for k, key in enumerate(order)}, True)
    leaf_set = load_baked_leaf_set(folder)
    return BakedLeaves(leaf_set, {key: leaf_set.leaves[value % len(leaf_set.leaves)]
                                  for key, value in keys.items()}, False)


def reference_baked_leaf(stage, prim_path, leaf_set, leaf, blade_length_m, frame=None):
    """Author an Xform at ``prim_path`` that references one baked leaflet, scaled uniformly so
    its base-tip length is ``blade_length_m``. ``frame`` (a ``Gf.Matrix4d`` in the parent's
    space, rows = asset X, Y, Z axes and origin) places it; without it, place and orient the
    Xform's parent so the origin sits at the petiolule tip, +Y along the blade, +Z up."""
    import os

    from pxr import Gf, Sdf, UsdGeom

    xform = UsdGeom.Xform.Define(stage, prim_path)
    prim = xform.GetPrim()
    asset = str(leaf_set.path(leaf))
    layer = stage.GetRootLayer().realPath
    if layer:  # keep the exported stage relocatable together with the asset folder
        asset = os.path.relpath(asset, os.path.dirname(layer))
    prim.GetReferences().AddReference(asset)
    scale = float(blade_length_m) / leaf_set.length_m
    if frame is not None:
        xform.AddTransformOp().Set(Gf.Matrix4d(1.0).SetScale(scale) * frame)
    else:
        xform.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    prim.SetInstanceable(True)
    prim.CreateAttribute("autotom:leafAsset", Sdf.ValueTypeNames.String, custom=True).Set(leaf["file"])
    prim.CreateAttribute("autotom:leafAssetSeed", Sdf.ValueTypeNames.Int64, custom=True).Set(int(leaf["seed"]))
    return prim
