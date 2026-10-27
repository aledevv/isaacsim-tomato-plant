"""Stable per-leaflet seeds and the record each blade carries into the USD."""
from dataclasses import dataclass
import hashlib
import json


def leaflet_seed(global_seed, plant_id, structural_id, role):
    if role not in ("left", "right", "terminal"):
        raise ValueError(f"invalid leaflet role: {role!r}")
    payload = json.dumps(["autotom-leaf-seed-v1", global_seed, plant_id, structural_id, role], separators=(",", ":"), ensure_ascii=True)
    return int.from_bytes(hashlib.sha256(payload.encode()).digest()[:8], "big") & ((1 << 63)-1)


@dataclass
class PreparedLeafShapes:
    seed: int
    shapes: dict

    def for_leaf(self, structural_id):
        if structural_id not in self.shapes:
            raise ValueError(f"leaf shape not prepared for {structural_id}")
        return self.shapes[structural_id]

    def manifest(self):
        return {"seed": self.seed, "leaves": [shape for _, shape in sorted(self.shapes.items())]}


def prepare_leaf_shapes(leaves, *, plant_id, seed):
    """One record (id, role, seed) per ``(structural_id, role)`` leaflet."""
    shapes = {}
    for structural_id, role in leaves:
        if structural_id in shapes:
            raise ValueError(f"duplicate leaflet identity: {structural_id}")
        shapes[structural_id] = {
            "id": structural_id,
            "role": role,
            "seed": leaflet_seed(seed, plant_id, structural_id, role),
            "backend": "legacy",
        }
    return PreparedLeafShapes(seed, shapes)


def author_shape_manifest(stage, prepared):
    from pxr import Sdf
    root = stage.GetDefaultPrim()
    root.CreateAttribute("autotom:leafShapes", Sdf.ValueTypeNames.String, custom=True).Set(
        json.dumps(prepared.manifest(), sort_keys=True, allow_nan=False)
    )
