"""Read and write ``plant_skeleton/1.0`` JSON files."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, Mapping

from .model import (
    SKELETON_SCHEMA_VERSION,
    PlantSkeleton,
    SkeletonFruit,
    SkeletonLeaflet,
    SkeletonMetadata,
    SkeletonNode,
)
from .validation import validate_skeleton


class SkeletonFormatError(ValueError):
    """Raised when the JSON does not have the expected shape."""


_NODE_REQUIRED = {"id", "parent", "kind", "start", "end", "radius"}
_NODE_OPTIONAL = {"frame", "axis", "organ"}
_LEAFLET_REQUIRED = {"id", "node", "side", "start", "end", "radius"}
_LEAFLET_OPTIONAL = {"frame", "inclination_deg", "mass_kg"}
_FRUIT_REQUIRED = {"id", "node", "radius"}
_FRUIT_OPTIONAL = {"ripeness", "center"}
_METADATA_FIELDS = {"plant_id", "source", "description", "time"}
_TOP_REQUIRED = {"schema_version", "nodes"}
_TOP_OPTIONAL = {"metadata", "leaflets", "fruits"}


def _check_keys(value: Any, required: set[str], optional: set[str], where: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise SkeletonFormatError(f"{where} must be an object")
    missing = sorted(required - set(value))
    unknown = sorted(set(value) - required - optional)
    if missing:
        raise SkeletonFormatError(f"{where} is missing {missing}")
    if unknown:
        allowed = sorted(required | optional)
        raise SkeletonFormatError(f"{where} has unknown fields {unknown}; allowed: {allowed}")
    return value


def _vector(value: Any, where: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise SkeletonFormatError(f"{where} must be [x, y, z]")
    try:
        return tuple(float(item) for item in value)  # type: ignore[return-value]
    except (TypeError, ValueError) as exc:
        raise SkeletonFormatError(f"{where} must contain numbers") from exc


def _frame(value: Any, where: str):
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 3:
        raise SkeletonFormatError(f"{where} must be a 3x3 array of rows")
    return tuple(_vector(row, f"{where} row {index}") for index, row in enumerate(value))


def _optional_float(value: Any, where: str) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise SkeletonFormatError(f"{where} must be a number") from exc


def skeleton_from_dict(data: Mapping[str, Any], *, validate: bool = True) -> PlantSkeleton:
    data = _check_keys(data, _TOP_REQUIRED, _TOP_OPTIONAL, "skeleton")
    if data["schema_version"] != SKELETON_SCHEMA_VERSION:
        raise SkeletonFormatError(
            f"schema_version must be {SKELETON_SCHEMA_VERSION!r}, got {data['schema_version']!r}"
        )
    meta = _check_keys(data.get("metadata", {}), set(), _METADATA_FIELDS, "metadata")
    metadata = SkeletonMetadata(
        plant_id=int(meta.get("plant_id", 1)),
        source=str(meta.get("source", "unknown")),
        description=str(meta.get("description", "")),
        time=_optional_float(meta.get("time"), "metadata.time"),
    )
    nodes = []
    for index, raw in enumerate(data["nodes"]):
        where = f"nodes[{index}]"
        raw = _check_keys(raw, _NODE_REQUIRED, _NODE_OPTIONAL, where)
        nodes.append(
            SkeletonNode(
                id=str(raw["id"]),
                parent=None if raw["parent"] is None else str(raw["parent"]),
                kind=str(raw["kind"]),
                start=_vector(raw["start"], f"{where}.start"),
                end=_vector(raw["end"], f"{where}.end"),
                radius=float(raw["radius"]),
                frame=_frame(raw.get("frame"), f"{where}.frame"),
                axis=None if raw.get("axis") is None else str(raw["axis"]),
                organ=None if raw.get("organ") is None else str(raw["organ"]),
            )
        )
    leaflets = []
    for index, raw in enumerate(data.get("leaflets", [])):
        where = f"leaflets[{index}]"
        raw = _check_keys(raw, _LEAFLET_REQUIRED, _LEAFLET_OPTIONAL, where)
        leaflets.append(
            SkeletonLeaflet(
                id=str(raw["id"]),
                node=str(raw["node"]),
                side=str(raw["side"]),
                start=_vector(raw["start"], f"{where}.start"),
                end=_vector(raw["end"], f"{where}.end"),
                radius=float(raw["radius"]),
                frame=_frame(raw.get("frame"), f"{where}.frame"),
                inclination_deg=_optional_float(raw.get("inclination_deg"), f"{where}.inclination_deg"),
                mass_kg=float(raw.get("mass_kg", 0.0)),
            )
        )
    fruits = []
    for index, raw in enumerate(data.get("fruits", [])):
        where = f"fruits[{index}]"
        raw = _check_keys(raw, _FRUIT_REQUIRED, _FRUIT_OPTIONAL, where)
        fruits.append(
            SkeletonFruit(
                id=str(raw["id"]),
                node=str(raw["node"]),
                radius=float(raw["radius"]),
                ripeness=float(raw.get("ripeness", 0.0)),
                center=None if raw.get("center") is None else _vector(raw["center"], f"{where}.center"),
            )
        )
    skeleton = PlantSkeleton(
        nodes=tuple(nodes),
        leaflets=tuple(leaflets),
        fruits=tuple(fruits),
        metadata=metadata,
    )
    return validate_skeleton(skeleton) if validate else skeleton


def _clean(value: Any) -> Any:
    """Drop optional fields that are None so files stay short."""

    if isinstance(value, dict):
        return {key: _clean(item) for key, item in value.items() if item is not None or key == "parent"}
    if isinstance(value, (list, tuple)):
        return [_clean(item) for item in value]
    return value


_NUMBER_ARRAY = re.compile(r"\[\s*([-+0-9.eE]+(?:,\s*[-+0-9.eE]+)*)\s*\]")
_ARRAY_OF_ARRAYS = re.compile(r"\[\s*(\[[^\[\]]*\](?:,\s*\[[^\[\]]*\])*)\s*\]")


def _round(value: Any) -> Any:
    if isinstance(value, float):
        return round(value, 10) + 0.0
    if isinstance(value, dict):
        return {key: _round(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_round(item) for item in value]
    return value


def skeleton_json(skeleton: PlantSkeleton) -> str:
    """Pretty JSON with vectors and frames kept on one line each."""

    text = json.dumps(_round(skeleton_to_dict(skeleton)), indent=2, allow_nan=False)
    text = _NUMBER_ARRAY.sub(lambda m: "[" + ", ".join(part.strip() for part in m.group(1).split(",")) + "]", text)
    text = _ARRAY_OF_ARRAYS.sub(lambda m: "[" + ", ".join(part.strip() for part in re.findall(r"\[[^\[\]]*\]", m.group(1))) + "]", text)
    return text + "\n"


def skeleton_to_dict(skeleton: PlantSkeleton) -> dict[str, Any]:
    data = _clean(skeleton.to_dict())
    ordered = {"schema_version": data.pop("schema_version"), "metadata": data.pop("metadata")}
    ordered.update(data)
    return ordered


def load_skeleton(path: str | Path, *, validate: bool = True) -> PlantSkeleton:
    path = Path(path)
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SkeletonFormatError(f"{path} is not valid JSON: {exc}") from exc
    return skeleton_from_dict(data, validate=validate)


def save_skeleton(skeleton: PlantSkeleton, path: str | Path) -> Path:
    validate_skeleton(skeleton)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(skeleton_json(skeleton), encoding="utf-8")
    return path
