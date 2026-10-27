"""Small vector helpers shared by validation and the exporter."""

from __future__ import annotations

import math
from typing import Sequence

from .model import Matrix3, Vector3


def sub(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return (float(a[0]) - float(b[0]), float(a[1]) - float(b[1]), float(a[2]) - float(b[2]))


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return float(a[0]) * float(b[0]) + float(a[1]) * float(b[1]) + float(a[2]) * float(b[2])


def cross(a: Sequence[float], b: Sequence[float]) -> Vector3:
    return (
        float(a[1]) * float(b[2]) - float(a[2]) * float(b[1]),
        float(a[2]) * float(b[0]) - float(a[0]) * float(b[2]),
        float(a[0]) * float(b[1]) - float(a[1]) * float(b[0]),
    )


def norm(a: Sequence[float]) -> float:
    return math.sqrt(dot(a, a))


def normalized(a: Sequence[float]) -> Vector3:
    length = norm(a)
    if length <= 0.0:
        raise ValueError("cannot normalize a zero vector")
    return (float(a[0]) / length, float(a[1]) / length, float(a[2]) / length)


def segment_length(start: Sequence[float], end: Sequence[float]) -> float:
    return norm(sub(end, start))


def frame_columns(frame: Matrix3) -> tuple[Vector3, Vector3, Vector3]:
    """Return the local X, Y and Z axes stored in a row-major rotation."""

    return tuple(
        (float(frame[0][column]), float(frame[1][column]), float(frame[2][column]))
        for column in range(3)
    )  # type: ignore[return-value]


def frame_from_columns(x: Vector3, y: Vector3, z: Vector3) -> Matrix3:
    return (
        (float(x[0]), float(y[0]), float(z[0])),
        (float(x[1]), float(y[1]), float(z[1])),
        (float(x[2]), float(y[2]), float(z[2])),
    )


def derive_frame(direction: Sequence[float], reference_x: Sequence[float] | None = None) -> Matrix3:
    """Build a right-handed frame whose Z axis follows ``direction``.

    The X axis is ``reference_x`` (usually the parent's X axis) made
    perpendicular to Z, so a chain without explicit frames does not twist.
    """

    z = normalized(direction)
    candidates = []
    if reference_x is not None:
        candidates.append(tuple(float(value) for value in reference_x))
    candidates.extend(((1.0, 0.0, 0.0), (0.0, 1.0, 0.0)))
    for reference in candidates:
        projected = sub(reference, tuple(dot(reference, z) * value for value in z))
        if norm(projected) > 1e-6:
            x = normalized(projected)
            y = cross(z, x)
            return frame_from_columns(x, y, z)
    raise ValueError("could not derive a frame")  # pragma: no cover


def frame_error(frame: Matrix3, direction: Sequence[float]) -> str | None:
    """Return why ``frame`` is not a valid frame for ``direction``, or None."""

    x, y, z = frame_columns(frame)
    for name, column in (("X", x), ("Y", y), ("Z", z)):
        if abs(norm(column) - 1.0) > 1e-6:
            return f"column {name} is not unit length"
    if abs(dot(x, y)) > 1e-6 or abs(dot(x, z)) > 1e-6 or abs(dot(y, z)) > 1e-6:
        return "columns are not orthogonal"
    if dot(cross(x, y), z) < 0.0:
        return "frame is left-handed (determinant -1)"
    if norm(sub(z, normalized(direction))) > 1e-5:
        return "column Z does not follow start -> end"
    return None


def closest_fraction(point: Sequence[float], start: Sequence[float], end: Sequence[float]) -> tuple[float, float]:
    """Return (distance, fraction along the segment) of the closest point."""

    delta = sub(end, start)
    denominator = dot(delta, delta)
    fraction = 0.0
    if denominator > 0.0:
        fraction = max(0.0, min(1.0, dot(sub(point, start), delta) / denominator))
    closest = tuple(float(start[i]) + fraction * delta[i] for i in range(3))
    return norm(sub(point, closest)), fraction
