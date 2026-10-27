"""Shared topology helpers for procedural tube meshes."""

from pxr import Gf


def build_open_tube_topology(ring_count: int, radial_segments: int):
    """Return the triangular side faces for a tube without end caps."""
    face_counts = []
    face_indices = []
    for ring in range(ring_count - 1):
        row0 = ring * radial_segments
        row1 = (ring + 1) * radial_segments
        for radial in range(radial_segments):
            next_radial = (radial + 1) % radial_segments
            face_counts.extend((3, 3))
            face_indices.extend(
                (
                    row0 + radial,
                    row1 + radial,
                    row1 + next_radial,
                    row0 + radial,
                    row1 + next_radial,
                    row0 + next_radial,
                )
            )
    return face_counts, face_indices


def smooth_vertex_normals(points, face_counts, face_indices):
    """Area-weighted per-vertex normals, so tubes shade round, not faceted."""
    sums = [Gf.Vec3d(0.0) for _ in points]
    offset = 0
    for count in face_counts:
        face = face_indices[offset:offset + count]
        offset += count
        origin = Gf.Vec3d(points[face[0]])
        for i in range(1, count - 1):
            # Cross product length is twice the triangle area.
            normal = Gf.Cross(
                Gf.Vec3d(points[face[i]]) - origin,
                Gf.Vec3d(points[face[i + 1]]) - origin,
            )
            for index in (face[0], face[i], face[i + 1]):
                sums[index] += normal
    normals = []
    for total in sums:
        length = total.GetLength()
        normals.append(Gf.Vec3f(*(total / length)) if length > 1e-20 else Gf.Vec3f(0.0, 0.0, 1.0))
    return normals
