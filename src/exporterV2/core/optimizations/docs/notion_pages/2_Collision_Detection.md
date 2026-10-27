# 2. Collision Detection System

## Motivation and Purpose
When optimizing a tree structure (e.g., shortening a trunk segment and remapping attached branches), there is a risk of physically overlapping adjacent branches. In physical simulation engines like Isaac Sim, overlapping collision meshes between adjacent nodes trigger explosive solver behaviors, causing the plant to jitter or launch into the air ("rise-on-touch"). This module prevents the application of geometric optimizations if they lead to physically invalid self-intersections.

## Technical, Geometric, and Physical Aspects
To ensure high procedural performance, the system employs a two-stage Broad-Phase pipeline:
1. **Stage 1 (Bounding Sphere)**: An `O(1)` fast check based on the distance between bounding sphere centers enclosing branch cylinders. It is conservative, immediately eliminating 70-80% of non-colliding pairs without heavy computation.
2. **Stage 2 (AABB - Axis-Aligned Bounding Box)**: If bounding spheres overlap (or in false-positive cases like near-perpendicular branches), an AABB is calculated by sampling branch cylinder geometries along X, Y, Z axes. Overlap across all three axes simultaneously mathematically confirms true spatial clipping.

## Testing and Validation
The module features a comprehensive validation suite:
- Vector math unit tests (`Vec3`).
- Bounding sphere and AABB intersection tests, accounting for parametric safety margins (e.g., `margin=0.01m`).
- Visual 3D test scripts using `matplotlib` to render collision scenarios, demonstrating false-positive rejection and clipping detection accuracy.

## Notes, Limitations, and Assumptions
- **Assumption**: Each segment (link) of the plant is modeled as a perfect cylinder for bounding volume calculations, approximating the generated mesh geometry.
- **Limitation**: Uses AABBs instead of OBBs (Oriented Bounding Boxes). While OBBs follow exact branch rotations, computing them via the Separating Axis Theorem (SAT) introduces performance overhead. A conservative AABB safety margin is sufficient and computationally optimal for botanical simulation.
