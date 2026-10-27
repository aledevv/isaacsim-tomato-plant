# 4. Petiole Lock Technique

## Motivation and Purpose
Petioles (the small stems attaching leaves to branches) significantly inflate joint counts. In many plant-robot interaction scenarios, individual petiole compliance has minimal impact on visual realism or macroscopic physical response, yet consumes significant solver computation per frame due to articulated D6 joints. The *Petiole Lock* technique addresses this by converting petiole joints to fixed constraints, saving physics overhead with minimal visual tradeoff.

## Technical, Geometric, and Physical Aspects
In Isaac Sim USD representations, D6 joints define angular limits and stiffness/damping drives.
This technique operates without changing spatial geometry (remapping is not required). Instead, it flags targeted petiole branches with a `lock: true` property. During USD generation, the exporter converts these joint definitions to `Fixed` joints, effectively merging petiole inertia into the parent branch for PhysX, significantly reducing solver complexity.

## Testing and Validation
Unit tests verify string-pattern filtering:
- The algorithm identifies petiole nodes (e.g., `Leaf_rX_oY_petiole`) without affecting other branch types.
- Tests validate batch updating of targeted petiole nodes to the locked state and quantify joint savings.

## Notes, Limitations, and Assumptions
- **Assumption**: Assigned high priority in optimization configuration. Applied first because the solver performance boost far outweighs the minor visual loss in leaf-level swaying.
- The canopy becomes slightly stiffer under contact; leaves will not flex individually around their base, but still inherit the dynamic movement of their parent branches.
