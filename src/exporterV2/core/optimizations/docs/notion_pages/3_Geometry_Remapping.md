# 3. Geometry Remapping (attach_frac)

## Motivation and Purpose
When compressing stem geometry (e.g., collapsing a 10-link trunk down to 3 links to save joint budget), child branches attached to intermediate links (e.g., original link 9 of 10) would be geometrically misplaced or forced onto the top of the last link, drastically distorting the plant's shape. This module solves attachment remapping mathematically, maintaining exact proportional height and world-space positions.

## Technical, Geometric, and Physical Aspects
The legacy approach (based on absolute height sums across 0-based arrays) was replaced with a highly stable 1-based fractional model:
- Each child branch tracks its attachment using an `attach_link` index (1-based) and a sub-link fractional offset `attach_frac` (`0.0` to `1.0`).
- When a parent branch collapses, `remap_link_attachment(attach_link, n_old, n_new)` computes proportional scaling (e.g., a branch at 90% total height is remapped to `k = floor(V) + 1` and `p = V - floor(V)`).
This preserves topological and spatial consistency: although the trunk becomes stiffer due to fewer joint links, child branches originate at the exact same physical height above ground.

## Testing and Validation
The module has been verified analytically and visually:
- Edge cases are unit-tested, guaranteeing that branches attached to the absolute top of the original stem don't overflow link array boundaries, anchoring cleanly at `attach_frac = 1.0` on the top link.
- 3D visualizers and Isaac Sim test scripts validate remapping across progressive collapse steps (5 -> 4 -> 3 -> 2 -> 1 links).

## Notes, Limitations, and Assumptions
- **Assumption**: Assumes segment height reduction is applied uniformly across parent links during collapse.
- Remapping is the core geometric dependency enabling all structural reduction techniques (e.g., Leaf Branch Reduction, Stem Collapse), allowing cascaded automatic updates across the tree hierarchy.
