# 5. Lateral Branch Reduction

## Motivation and Purpose
Peripheral lateral branches contribute significantly to total joint budget when constructed with multiple articulated links (e.g., 5-7 links per branch) to simulate bending under gravity or wind. When overall joint budget is tight, reducing lateral branch link counts trades off fine compliance while preserving overall plant volume, canopy spread, and branch topology.

## Technical, Geometric, and Physical Aspects
The technique inspects the plant structure to locate lateral branches (matching ID patterns such as `Branch_`).
Once identified, it iteratively decrements `n_links` across lateral branches (typically by 1 link per pass), recalculating link radii and heights to preserve total branch length. Reduction stops when branches reach a configurable minimum limit (`min_segments: 1`). A round-robin reduction strategy ensures balanced joint savings across all lateral branches rather than aggressively stripping a single branch.
Child branches attached to lateral stems remain hierarchically connected without requiring remapping if lateral link reduction preserves total length.

## Testing and Validation
Unit tests confirm that link counts are reduced evenly across branches and adhere strictly to the `min_segments` constraint (preventing branch deletion or disconnected link floating).

## Notes, Limitations, and Assumptions
- **Assumption**: Assumes child sub-branches attach at branch endpoints rather than intermediate joints. If intermediate attachments exist, link reduction preserves connectivity but adjusts joint dynamics.
- Visual impact is moderate: lateral branches appear stiffer and straighter while maintaining correct spatial footprint.
