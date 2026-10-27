# 6. Leaf Branch Reduction

## Motivation and Purpose
In highly detailed procedural plant models, each compound leaf consists of a base petiole, a multi-segment articulated rachis, and multiple attached leaflets (petiolules). This structural detail introduces a high density of physical joints per leaf. The *Leaf Branch Reduction* technique merges the petiole and rachis into a single rigid segment while remapping child petiolules, achieving a "static leaf, dynamic branch" compromise.

## Technical, Geometric, and Physical Aspects
This advanced technique performs the following steps:
- Identifies petiole, rachis, and petiolule branch groups.
- Merges the petiole and rachis: sums their lengths and averages their radii into a single merged segment (ID ending in `_merged`).
- Applies fractional remapping (`attach_frac`): recalculates relative attachment positions of all child petiolules along the new merged leaf segment (`0.0 - 1.0` range).

## Testing and Validation
Combines graph topology modification and spatial remapping:
- Unit tests confirm valid parent re-anchoring, exact fractional offset calculations, and joint savings per leaf.
- A USD comparison generator (`generate_comparison_usd.py`) outputs three stages for visual inspection in **Isaac Sim**:
    1. Baseline (Fully articulated leaf, 13 links)
    2. Partial (Reduced rachis links, 11 links)
    3. Merged (Rigid leaf with exact petiolule remapping, 10 links)
Visual inspection confirms that leaflet positions in 3D world space match baseline alignment.

## Notes, Limitations, and Assumptions
- **Assumption**: Converts compound leaf structures from compliant flexible elements to rigid single-link structures.
- **Limitation**: Highly effective for joint reduction, but assigned lower priority (Priority 5) to preserve leaf articulation until significant budget reduction is required.
