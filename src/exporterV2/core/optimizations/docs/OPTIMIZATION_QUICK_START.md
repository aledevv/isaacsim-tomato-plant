# Joint-Budget Optimization - Quick Start Guide

> **Quick reference** for using and extending the optimization system.

## Getting Started

### 1. Key Documents

1. **[7_Comprehensive_Optimization_Report.md](./notion_pages/7_Comprehensive_Optimization_Report.md)** - Exhaustive technical report
2. **[OPTIMIZATION_README.md](./OPTIMIZATION_README.md)** - Overview and documentation index
3. **This file** - Quick reference for usage, configuration, and troubleshooting

---

## System Usage

### Basic Python API

```python
from exporterV2.core.optimizations import BudgetOptimizer

# Load optimizer with default config (budget_config.yaml)
optimizer = BudgetOptimizer()

# Optimize branches configuration
optimized_branches, report = optimizer.optimize(branches)

# Print detailed report
print(report)
# Output:
# ========================================
# Joint-Budget Optimization Report
# ========================================
# Original joints: 320
# Budget: 250
# Lower bound: 45
# 
# Techniques applied:
#   1. Petiole Lock: 320 → 290 (-30 joints)
#   2. Lateral Reduce: 290 → 260 (-30 joints)
#   3. Stem Collapse: 260 → 250 (-10 joints)
# 
# Final joints: 250 ✓
# Total reduction: -70 joints (21.9%)
# ========================================
```

### Integration in Parse Pipeline

```python
# In main.py
branches, json_path = parse_csv_to_branches(day=50, optimize=True)
```

### Command-Line Interface (CLI)

```bash
# Set joints.optimize: lock (or full) in config/physics.yaml, then
./run_main.sh --generate-only

# Output includes optimization report prior to USD generation
```

---

## File Structure Reference

```
exporterV2/core/optimizations/
├── __init__.py                 # Exports: BudgetOptimizer
├── optimizer.py                # Main orchestrator class
├── budget_config.yaml          # Configuration file
│
├── techniques/
│   ├── base.py                 # Abstract base: OptimizationTechnique
│   ├── petiole_lock.py         # Priority 1: D6 -> Fixed
│   ├── lateral_reduce.py       # Priority 2: Reduce lateral segments
│   ├── stem_collapse.py        # Priority 3: Collapse trunk + remap
│   ├── truss_static.py         # Priority 4: Pre-bent static geometry
│   └── leaf_branch_reduce.py   # Priority 5: Merge petiole + rachis
│
├── collision/
│   ├── sphere.py               # Stage 1: Fast sphere overlap check
│   ├── aabb.py                 # Stage 2: Precision AABB check
│   └── broad_phase.py          # Collision orchestration
│
├── geometry/
│   ├── remapping.py            # Attachment height remapping
│   └── bounds.py               # Bounding volume helpers
│
└── docs/
    ├── OPTIMIZATION_README.md  # Main documentation index
    ├── OPTIMIZATION_QUICK_START.md # Quick reference guide
    ├── RESEARCH_VALIDATION.md  # Academic & research background
    └── notion_pages/           # Detailed pages (1-7)
```

---

## Configuration Quick Ref

### Key Sections in `budget_config.yaml`

```yaml
budget:
  max_joints: 250              # ← Edit for your hardware target

structural_limits:
  trunk: { min_links: 1 }
  lateral_branch: { min_links: 1 }
  petiole: { min_links: 1 }

techniques:
  - id: "petiole_lock"
    priority: 1                # ← Lower number = applied first
    enabled: true              # ← Set false to disable technique
    params:
      convert_all_petiolules: true
```

### Enabling / Disabling Techniques

```yaml
techniques:
  - id: "stem_collapse"
    enabled: false             # Disable stem collapse technique
```

### Adjusting Collision Safety Margin

```yaml
techniques:
  - id: "stem_collapse"
    params:
      collision_check:
        safety_margin: 0.02    # Increase for more conservative checks (meters)
```

---

## Testing Reference

### Run All Integration Tests

```bash
cd src/exporterV2
pytest core/optimizations/tests/ -v
```

### Run Specific Technique Test

```bash
pytest core/optimizations/tests/test_stem_collapse.py -v
```

---

## Common Issues & Solutions

### Issue: "Budget impossible to meet (lower bound > budget)"

**Cause**: The plant topology requires more joints than the budget even in its minimal structural state.

**Solutions**:
1. Increase `max_joints` in `budget_config.yaml`.
2. Reduce initial plant growth day or complexity (fewer leaves/lateral branches).
3. Use a simplified plant profile.

### Issue: "Collision detected after remapping"

**Cause**: Stem collapse created overlapping geometries among sibling branches.

**Solutions**:
1. Increase `safety_margin` in the collision configuration.
2. Temporarily disable `stem_collapse`.
3. Use 3D visual validation tools to inspect the spatial overlaps.

---

## Key Algorithms Quick Ref

### Lower Bound Calculation

```python
lower_bound = (
    n_trunk * 1 +           # Min 1 link per trunk
    n_lateral * 1 +         # Min 1 link per lateral branch
    n_petiole * 1 +         # Min 1 link per petiole
    n_truss * 1             # Min 1 link per truss
    # petiolules and rachis can be 0
)
```

### Attachment Remapping

```python
absolute_height_before = sum(heights[0:original_link_idx])
new_link_idx = int(absolute_height_before / new_segment_height)
offset_z = absolute_height_before - (new_link_idx * new_segment_height)
```

### Two-Stage Collision Check

```python
# Stage 1 (Fast Sphere Check):
distance = (center1 - center2).length()
sphere_overlap = distance < (radius1 + radius2 + margin)

# Stage 2 (Precision AABB Check):
# Evaluated across X, Y, and Z axes
aabb_overlap = (min1.x <= max2.x and max1.x >= min2.x) and \
               (min1.y <= max2.y and max1.y >= min2.y) and \
               (min1.z <= max2.z and max1.z >= min2.z)
```

---

**Last Updated**: 2026-08-07  
**Status**: Production Ready
