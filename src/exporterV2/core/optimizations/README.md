# Joint-Budget Optimization System

> **Incremental optimization system to reduce joint count in USD plant models for Isaac Sim/PhysX.**

## Quick Links

- 📖 **Full Documentation**: See [`docs/OPTIMIZATION_README.md`](docs/OPTIMIZATION_README.md)
- 📋 **Implementation Plan**: [`docs/OPTIMIZATION_IMPLEMENTATION_PLAN.md`](docs/OPTIMIZATION_IMPLEMENTATION_PLAN.md)
- 🏗️ **Architecture & Design**: [`docs/OPTIMIZATION_README.md`](docs/OPTIMIZATION_README.md)
- 🚀 **Quick Start Guide**: [`docs/OPTIMIZATION_QUICK_START.md`](docs/OPTIMIZATION_QUICK_START.md)

---

## Current Status

**Progress**: 🟡 **1/12 tasks complete** (Task 1: Setup Infrastructure ✅)

### Completed
- ✅ Task 1: Setup Infrastructure (2025-01-08)

### Next Up
- 🔴 Task 2: Collision Detection (sphere + AABB)
- 🔴 Task 3: Geometry Remapping

See [`TASK1_SUMMARY.md`](TASK1_SUMMARY.md) for Task 1 completion details.

---

## Usage (After Full Implementation)

```python
from exporterV2.core.optimizations import BudgetOptimizer

# Initialize optimizer
optimizer = BudgetOptimizer()

# Optimize branches configuration
optimized_branches, report = optimizer.optimize(branches)

# Print report
print(report)
# Output:
# ========================================
# Joint-Budget Optimization Report
# ========================================
# Original joints: 320
# Budget: 250
# Techniques applied:
#   1. Petiole Lock: 320 → 290 (-30)
#   2. Lateral Reduce: 290 → 250 (-40)
# Final joints: 250 ✓
# ========================================
```

---

## Running Tests

```bash
# From the repository root
uv run pytest -q src/exporterV2/core/optimizations/tests
```

---

## Configuration

Edit [`budget_config.yaml`](budget_config.yaml) to adjust:
- Budget limits (`max_joints`)
- Structural minimums (per component type)
- Technique priorities and parameters

Example:
```yaml
budget:
  max_joints: 250  # ← Adjust for your hardware

techniques:
  - id: "petiole_lock"
    priority: 1
    enabled: true  # ← Set to false to disable
```

---

## Directory Structure

```
optimizations/
├── docs/              # Comprehensive documentation
├── techniques/        # Optimization technique plugins
├── collision/         # Collision detection utilities
├── geometry/          # Geometry remapping utilities
├── tests/             # Test suite
├── optimizer.py       # Main orchestrator
└── budget_config.yaml # Configuration
```

---

## Key Features

- ✅ **Incremental**: Applies techniques progressively until budget met
- ✅ **Safe**: Validates geometry + collisions after each step
- ✅ **Transparent**: Detailed report with breakdown per technique
- ✅ **Configurable**: External YAML for all parameters
- ✅ **Extensible**: Plugin architecture for new techniques

---

## Optimization Techniques (Planned)

1. **Petiole Lock** (Priority 1): D6 → Fixed joint
2. **Lateral Reduce** (Priority 2): Reduce lateral branch segments
3. **Stem Collapse** (Priority 3): Collapse trunk + remap attachments
4. **Truss Static** (Priority 4): Pre-bent static geometry
5. **Leaf Branch Reduce** (Priority 5): Merge petiole + rachis

---

## For Developers

### Adding a New Technique

1. Create file in `techniques/my_technique.py`
2. Extend `OptimizationTechnique` base class
3. Add to `budget_config.yaml`
4. Write tests in `tests/test_my_technique.py`

See [`docs/OPTIMIZATION_README.md`](docs/OPTIMIZATION_README.md) for details.

### Running Full Test Suite (After Implementation)

```bash
uv run pytest src/exporterV2/core/optimizations/tests/ -v
```

---

## Support

- 📖 Read [`docs/OPTIMIZATION_README.md`](docs/OPTIMIZATION_README.md) for comprehensive guide
- 🐛 Check [`docs/OPTIMIZATION_QUICK_START.md`](docs/OPTIMIZATION_QUICK_START.md#common-issues--solutions) for troubleshooting
- 📋 Track progress in [`docs/OPTIMIZATION_IMPLEMENTATION_PLAN.md`](docs/OPTIMIZATION_IMPLEMENTATION_PLAN.md)

---

**Last Updated**: 2025-01-08  
**Status**: Task 1 Complete, Ready for Task 2
