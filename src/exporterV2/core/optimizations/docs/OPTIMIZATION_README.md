# Joint-Budget Optimization System - Documentation Index

> **Incremental optimization system to reduce joint count in USD plant models for Isaac Sim/PhysX.**

## 📚 Available Documentation

### 1. [OPTIMIZATION_QUICK_START.md](./OPTIMIZATION_QUICK_START.md)
**Contents**: Quick reference guide to get started, usage examples, configuration, and troubleshooting.

**When to use it**:
- First approach to the system
- Quick reference during usage and development
- Debugging common issues
- API code examples

---

### 2. This File (README)
**Contents**: General overview and documentation index.

---

## 🎯 Quick Navigation

| Goal... | Go to... |
|-----------|----------|
| Get started using the system | [Quick Start](./OPTIMIZATION_QUICK_START.md) |
| V2 implementation notes | [Exporter V2 Implementation Notes](../../../docs/07_implementation_notes.md) |
| Troubleshooting | [Quick Start](./OPTIMIZATION_QUICK_START.md#common-issues--solutions) |
| Configure budget | [Quick Start](./OPTIMIZATION_QUICK_START.md#configuration-quick-ref) |
| Testing Reference | [Quick Start](./OPTIMIZATION_QUICK_START.md#testing-reference) |

---

## 📋 Executive Summary

### Problem
Isaac Sim/PhysX has a hardware-imposed limit of ~250 joints per articulation. Day 100+ tomato plants with multiple branches, trusses, and fruits exceed this limit, causing physics solver instability or engine crashes.

### Solution
An incremental optimization system that applies 5 LOD-based techniques ordered by minimal visual impact, reducing joints until fitting within the budget while maintaining structural integrity.

### Techniques (Priority Order)
1. **Petiole Lock** (Priority 1): D6 → Fixed joint (no geometry change)
2. **Lateral Reduce** (Priority 2): Reduce segments in lateral branches
3. **Stem Collapse** (Priority 3): Collapse trunk segments + remap attachments
4. **Truss Static** (Priority 4): Pre-bent static geometry
5. **Leaf Branch Reduce** (Priority 5): Merge petiole + rachis

### Key Features
- ✅ **Incremental**: Applies techniques progressively, stopping immediately when budget is met
- ✅ **Safe**: Geometric validation + collision checks after each step
- ✅ **Transparent**: Detailed report with breakdown per technique
- ✅ **Configurable**: External YAML for budget, limits, parameters
- ✅ **Extensible**: Plugin architecture for new techniques

### Validation
- **Industry standard**: Two-stage broad-phase collision detection (Sphere + AABB)

---

## 🚀 Getting Started

### For Developers & Researchers

1. **Consult** [Exporter V2 Implementation Notes](../../../docs/07_implementation_notes.md) for current engineering decisions.
2. **Use** [Quick Start](./OPTIMIZATION_QUICK_START.md) for quick references on running tests and configuring the environment.

### For Users

```python
# Basic Example
from exporterV2.core.optimizations import BudgetOptimizer

optimizer = BudgetOptimizer()
optimized_branches, report = optimizer.optimize(branches)
print(report)
```

```bash
# From the launcher: joints.optimize: lock (or full) in config/physics.yaml
./run_main.sh --generate-only
```

See [Quick Start](./OPTIMIZATION_QUICK_START.md) for full examples.

---

## 📊 Project Status

✅ **Project Completed**: All phases (Infrastructure, Techniques, Integration, and Visual Validation) have been successfully implemented, tested, and documented.

---

## 🏗️ Architecture Overview

```
Optimizer (Orchestrator)
    ↓
Apply Techniques by Priority
    ↓
┌─────────────┬──────────────┬────────────┐
│  Technique  │  Collision   │  Geometry  │
│  Plugins    │  Detection   │  Remapping │
└─────────────┴──────────────┴────────────┘
    ↓
Optimized Branches Config
    ↓
build_stage() → USD Export
```

---

## 📝 Notes

### Design Principles
- **Minimal Visual Impact**: Priority to techniques preserving realistic appearance
- **Structural Integrity**: Never drop below structural lower bound
- **Fail Safe**: Clear error message if budget cannot be met
- **Transparent**: Report tracks every step

---

## 🔗 Related Documentation

- **Collision Recommendation**: `collision_check_recommendation.md`
- **Tree Config**: `../core/tree_config.py`
- **USD Stage Builder**: `../core/usd/stage.py`

---

**Version**: 2.0  
**Author**: Alessandro
