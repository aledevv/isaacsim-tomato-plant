# 1. Budget Optimizer & Infrastructure

## Motivation and Purpose
The botanical Digital Twin in Isaac Sim must represent plants at various growth stages, but uncontrolled addition of articulations (D6 joints) drastically drops physics engine performance. The goal of the Budget Optimizer is to orchestrate a series of structural reduction techniques, dynamically altering plant topology until the total number of joints fits within a predefined limit (`max_joints`), all while preserving the visual appearance and overall spatial envelope of the tree.

## Technical, Geometric, and Physical Aspects
The architecture relies on a plugin design pattern (via the abstract base class `OptimizationTechnique`).
The optimizer loads limits from `budget_config.yaml`, computes a structural "lower bound" (the minimum un-eliminable joints needed to prevent plant destruction), and then sequentially applies active techniques ordered by priority.
The optimization loop terminates as soon as the calculated joint count drops below the budget threshold, returning the updated USD dictionary of the plant along with a detailed optimization report.

## Testing and Validation
Tests ensure that the infrastructure operates correctly under edge-case scenarios:
- Correct loading of hierarchies and priorities from the YAML config file.
- Exact calculation of lower bounds for complex branching structures (trunks, laterals, petioles, trusses).
- Detection of impossible budgets (e.g., requesting a budget lower than the structural lower bound).
All of these aspects are verified via isolated unit tests that guarantee orchestrator stability.

## Notes, Limitations, and Assumptions
- **Assumption**: Assumes the designer has coherently configured priority ordering in the YAML file; if aggressive techniques have high priority, visual detail will be sacrificed earlier than necessary.
- **Limitation**: The optimizer uses a "greedy" approach (applies techniques sequentially and stops as soon as it meets the budget), thus not guaranteeing a global visual optimum, but providing near-instantaneous execution suitable for procedural generation.
