"""Joint-budget optimization for skeleton branches.

skeleton links carry per-organ ``link_specs`` (geometry, mass, canonical
ids), so techniques never move or resize a link. Two technique sets exist:

* ``lock`` -- joint locking only (``thin_link_lock``, ``petiole_lock``,
  ``pedicel_lock``): D6 joints become fixed, every body is kept;
* ``full`` -- every technique enabled in ``budget_config.yaml``, in its
  priority order, with skeleton link merging for ``lateral_reduce``,
  ``stem_collapse``, ``truss_static`` and ``leaf_branch_reduce``
  (see ``joint_budget_merges``). Merged links are absorbed into
  their upstream body after the stage is built
  (``core.usd.body_merge.merge_rigid_links``).
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any, Dict, List

import yaml

from .core.optimizations.optimizer import BudgetConfig, BudgetOptimizer
from .skeleton_branches import BranchPlan, SkeletonBranchesError
from .joint_budget_merges import (
    MAX_ARTICULATION_LINKS,
    MERGE_TECHNIQUES,
    FixedLinkMergeTechnique,
    merged_links,
    count_bodies,
    count_d6,
)


PLANT_STATE_LOCK_TECHNIQUES = ("thin_link_lock", "petiole_lock", "pedicel_lock")
PLANT_STATE_TECHNIQUE_SETS = ("lock", "full")
_DEFAULT_CONFIG = (
    Path(__file__).resolve().parent / "core" / "optimizations" / "budget_config.yaml"
)
_SUPPORTED = set(PLANT_STATE_LOCK_TECHNIQUES) | set(MERGE_TECHNIQUES)


def _technique_ids(techniques: str | tuple[str, ...], mapping: dict) -> tuple[str, ...]:
    if techniques == "lock":
        return PLANT_STATE_LOCK_TECHNIQUES
    if techniques == "full":
        return tuple(
            entry["id"] for entry in mapping["techniques"] if entry.get("enabled", True)
        )
    if isinstance(techniques, str):
        raise SkeletonBranchesError(
            f"technique set must be one of {PLANT_STATE_TECHNIQUE_SETS}, got {techniques!r}"
        )
    return tuple(techniques)


def budget_config(
    joint_budget: int,
    techniques: str | tuple[str, ...] = "lock",
    config_path: str | Path = _DEFAULT_CONFIG,
) -> BudgetConfig:
    """Return the YAML configuration restricted to the selected techniques."""

    mapping = yaml.safe_load(Path(config_path).read_text())
    ids = _technique_ids(techniques, mapping)
    unsupported = set(ids) - _SUPPORTED
    if unsupported:
        raise SkeletonBranchesError(
            f"Joint-budget optimization does not implement {sorted(unsupported)}"
        )
    by_id = {entry["id"]: entry for entry in mapping["techniques"]}
    selected = []
    for technique_id in ids:
        if technique_id == "pedicel_lock":
            # Same priority and parameters as the truss_static stage it reuses.
            source = by_id.get("truss_static", {"priority": 4, "params": {}})
            entry = {**source, "id": "pedicel_lock"}
        else:
            entry = dict(by_id.get(technique_id, {"id": technique_id, "priority": 1}))
        entry["enabled"] = True
        selected.append(entry)
    return BudgetConfig.from_mapping(
        {**mapping, "techniques": selected}, max_joints=joint_budget
    )


class SkeletonBudgetOptimizer(BudgetOptimizer):
    """Budget optimizer that counts merged links and uses link merges."""

    def calculate_total_joints(self, branches: List[Dict]) -> int:
        return count_d6(branches)

    def calculate_total_rigid_bodies(self, branches: List[Dict], terminal_body_count: int = 0) -> int:
        return count_bodies(branches) + int(terminal_body_count)

    def calculate_lower_bound(self, branches: List[Dict]) -> int:
        """D6 joints left if every technique ran to completion."""

        limits = self.config.structural_limits
        bound = 0
        for branch in branches:
            if branch.get("joint_type", "d6").lower() == "fixed":
                continue
            kind = branch.get("kind")
            if kind == "lateral_branch":
                bound += min(limits["lateral_branch"]["min_links"], branch["n_links"])
            elif kind == "leaf_petiole":
                bound += limits["petiole"]["min_links"]
            elif kind == "truss_rachis":
                bound += limits["truss"]["min_links"]
            elif branch.get("parent") is None:
                bound += limits["trunk"]["min_links"]
        return bound

    def _get_technique(self, technique_config: Dict):
        factory = MERGE_TECHNIQUES.get(technique_config["id"])
        if factory is not None:
            return factory(technique_config.get("params", {}))
        return super()._get_technique(technique_config)


def optimize_branches(
    result: BranchPlan,
    joint_budget: int,
    *,
    techniques: str | tuple[str, ...] = "lock",
) -> tuple[BranchPlan, dict[str, Any]]:
    """Lock and/or merge links until the D6 count fits ``joint_budget``.

    Returns the new result and a JSON-serializable report. Raises
    ``SkeletonBranchesError`` when the selected techniques cannot meet the
    budget.
    """

    if joint_budget < 1:
        raise SkeletonBranchesError("joint_budget must be positive")
    config = budget_config(joint_budget, techniques)
    enabled = [entry["id"] for entry in config.techniques]
    optimizer = SkeletonBudgetOptimizer(config=config)
    terminal_bodies = sum(body.get("physical", True) for body in result.terminal_bodies)
    optimized, report = optimizer.optimize(
        [dict(branch) for branch in result.branches],
        terminal_body_count=terminal_bodies,
    )
    if not report.success:
        raise SkeletonBranchesError(
            f"joint budget {joint_budget} is not reachable with "
            f"{', '.join(enabled)}: {report.error_message}"
        )

    original = {branch["id"]: branch for branch in result.branches}
    locked = []
    for branch in optimized:
        before = original[branch["id"]]
        if branch.get("joint_type") == "fixed" and before.get("joint_type") != "fixed":
            # The USD builder reads the attachment joint separately; without
            # this the first link would keep a D6 that the budget no longer
            # counts (same convention as lateral_joint_policy="fixed").
            branch["attachment_joint_type"] = "fixed"
            locked.append(branch["id"])

    # The budget counts D6 joints, but PhysX also caps links per articulation.
    # With merging enabled, absorb fixed-joint links (no fidelity cost) until
    # the plant fits; joint locking alone cannot remove bodies.
    link_limit_merges = 0
    merging = any(technique in MERGE_TECHNIQUES for technique in enabled)
    if merging:
        fixed_merge = FixedLinkMergeTechnique()
        while (
            count_bodies(optimized) > MAX_ARTICULATION_LINKS
            and fixed_merge.can_apply(optimized)
        ):
            optimized, _ = fixed_merge.apply(optimized)
            link_limit_merges += 1
    final_bodies = count_bodies(optimized)
    if final_bodies > MAX_ARTICULATION_LINKS:
        hint = (
            "" if merging else "; set joints.optimize: full in config/physics.yaml to merge fixed links"
        )
        raise SkeletonBranchesError(
            f"optimized plant has {final_bodies} articulation links, above the "
            f"PhysX limit of {MAX_ARTICULATION_LINKS}{hint}"
        )


    summary: dict[str, dict[str, Any]] = {}
    for item in report.technique_reports:
        entry = summary.setdefault(
            item.technique_name,
            {"joints_before": item.joints_before, "joints_saved": 0, "passes": 0},
        )
        entry["joints_after"] = item.joints_after
        entry["joints_saved"] += item.joints_saved
        entry["passes"] += 1
    return replace(result, branches=tuple(optimized)), {
        "budget": joint_budget,
        "technique_set": techniques if isinstance(techniques, str) else "custom",
        "techniques_enabled": enabled,
        "original_d6": report.original_joints,
        "final_d6": report.final_joints,
        "original_rigid_bodies": report.original_rigid_bodies,
        "articulation_links": final_bodies,
        "rigid_bodies": final_bodies + terminal_bodies,
        "link_limit_merges": link_limit_merges,
        "lower_bound": report.lower_bound,
        "minimum_achievable": report.minimum_achievable,
        "techniques": summary,
        "locked_branch_ids": sorted(locked),
        "merged_links": {
            branch["id"]: sorted(merged_links(branch))
            for branch in optimized
            if merged_links(branch)
        },
    }
