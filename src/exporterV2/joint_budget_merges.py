"""Link-merging optimization techniques for skeleton branches.

skeleton links carry exact canonical geometry, so these techniques never
change ``n_links``, lengths or poses. They mark links in ``merged_links``
(1-based link indices): a marked link is later absorbed into the rigid body
upstream of it by ``core.usd.body_merge.merge_rigid_links`` and its joint is
removed. Link ``1`` is absorbed into the parent branch's attach link.

They mirror the legacy techniques of the same id:

* ``lateral_reduce``     -- thinnest lateral first, one link per pass;
* ``stem_collapse``      -- stem down to ``target_segments`` bodies;
* ``truss_static``       -- pedicels locked per truss, then the rachis becomes
                            one body with its root D6;
* ``leaf_branch_reduce`` -- petiole and rachis become one body.

Lateral and stem merges keep the bodies balanced: they join the adjacent
pair of body groups with the fewest links, so compliance stays distributed.
"""

from __future__ import annotations

from copy import deepcopy
from typing import Dict, List, Tuple

from .core.optimizations.techniques.base import (
    OptimizationReport,
    OptimizationTechnique,
    ValidationResult,
)
from .core.optimizations.techniques.truss_static import PedicelLockTechnique


# Measured on Isaac Sim 4.5 / PhysX 5 (2026-10-06): an articulation with 255
# links opens and simulates; 256 links fails with CUDA error 700 on the GPU
# pipeline and a segmentation fault on CPU dynamics.
MAX_ARTICULATION_LINKS = 255


def merged_links(branch: dict) -> set[int]:
    return {int(index) for index in branch.get("merged_links", ())}


def branch_bodies(branch: dict) -> int:
    return int(branch["n_links"]) - len(merged_links(branch))


def count_d6(branches: List[Dict]) -> int:
    """D6 joints after merges: one per surviving link of a non-fixed branch."""

    return sum(
        branch_bodies(branch)
        for branch in branches
        if branch.get("joint_type", "d6").lower() != "fixed"
    )


def count_bodies(branches: List[Dict]) -> int:
    return sum(branch_bodies(branch) for branch in branches)


def _groups(branch: dict) -> list[list[int]]:
    merged = merged_links(branch)
    groups: list[list[int]] = []
    for index in range(1, int(branch["n_links"]) + 1):
        if index in merged and groups:
            groups[-1].append(index)
        else:
            groups.append([index])
    return groups


def _balanced_merge_index(branch: dict) -> int:
    """First link of the group that joins its predecessor with fewest links."""

    groups = _groups(branch)
    best = min(
        range(1, len(groups)),
        key=lambda j: (len(groups[j - 1]) + len(groups[j]), -j),
    )
    return groups[best][0]


def _with_merged(branch: dict, indices) -> dict:
    copy = deepcopy(branch)
    copy["merged_links"] = sorted(merged_links(branch) | {int(i) for i in indices})
    return copy


def _report(name: str, before: List[Dict], after: List[Dict], **details) -> OptimizationReport:
    joints_before, joints_after = count_d6(before), count_d6(after)
    return OptimizationReport(
        technique_name=name,
        joints_before=joints_before,
        joints_after=joints_after,
        joints_saved=joints_before - joints_after,
        details={
            "bodies_before": count_bodies(before),
            "bodies_after": count_bodies(after),
            **details,
        },
    )


def _replace(branches: List[Dict], updated: dict) -> List[Dict]:
    return [updated if branch["id"] == updated["id"] else branch for branch in branches]


def validate_merges(original: List[Dict], modified: List[Dict]) -> ValidationResult:
    """Merges may only add ``merged_links``; topology and geometry are frozen."""

    errors = []
    before = {branch["id"]: branch for branch in original}
    after = {branch["id"]: branch for branch in modified}
    if before.keys() != after.keys():
        errors.append("branch set changed")
    for branch_id, branch in after.items():
        old = before.get(branch_id)
        if old is None:
            continue
        for key in ("parent", "attach_link", "n_links", "height", "radius", "link_specs"):
            if old.get(key) != branch.get(key):
                errors.append(f"{branch_id}: {key} changed")
        indices = merged_links(branch)
        if not merged_links(old) <= indices:
            errors.append(f"{branch_id}: merges were undone")
        if any(not 1 <= index <= int(branch["n_links"]) for index in indices):
            errors.append(f"{branch_id}: merged link outside 1..n_links")
        if branch.get("parent") is None and 1 in indices:
            errors.append(f"{branch_id}: the world-anchored root link cannot merge")
    if count_d6(modified) > count_d6(original):
        errors.append("D6 count increased")
    return ValidationResult(not errors, errors, [])


class _MergeTechnique(OptimizationTechnique):
    technique_id = ""
    technique_priority = 99

    @property
    def name(self) -> str:
        return self.technique_id

    @property
    def priority(self) -> int:
        return self.technique_priority

    def validate(self, original: List[Dict], modified: List[Dict]) -> ValidationResult:
        return validate_merges(original, modified)


class LateralLinkMergeTechnique(_MergeTechnique):
    technique_id = "lateral_reduce"
    technique_priority = 2

    def _candidates(self, branches: List[Dict]) -> List[Dict]:
        minimum = int(self.params.get("min_segments", 1))
        return sorted(
            (
                b for b in branches
                if b.get("kind") == "lateral_branch" and branch_bodies(b) > minimum
            ),
            key=lambda b: (float(b["radius"]), int(b.get("attach_link", 0)), b["id"]),
        )

    def can_apply(self, branches: List[Dict]) -> bool:
        return bool(self._candidates(branches))

    def estimate_reduction(self, branches: List[Dict]) -> int:
        minimum = int(self.params.get("min_segments", 1))
        return sum(
            branch_bodies(b) - minimum
            for b in self._candidates(branches)
            if b.get("joint_type", "d6") != "fixed"
        )

    def apply(self, branches: List[Dict]) -> Tuple[List[Dict], OptimizationReport]:
        target = self._candidates(branches)[0]
        index = _balanced_merge_index(target)
        modified = _replace(branches, _with_merged(target, [index]))
        return modified, _report(self.name, branches, modified, branch_id=target["id"], link=index)


class StemLinkMergeTechnique(_MergeTechnique):
    technique_id = "stem_collapse"
    technique_priority = 3

    def _stems(self, branches: List[Dict]) -> List[Dict]:
        target = int(self.params.get("target_segments", 3))
        return [
            b for b in branches
            if b.get("parent") is None and branch_bodies(b) > max(target, 1)
        ]

    def can_apply(self, branches: List[Dict]) -> bool:
        return bool(self._stems(branches))

    def estimate_reduction(self, branches: List[Dict]) -> int:
        target = max(int(self.params.get("target_segments", 3)), 1)
        return sum(
            branch_bodies(b) - target
            for b in self._stems(branches)
            if b.get("joint_type", "d6") != "fixed"
        )

    def apply(self, branches: List[Dict]) -> Tuple[List[Dict], OptimizationReport]:
        stem = self._stems(branches)[0]
        index = _balanced_merge_index(stem)
        modified = _replace(branches, _with_merged(stem, [index]))
        return modified, _report(self.name, branches, modified, branch_id=stem["id"], link=index)


class TrussRigidRachisTechnique(PedicelLockTechnique):
    """Skeleton ``truss_static``: lock one truss's pedicels per pass, then
    turn one rachis per pass into a single body with its root D6."""

    @property
    def name(self) -> str:
        return "truss_static"

    def _rachis_candidates(self, branches: List[Dict]) -> List[Dict]:
        return sorted(
            (
                b for b in branches
                if b.get("kind") == "truss_rachis" and branch_bodies(b) > 1
            ),
            key=lambda b: (-branch_bodies(b), b["id"]),
        )

    def can_apply(self, branches: List[Dict]) -> bool:
        return bool(self._dynamic_pedicel_groups(branches) or self._rachis_candidates(branches))

    def estimate_reduction(self, branches: List[Dict]) -> int:
        return super().estimate_reduction(branches) + sum(
            branch_bodies(b) - 1
            for b in self._rachis_candidates(branches)
            if b.get("joint_type", "d6") != "fixed"
        )

    def apply(self, branches: List[Dict]) -> Tuple[List[Dict], OptimizationReport]:
        if self._dynamic_pedicel_groups(branches):
            was_fixed = {b["id"] for b in branches if b.get("joint_type") == "fixed"}
            modified, _ = super().apply(branches)
            for branch in modified:
                if branch.get("joint_type") == "fixed" and branch["id"] not in was_fixed:
                    branch["attachment_joint_type"] = "fixed"
            return modified, _report(self.name, branches, modified, stage="pedicels_fixed")
        rachis = self._rachis_candidates(branches)[0]
        indices = range(2, int(rachis["n_links"]) + 1)
        modified = _replace(branches, _with_merged(rachis, indices))
        return modified, _report(
            self.name, branches, modified, stage="rigid_rachis", branch_id=rachis["id"]
        )

    def validate(self, original: List[Dict], modified: List[Dict]) -> ValidationResult:
        return validate_merges(original, modified)


class LeafBranchMergeTechnique(_MergeTechnique):
    technique_id = "leaf_branch_reduce"
    technique_priority = 5

    def _pairs(self, branches: List[Dict]) -> list[tuple[dict, dict]]:
        by_id = {b["id"]: b for b in branches}
        pairs = []
        for rachis in branches:
            petiole = by_id.get(rachis.get("parent"))
            if (
                rachis.get("kind") == "leaf_rachis"
                and petiole is not None
                and petiole.get("kind") == "leaf_petiole"
                and (branch_bodies(rachis) > 0 or branch_bodies(petiole) > 1)
            ):
                pairs.append((petiole, rachis))
        return pairs

    def can_apply(self, branches: List[Dict]) -> bool:
        return bool(self._pairs(branches))

    def estimate_reduction(self, branches: List[Dict]) -> int:
        saved = 0
        for petiole, rachis in self._pairs(branches):
            if rachis.get("joint_type", "d6") != "fixed":
                saved += branch_bodies(rachis)
            if petiole.get("joint_type", "d6") != "fixed":
                saved += branch_bodies(petiole) - 1
        return saved

    def apply(self, branches: List[Dict]) -> Tuple[List[Dict], OptimizationReport]:
        petiole, rachis = self._pairs(branches)[0]
        merged_rachis = _with_merged(rachis, range(1, int(rachis["n_links"]) + 1))
        merged_petiole = _with_merged(petiole, range(2, int(petiole["n_links"]) + 1))
        modified = _replace(_replace(branches, merged_rachis), merged_petiole)
        return modified, _report(
            self.name, branches, modified, petiole_id=petiole["id"], rachis_id=rachis["id"]
        )


class FixedLinkMergeTechnique(_MergeTechnique):
    """Absorb links whose joints are all fixed: no fidelity cost, fewer bodies.

    A fixed joint inside a reduced-coordinate articulation is already rigid,
    so merging such a link only removes a body. Petiolules go first, then
    pedicels, then any other locked branch; one branch per pass. Link 1 is
    merged into the parent only when the attachment joint is fixed too.
    """

    technique_id = "fixed_link_merge"
    technique_priority = 6
    _ORDER = {"petiolule": 0, "pedicel": 1}

    def _mergeable(self, branch: dict) -> list[int]:
        if branch.get("parent") is None or branch.get("joint_type") != "fixed":
            return []
        first = 1 if branch.get("attachment_joint_type", branch["joint_type"]) == "fixed" else 2
        return [
            index
            for index in range(first, int(branch["n_links"]) + 1)
            if index not in merged_links(branch)
        ]

    def _candidates(self, branches: List[Dict]) -> List[Dict]:
        return sorted(
            (b for b in branches if self._mergeable(b)),
            key=lambda b: (self._ORDER.get(b.get("kind"), 2), b["id"]),
        )

    def can_apply(self, branches: List[Dict]) -> bool:
        return bool(self._candidates(branches))

    def estimate_reduction(self, branches: List[Dict]) -> int:
        return 0  # fixed links carry no D6 joints

    def apply(self, branches: List[Dict]) -> Tuple[List[Dict], OptimizationReport]:
        target = self._candidates(branches)[0]
        modified = _replace(branches, _with_merged(target, self._mergeable(target)))
        return modified, _report(self.name, branches, modified, branch_id=target["id"])


MERGE_TECHNIQUES = {
    "lateral_reduce": LateralLinkMergeTechnique,
    "stem_collapse": StemLinkMergeTechnique,
    "truss_static": TrussRigidRachisTechnique,
    "leaf_branch_reduce": LeafBranchMergeTechnique,
    "fixed_link_merge": FixedLinkMergeTechnique,
}
