"""Progressively simplify tomato trusses while preserving their silhouette."""

from __future__ import annotations

from copy import deepcopy
import math
from typing import Dict, List, Tuple

try:
    from .base import OptimizationTechnique, OptimizationReport, ValidationResult, count_d6_joints
except ImportError:
    from base import OptimizationTechnique, OptimizationReport, ValidationResult, count_d6_joints


class TrussStaticTechnique(OptimizationTechnique):
    """Lock pedicels first, then turn a rachis into a pre-bent rigid block."""

    def __init__(self, params: Dict | None = None):
        params = params or {}
        self.curve_segments = int(params.get("curve_segments", 5))
        self.bend_per_segment_deg = float(params.get("bend_per_segment_deg", 8.0))
        self.pedicel_droop_deg = float(params.get("pedicel_droop_deg", 10.0))
        self.root_bend_limit_deg = float(params.get("root_bend_limit_deg", 18.0))
        self.root_drive_stiffness_scale = float(
            params.get("root_drive_stiffness_scale", 0.40)
        )
        if self.curve_segments < 2:
            raise ValueError("curve_segments must be at least 2")
        if self.root_drive_stiffness_scale <= 0.0:
            raise ValueError("root_drive_stiffness_scale must be positive")

    @property
    def name(self) -> str:
        return "truss_static"

    @property
    def priority(self) -> int:
        return 4

    @staticmethod
    def _is_pedicel(branch: dict) -> bool:
        return (
            branch.get("physics_profile") == "truss"
            and "_pedicel_" in branch.get("id", "").lower()
        )

    @staticmethod
    def _is_original_rachis(branch: dict) -> bool:
        branch_id = branch.get("id", "").lower()
        return (
            branch.get("physics_profile") == "truss"
            and "_rachis" in branch_id
            and "_pedicel_" not in branch_id
            and "_static_curve_" not in branch_id
        )

    @staticmethod
    def _truss_id_from_pedicel(branch: dict) -> str:
        return branch["id"].split("_pedicel_", 1)[0]

    @staticmethod
    def _clamp_unit(value: float) -> float:
        return min(max(value, 0.0), 1.0)

    def _dynamic_pedicel_groups(self, branches: List[dict]) -> Dict[str, List[dict]]:
        groups: Dict[str, List[dict]] = {}
        for branch in branches:
            if self._is_pedicel(branch) and branch.get("joint_type", "d6").lower() != "fixed":
                groups.setdefault(self._truss_id_from_pedicel(branch), []).append(branch)
        return groups

    def _static_candidates(self, branches: List[dict]) -> List[dict]:
        return sorted(
            (
                branch
                for branch in branches
                if self._is_original_rachis(branch)
                and branch.get("joint_type", "d6").lower() != "fixed"
            ),
            key=lambda branch: (-int(branch.get("n_links", 1)), branch["id"]),
        )

    def can_apply(self, branches: List[dict]) -> bool:
        return bool(self._dynamic_pedicel_groups(branches) or self._static_candidates(branches))

    def estimate_reduction(self, branches: List[dict]) -> int:
        pedicel_reduction = sum(
            int(branch.get("n_links", 1))
            for group in self._dynamic_pedicel_groups(branches).values()
            for branch in group
        )
        rachis_reduction = sum(
            max(int(branch.get("n_links", 1)) - 1, 0)
            for branch in self._static_candidates(branches)
        )
        return pedicel_reduction + rachis_reduction

    def apply(self, branches: List[dict]) -> Tuple[List[dict], OptimizationReport]:
        before = count_d6_joints(branches)
        pedicel_groups = self._dynamic_pedicel_groups(branches)

        if pedicel_groups:
            truss_id = sorted(pedicel_groups)[0]
            target_ids = {branch["id"] for branch in pedicel_groups[truss_id]}
            modified = [deepcopy(branch) for branch in branches]
            for branch in modified:
                if branch["id"] in target_ids:
                    branch["joint_type"] = "fixed"

            after = count_d6_joints(modified)
            return modified, OptimizationReport(
                technique_name=self.name,
                joints_before=before,
                joints_after=after,
                joints_saved=before - after,
                details={
                    "stage": "pedicels_fixed",
                    "truss_id": truss_id,
                    "pedicels_fixed": len(target_ids),
                },
            )

        candidates = self._static_candidates(branches)
        if not candidates:
            return branches, OptimizationReport(
                technique_name=self.name,
                joints_before=before,
                joints_after=before,
                joints_saved=0,
                details={"stage": "not_applicable"},
            )

        reducible = [branch for branch in candidates if int(branch.get("n_links", 1)) > 1]
        targets = reducible[:1] if reducible else candidates
        modified = branches
        for target in targets:
            current_rachis = next(branch for branch in modified if branch["id"] == target["id"])
            modified = self._make_static_curve(modified, current_rachis)
        after = count_d6_joints(modified)
        return modified, OptimizationReport(
            technique_name=self.name,
            joints_before=before,
            joints_after=after,
            joints_saved=before - after,
            details={
                "stage": "static_prebent",
                "truss_id": targets[0]["id"],
                "trusses_staticized": len(targets),
                "original_rachis_links": int(targets[0].get("n_links", 1)),
                "static_curve_pieces": self.curve_segments,
                "root_d6_joints": 1,
            },
        )

    def _remap_child_to_static_curve(
        self,
        child: dict,
        old_rachis_id: str,
        old_link_count: int,
        curve_ids: List[str],
    ) -> None:
        """Move a child branch from the dynamic rachis onto the static curve."""
        if child.get("parent") != old_rachis_id:
            return

        old_attach_link = int(child.get("attach_link", 1))
        old_attach_frac = float(child.get("attach_frac", 1.0))
        axial_fraction = (old_attach_link - 1 + old_attach_frac) / old_link_count
        scaled_position = self._clamp_unit(axial_fraction) * self.curve_segments
        curve_index = min(
            max(math.ceil(scaled_position) - 1, 0),
            self.curve_segments - 1,
        )
        local_fraction = scaled_position - curve_index

        child["parent"] = curve_ids[curve_index]
        child["attach_link"] = 1
        child["attach_frac"] = self._clamp_unit(local_fraction)
        if self._is_pedicel(child):
            child["joint_type"] = "fixed"
            child["tilt"] = float(child.get("tilt", 0.0)) + self.pedicel_droop_deg

    def _make_static_curve(self, branches: List[dict], rachis: dict) -> List[dict]:
        rachis_id = rachis["id"]
        old_link_count = int(rachis.get("n_links", 1))
        total_length = float(rachis.get("height", 0.0)) * old_link_count
        piece_height = total_length / self.curve_segments
        curve_ids = [
            f"{rachis_id}_static_curve_{index + 1:02d}"
            for index in range(self.curve_segments)
        ]

        curve = []
        for index, curve_id in enumerate(curve_ids):
            piece = deepcopy(rachis)
            piece.update(
                {
                    "id": curve_id,
                    "parent": rachis.get("parent") if index == 0 else curve_ids[index - 1],
                    "attach_link": rachis.get("attach_link") if index == 0 else 1,
                    "n_links": 1,
                    "height": piece_height,
                    "tilt": rachis.get("tilt", 0.0) if index == 0 else self.bend_per_segment_deg,
                    "rot": rachis.get("rot", 0.0) if index == 0 else 0.0,
                    "joint_type": "d6" if index == 0 else "fixed",
                    "optimization_state": "truss_static_curve",
                }
            )
            if index == 0:
                piece["bend_limit_deg"] = self.root_bend_limit_deg
                piece["drive_stiffness_scale"] = self.root_drive_stiffness_scale
            else:
                piece.pop("attach_frac", None)
                piece.pop("bend_limit_deg", None)
                piece.pop("drive_stiffness_scale", None)
            curve.append(piece)

        modified = []
        for branch in branches:
            if branch["id"] == rachis_id:
                modified.extend(curve)
                continue

            child = deepcopy(branch)
            self._remap_child_to_static_curve(
                child,
                rachis_id,
                old_link_count,
                curve_ids,
            )
            modified.append(child)

        return modified

    def validate(self, original: List[dict], modified: List[dict]) -> ValidationResult:
        errors = []
        modified_ids = {branch["id"] for branch in modified}

        for branch in modified:
            parent = branch.get("parent")
            if parent is not None and parent not in modified_ids:
                errors.append(f"Branch {branch['id']} has missing parent {parent}")

        original_pedicels = {
            branch["id"] for branch in original if self._is_pedicel(branch)
        }
        missing_pedicels = original_pedicels - modified_ids
        if missing_pedicels:
            errors.append(f"Pedicels removed: {sorted(missing_pedicels)}")

        if count_d6_joints(modified) > count_d6_joints(original):
            errors.append("Truss optimization increased the D6 joint count")

        return ValidationResult(valid=not errors, errors=errors, warnings=[])


class PedicelLockTechnique(TrussStaticTechnique):
    """Only the pedicel stage of ``truss_static``: lock one truss per pass.

    The rachis keeps its links, so no geometry is regenerated. This is the
    variant that is safe for PlantState branches, whose links carry
    per-organ ``link_specs``.
    """

    @property
    def name(self) -> str:
        return "pedicel_lock"

    def _static_candidates(self, branches: List[dict]) -> List[dict]:
        return []
