"""
lock_base.py - Base class for Joint Locking Techniques

Provides common logic for techniques that lock joints by changing their type to "fixed".
"""

from typing import List, Dict, Tuple
from abc import abstractmethod

try:
    from .base import OptimizationTechnique, OptimizationReport, ValidationResult, count_d6_joints
except ImportError:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from base import OptimizationTechnique, OptimizationReport, ValidationResult, count_d6_joints


class BaseLockTechnique(OptimizationTechnique):
    """
    Base class for techniques that convert D6 joints to Fixed.
    """
    
    @abstractmethod
    def _is_target(self, branch: dict) -> bool:
        """
        Check if a branch is a target for locking.
        Must be implemented by subclasses.
        """
        pass
    
    def _has_fixed_joint(self, branch: dict) -> bool:
        """
        Check if branch already has a fixed joint.
        """
        return branch.get("joint_type", "d6").lower() == "fixed"
    
    def can_apply(self, branches: List[dict]) -> bool:
        """
        Check if technique can be applied (i.e. there are targets not yet fixed).
        """
        for branch in branches:
            if self._is_target(branch) and not self._has_fixed_joint(branch):
                return True
        return False
    
    def estimate_reduction(self, branches: List[dict]) -> int:
        """
        Estimate D6 joint reduction.
        Every D6 joint converted to Fixed saves `n_links` D6 joints.
        """
        count = 0
        for branch in branches:
            if self._is_target(branch) and not self._has_fixed_joint(branch):
                count += branch.get("n_links", 1)
        return count
    
    def apply(self, branches: List[dict]) -> Tuple[List[dict], OptimizationReport]:
        """
        Apply the lock technique.
        Converts targeted joints from D6 to Fixed by adding metadata.
        """
        modified = []
        items_locked = 0
        items_found = 0
        
        joints_before_total = count_d6_joints(branches)
        
        for branch in branches:
            branch_copy = branch.copy()
            
            if self._is_target(branch_copy):
                items_found += 1
                
                if not self._has_fixed_joint(branch_copy):
                    branch_copy["joint_type"] = "fixed"
                    items_locked += 1
            
            modified.append(branch_copy)
            
        joints_after_total = count_d6_joints(modified)
        joints_saved = joints_before_total - joints_after_total
        
        report = OptimizationReport(
            technique_name=self.name,
            joints_before=joints_before_total,
            joints_after=joints_after_total,
            joints_saved=joints_saved,
            details={
                "items_found": items_found,
                "items_locked": items_locked,
                "dof_reduced": items_locked * 6,
            }
        )
        
        return modified, report
    
    def validate(self, original: List[dict], modified: List[dict]) -> ValidationResult:
        """
        Validate that optimization preserved topology and geometry.
        """
        errors = []
        warnings = []
        
        if len(original) != len(modified):
            errors.append(f"Branch count mismatch: {len(original)} → {len(modified)}")
            return ValidationResult(False, errors, warnings)
            
        orig_dict = {b["id"]: b for b in original}
        mod_dict = {b["id"]: b for b in modified}
        
        for branch_id in orig_dict:
            if branch_id not in mod_dict:
                errors.append(f"Branch {branch_id} missing after optimization")
                
        if errors:
            return ValidationResult(False, errors, warnings)
            
        for branch_id, orig_branch in orig_dict.items():
            mod_branch = mod_dict[branch_id]
            
            # Check parent relationship
            if orig_branch.get("parent") != mod_branch.get("parent"):
                errors.append(
                    f"Branch {branch_id}: parent changed "
                    f"{orig_branch.get('parent')} → {mod_branch.get('parent')}"
                )
                
            # Check geometry
            for key in ["n_links", "height", "radius"]:
                if key in orig_branch:
                    orig_val = orig_branch[key]
                    mod_val = mod_branch.get(key)
                    if orig_val != mod_val:
                        errors.append(
                            f"Branch {branch_id}: {key} changed {orig_val} → {mod_val}"
                        )
                        
            # Check attachment
            if "attach_link" in orig_branch:
                if orig_branch["attach_link"] != mod_branch.get("attach_link"):
                    errors.append(f"Branch {branch_id}: attachment changed")
                    
        # Check that only targets got joint_type metadata
        for branch_id, mod_branch in mod_dict.items():
            if "joint_type" in mod_branch:
                if not self._is_target(mod_branch):
                    warnings.append(
                        f"Non-target {branch_id} has joint_type metadata"
                    )
                    
        success = len(errors) == 0
        if success and not warnings:
            return ValidationResult(valid=True, errors=[], warnings=[])
            
        return ValidationResult(success, errors, warnings)
