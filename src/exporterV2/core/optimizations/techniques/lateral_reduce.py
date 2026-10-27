"""
lateral_reduce.py - Lateral Branch Reduction Optimization Technique

Reduces the number of segments (n_links) in lateral branches incrementally.
This technique collapses segments and remaps child attachment points (leaves, etc.)
using geometry remapping from Task 3.

Rationale:
- Lateral branches contribute significantly to joint count in mature plants
- Reducing segments from 3→2→1 maintains structural integrity with less articulation
- Child branches (leaves) are remapped to preserve geometric positioning

Priority Strategy:
- Reduce smallest branches first (by radius) - least significant for plant mechanics
- In case of tie (same radius), reduce lower branches first (by attach_link)
- In case of tie (same attach + radius), alphabetically by ID

Usage:
    technique = LateralBranchReductionTechnique()
    if technique.can_apply(branches):
        reduction = technique.estimate_reduction(branches)
        modified, report = technique.apply(branches)
"""

from typing import List, Dict, Tuple

try:
    from .base import OptimizationTechnique, OptimizationReport, ValidationResult, count_d6_joints
except ImportError:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from base import OptimizationTechnique, OptimizationReport, ValidationResult, count_d6_joints

# Import geometry remapping (Task 3)
try:
    from ..geometry.remapping import remap_link_attachment
except ImportError:
    try:
        import sys
        import os
        sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
        from geometry.remapping import remap_link_attachment
    except ImportError:
        # Fallback if remapping not available
        remap_link_attachment = None




class LateralBranchReductionTechnique(OptimizationTechnique):
    """
    Lateral Branch Reduction: Reduce n_links in lateral branches.
    
    Priority: 2 (after petiole lock - medium visual impact)
    Impact: Reduces joint count with geometry changes
    """
    
    def __init__(self, min_segments: int = 1):
        """
        Initialize lateral branch reduction technique.
        
        Args:
            min_segments: Minimum segments to preserve (default: 1)
        """
        self._name = "lateral_reduce"
        self._priority = 2
        self._min_segments = min_segments
    
    @property
    def name(self) -> str:
        """Technique identifier."""
        return self._name
    
    @property
    def priority(self) -> int:
        """Execution priority (lower = earlier)."""
        return self._priority
    
    def _is_lateral_branch(self, branch: dict) -> bool:
        """
        Check if a branch is a lateral branch.
        
        Lateral branches are identified by naming pattern: "Branch_r*_o*"
        
        Args:
            branch: Branch configuration dict
        
        Returns:
            True if branch is a lateral branch
        """
        branch_id = branch.get("id", "")
        return branch_id.startswith("Branch_r") and "_o" in branch_id
    
    def _is_lateral_leaf(self, branch: dict) -> bool:
        """
        Check if a branch is a lateral leaf.
        
        Lateral leaves are identified by naming pattern: "LatLeaf_*"
        from the CSV fixture builder, with "LateralLeaf_*" kept for older tests.
        
        Args:
            branch: Branch configuration dict
        
        Returns:
            True if branch is a lateral leaf
        """
        branch_id = branch.get("id", "")
        return (
            branch_id.startswith("LatLeaf_r")
            or branch_id.startswith("LateralLeaf_r")
        ) and "_o" in branch_id
    
    def _can_reduce(self, branch: dict) -> bool:
        """
        Check if a lateral branch can be reduced.

        Args:
            branch: Branch configuration dict

        Returns:
            True if the branch is a non-Fixed lateral branch/leaf with n_links > min_segments.
            Fixed branches (locked by petiole_lock or thin_link_lock) are excluded because
            reducing their n_links saves zero D6 joints and would produce a wrong report.
        """
        if not (self._is_lateral_branch(branch) or self._is_lateral_leaf(branch)):
            return False
        if branch.get("joint_type", "d6").lower() == "fixed":
            return False
        n_links = branch.get("n_links", 1)
        return n_links > self._min_segments
    
    def _get_reduction_priority(self, branch: dict) -> Tuple[float, int, str]:
        """
        Calculate priority for reduction (lower value = reduce first).
        
        Priority order:
        1. Smallest radius (least significant branches)
        2. Lower attach_link (bottom branches first)
        3. Alphabetically by ID
        
        Args:
            branch: Branch configuration dict
        
        Returns:
            (radius, attach_link, branch_id) tuple for sorting
        """
        radius = branch.get("radius", 1.0)
        attach_link = branch.get("attach_link", 0)
        branch_id = branch.get("id", "")
        
        # Lower attach_link = higher priority (so use attach_link directly)
        return (radius, attach_link, branch_id)
    
    def can_apply(self, branches: List[dict]) -> bool:
        """
        Check if technique can be applied.
        
        Args:
            branches: List of branch configurations
        
        Returns:
            True if there are lateral branches/leaves that can be reduced
        """
        for branch in branches:
            if self._can_reduce(branch):
                return True
        return False
    
    def estimate_reduction(self, branches: List[dict]) -> int:
        """
        Estimate joint reduction.
        
        Counts total links that could be removed from all reducible branches.
        
        Args:
            branches: List of branch configurations
        
        Returns:
            Estimated joint reduction
        """
        total_reduction = 0
        for branch in branches:
            if self._can_reduce(branch):
                n_links = branch.get("n_links", 1)
                # Can reduce down to min_segments
                reducible = n_links - self._min_segments
                total_reduction += reducible
        
        return total_reduction
    
    def apply(self, branches: List[dict]) -> Tuple[List[dict], OptimizationReport]:
        """
        Apply lateral branch reduction technique.
        
        Reduces n_links by 1 for all applicable branches in priority order,
        remapping child attachment points using geometry remapping.
        
        Args:
            branches: List of branch configurations
        
        Returns:
            (modified_branches, report) tuple
        """
        # Find reducible branches
        reducible = [b for b in branches if self._can_reduce(b)]
        
        if not reducible:
            # No branches to reduce
            report = OptimizationReport(
                technique_name=self.name,
                joints_before=count_d6_joints(branches),
                joints_after=count_d6_joints(branches),
                joints_saved=0,
                details={
                    "branches_found": 0,
                    "branches_reduced": 0,
                    "links_removed": 0,
                    "children_remapped": 0,
                }
            )
            return branches, report
        
        # Sort by priority (smallest → lowest → alphabetical)
        reducible.sort(key=self._get_reduction_priority)
        
        # Create branch lookup dict
        branch_dict = {b["id"]: b for b in branches}
        
        # Track modifications
        branches_reduced = 0
        links_removed = 0
        children_remapped = 0
        
        # Reduce each branch by 1 link
        for branch in reducible:
            branch_id = branch["id"]
            old_n_links = branch["n_links"]
            new_n_links = old_n_links - 1
            
            # Update n_links
            branch_dict[branch_id]["n_links"] = new_n_links
            
            # Recalculate height to preserve total length
            old_height = branch["height"]
            new_height = old_height * old_n_links / new_n_links
            branch_dict[branch_id]["height"] = new_height
            
            branches_reduced += 1
            links_removed += 1
            
            # Remap children attachment points
            children = [b for b in branches if b.get("parent") == branch_id]
            
            for child in children:
                old_attach_link = child["attach_link"]
                old_attach_frac = child.get("attach_frac", 1.0)
                
                # Calculate absolute height position (0-indexed continuous)
                # Convert 1-based link + frac to absolute position
                abs_height_fraction = (old_attach_link - 1 + old_attach_frac) / old_n_links
                
                # Map to new coordinate system
                new_position = abs_height_fraction * new_n_links
                new_attach_link = int(new_position) + 1  # Convert back to 1-based
                new_attach_frac = new_position - int(new_position)
                
                # Clamp to valid range
                if new_attach_link > new_n_links:
                    new_attach_link = new_n_links
                    new_attach_frac = 1.0
                elif new_attach_link < 1:
                    new_attach_link = 1
                    new_attach_frac = 0.0
                
                # Update child's attach_link and attach_frac
                child_id = child["id"]
                branch_dict[child_id]["attach_link"] = new_attach_link
                branch_dict[child_id]["attach_frac"] = new_attach_frac
                children_remapped += 1
        
        # Convert back to list
        modified = list(branch_dict.values())
        
        # Create report
        report = OptimizationReport(
            technique_name=self.name,
            joints_before=count_d6_joints(branches),
            joints_after=count_d6_joints(modified),
            joints_saved=links_removed,
            details={
                "branches_found": len(reducible),
                "branches_reduced": branches_reduced,
                "links_removed": links_removed,
                "children_remapped": children_remapped,
            }
        )
        
        return modified, report
    
    def validate(self, original: List[dict], modified: List[dict]) -> ValidationResult:
        """
        Validate that optimization preserved topology.
        
        Checks:
        - Same number of branches
        - Same branch IDs
        - Same parent-child relationships
        - n_links reduced only for lateral branches/leaves
        - n_links >= min_segments
        - Total length preserved (height * n_links)
        - Child attachments remapped correctly
        
        Args:
            original: Original branch configuration
            modified: Modified branch configuration
        
        Returns:
            ValidationResult with success flag and messages
        """
        errors = []
        warnings = []
        
        # Check branch count
        if len(original) != len(modified):
            errors.append(
                f"Branch count mismatch: {len(original)} → {len(modified)}"
            )
            return ValidationResult(False, errors, warnings)
        
        # Create lookup dicts
        orig_dict = {b["id"]: b for b in original}
        mod_dict = {b["id"]: b for b in modified}
        
        # Check all branches exist
        for branch_id in orig_dict:
            if branch_id not in mod_dict:
                errors.append(f"Branch {branch_id} missing after optimization")
        
        if errors:
            return ValidationResult(False, errors, warnings)
        
        # Check each branch
        for branch_id, orig_branch in orig_dict.items():
            mod_branch = mod_dict[branch_id]
            
            # Check parent relationship
            if orig_branch.get("parent") != mod_branch.get("parent"):
                errors.append(
                    f"Branch {branch_id}: parent changed "
                    f"{orig_branch.get('parent')} → {mod_branch.get('parent')}"
                )
            
            # Check n_links changes
            orig_n_links = orig_branch.get("n_links", 1)
            mod_n_links = mod_branch.get("n_links", 1)
            
            if mod_n_links != orig_n_links:
                # Only lateral branches/leaves should change
                if not (self._is_lateral_branch(orig_branch) or 
                        self._is_lateral_leaf(orig_branch)):
                    errors.append(
                        f"Branch {branch_id}: non-lateral branch n_links changed"
                    )
                
                # Check minimum constraint
                if mod_n_links < self._min_segments:
                    errors.append(
                        f"Branch {branch_id}: n_links {mod_n_links} < min {self._min_segments}"
                    )
                
                # Check total length preservation
                orig_length = orig_n_links * orig_branch.get("height", 0)
                mod_length = mod_n_links * mod_branch.get("height", 0)
                length_error = abs(orig_length - mod_length) / orig_length if orig_length > 0 else 0
                
                if length_error > 0.01:  # 1% tolerance
                    warnings.append(
                        f"Branch {branch_id}: total length changed by {length_error*100:.1f}%"
                    )
            
            # Check radius unchanged
            if orig_branch.get("radius") != mod_branch.get("radius"):
                errors.append(
                    f"Branch {branch_id}: radius changed"
                )
            
            # Check attachment for children
            children = [b for b in modified if b.get("parent") == branch_id]
            for child in children:
                attach = child.get("attach_link", 1)
                parent_n_links = mod_branch.get("n_links", 1)
                
                if attach > parent_n_links or attach < 1:
                    errors.append(
                        f"Child {child['id']}: attach_link {attach} invalid for parent n_links {parent_n_links}"
                    )
        
        success = len(errors) == 0
        return ValidationResult(success, errors, warnings)
