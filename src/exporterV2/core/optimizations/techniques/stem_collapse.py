"""
stem_collapse.py - Stem Collapse Optimization Technique

Reduces the number of segments (n_links) in the main trunk/stem incrementally.
This technique collapses trunk segments and remaps ALL child attachment points
(lateral branches, leaves, trusses) using geometry remapping from Task 3.

Rationale:
- Main trunk contributes heavily to joint count (often 10-20 links)
- Collapsing to fewer segments (e.g., 10→5→3) maintains structural minimum
- Child branches are remapped to preserve their absolute height

Priority Strategy:
- Applied after petiole lock and lateral reduce
- Uses new attach_frac remapping for precise height preservation
- Iteratively reduces until target_segments reached or budget met

Usage:
    technique = StemCollapseTechnique(target_segments=3)
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




class StemCollapseTechnique(OptimizationTechnique):
    """
    Stem Collapse: Reduce n_links in main trunk/stem.
    
    Priority: 3 (after lateral reduce - medium-high visual impact)
    Impact: Reduces joint count with geometry changes, remaps all children
    """
    
    def __init__(self, target_segments: int = 3):
        """
        Initialize stem collapse technique.
        
        Args:
            target_segments: Target number of trunk segments (default: 3)
        """
        self._name = "stem_collapse"
        self._priority = 3
        self._target_segments = max(1, target_segments)  # Minimum 1
    
    @property
    def name(self) -> str:
        """Technique identifier."""
        return self._name
    
    @property
    def priority(self) -> int:
        """Execution priority (lower = earlier)."""
        return self._priority
    
    def _find_trunk(self, branches: List[dict]) -> dict:
        """
        Find the main trunk branch.
        
        Trunk is identified as the branch with parent=None.
        
        Args:
            branches: List of branch configurations
        
        Returns:
            Trunk branch dict, or None if not found
        """
        for branch in branches:
            if branch.get("parent") is None:
                return branch
        return None
    
    def _find_trunk_children(self, trunk_id: str, branches: List[dict]) -> List[dict]:
        """
        Find all direct children of trunk.
        
        Args:
            trunk_id: ID of trunk branch
            branches: List of all branches
        
        Returns:
            List of child branches attached to trunk
        """
        return [b for b in branches if b.get("parent") == trunk_id]
    
    def can_apply(self, branches: List[dict]) -> bool:
        """
        Check if technique can be applied.
        
        Args:
            branches: List of branch configurations
        
        Returns:
            True if trunk exists and has more segments than target
        """
        trunk = self._find_trunk(branches)
        if not trunk:
            return False
        if trunk.get("joint_type", "d6").lower() == "fixed":
            return False
        
        current_links = trunk.get("n_links", 1)
        return current_links > self._target_segments
    
    def estimate_reduction(self, branches: List[dict]) -> int:
        """
        Estimate joint reduction.
        
        Args:
            branches: List of branch configurations
        
        Returns:
            Estimated joint reduction (trunk links removed)
        """
        trunk = self._find_trunk(branches)
        if not trunk:
            return 0
        if trunk.get("joint_type", "d6").lower() == "fixed":
            return 0
        
        current_links = trunk.get("n_links", 1)
        if current_links <= self._target_segments:
            return 0
        
        return current_links - self._target_segments
    
    def apply(self, branches: List[dict]) -> Tuple[List[dict], OptimizationReport]:
        """
        Apply stem collapse technique.
        
        Collapses trunk to target_segments and remaps all children.
        
        Args:
            branches: List of branch configurations
        
        Returns:
            (modified_branches, report) tuple
        """
        trunk = self._find_trunk(branches)
        
        if not trunk or trunk.get("joint_type", "d6").lower() == "fixed":
            report = OptimizationReport(
                technique_name=self.name,
                joints_before=count_d6_joints(branches),
                joints_after=count_d6_joints(branches),
                joints_saved=0,
                details={
                    "trunk_found": trunk is not None,
                    "trunk_fixed": trunk is not None,
                    "original_links": 0,
                    "final_links": 0,
                    "links_removed": 0,
                    "children_remapped": 0,
                }
            )
            return branches, report
        
        trunk_id = trunk["id"]
        original_links = trunk.get("n_links", 1)
        
        # Check if reduction needed
        if original_links <= self._target_segments:
            report = OptimizationReport(
                technique_name=self.name,
                joints_before=count_d6_joints(branches),
                joints_after=count_d6_joints(branches),
                joints_saved=0,
                details={
                    "trunk_found": True,
                    "original_links": original_links,
                    "final_links": original_links,
                    "links_removed": 0,
                    "children_remapped": 0,
                }
            )
            return branches, report
        
        # Create modified branches list
        modified = []
        children_remapped = 0
        
        for branch in branches:
            branch_copy = branch.copy()
            
            if branch["id"] == trunk_id:
                # Collapse trunk and recalculate height to preserve total length
                old_height = branch.get("height", 0.1)
                new_height = old_height * original_links / self._target_segments
                
                branch_copy["n_links"] = self._target_segments
                branch_copy["height"] = new_height
                modified.append(branch_copy)
            
            elif branch.get("parent") == trunk_id:
                # Remap child attachment
                if "attach_link" in branch:
                    old_attach_link = branch["attach_link"]
                    old_attach_frac = branch.get("attach_frac", 1.0)
                    
                    # Use remapping function
                    if remap_link_attachment:
                        new_attach_link, new_attach_frac = remap_link_attachment(
                            old_attach_link,
                            original_links,
                            self._target_segments,
                            old_attach_frac,
                        )
                        
                        branch_copy["attach_link"] = new_attach_link
                        branch_copy["attach_frac"] = new_attach_frac
                        children_remapped += 1
                    else:
                        # Fallback: proportional remapping without geometry module
                        normalized = (
                            old_attach_link - 1 + old_attach_frac
                        ) / original_links
                        scaled = normalized * self._target_segments
                        if normalized >= 1.0:
                            new_attach_link = self._target_segments
                            new_attach_frac = 1.0
                        else:
                            new_attach_link = int(scaled) + 1
                            new_attach_frac = scaled - int(scaled)
                        branch_copy["attach_link"] = new_attach_link
                        branch_copy["attach_frac"] = new_attach_frac
                        children_remapped += 1
                
                modified.append(branch_copy)
            
            else:
                # Other branches unchanged
                modified.append(branch_copy)
        
        # Calculate metrics
        links_removed = original_links - self._target_segments
        joints_before = count_d6_joints(branches)
        joints_after = count_d6_joints(modified)
        
        report = OptimizationReport(
            technique_name=self.name,
            joints_before=joints_before,
            joints_after=joints_after,
            joints_saved=links_removed,
            details={
                "trunk_found": True,
                "original_links": original_links,
                "final_links": self._target_segments,
                "links_removed": links_removed,
                "children_remapped": children_remapped,
            }
        )
        
        return modified, report
    
    def validate(self, original: List[dict], modified: List[dict]) -> ValidationResult:
        """
        Validate that optimization preserved topology.
        
        Checks:
        - Trunk still exists
        - Trunk has target_segments links
        - All children still attached
        - No orphaned branches
        
        Args:
            original: Original branch configuration
            modified: Modified branch configuration
        
        Returns:
            ValidationResult with success flag and messages
        """
        errors = []
        warnings = []
        
        # Find original and modified trunk
        orig_trunk = self._find_trunk(original)
        mod_trunk = self._find_trunk(modified)
        
        if not orig_trunk:
            errors.append("Original trunk not found")
            return ValidationResult(False, errors, warnings)
        
        if not mod_trunk:
            errors.append("Modified trunk not found (trunk removed!)")
            return ValidationResult(False, errors, warnings)
        
        # Check trunk links
        if mod_trunk.get("n_links", 1) != self._target_segments:
            errors.append(
                f"Trunk should have {self._target_segments} links, "
                f"got {mod_trunk.get('n_links', 1)}"
            )
        
        # Check children still exist
        orig_trunk_id = orig_trunk["id"]
        mod_trunk_id = mod_trunk["id"]
        
        orig_children = self._find_trunk_children(orig_trunk_id, original)
        mod_children = self._find_trunk_children(mod_trunk_id, modified)
        
        if len(orig_children) != len(mod_children):
            errors.append(
                f"Child count mismatch: {len(orig_children)} → {len(mod_children)}"
            )
        
        # Check no orphaned branches
        mod_dict = {b["id"]: b for b in modified}
        for branch in modified:
            parent_id = branch.get("parent")
            if parent_id and parent_id not in mod_dict:
                errors.append(
                    f"Branch {branch['id']}: parent {parent_id} does not exist (orphaned)"
                )
        
        success = len(errors) == 0
        return ValidationResult(success, errors, warnings)
