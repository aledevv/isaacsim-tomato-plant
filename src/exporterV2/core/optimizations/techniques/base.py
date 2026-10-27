"""
base.py - Abstract Base Class for Optimization Techniques

Defines the interface that all optimization techniques must implement.
"""

from abc import ABC, abstractmethod
from typing import List, Dict, Tuple
from dataclasses import dataclass, field


def count_d6_joints(branches: List[Dict]) -> int:
    """
    Count only D6 joints (excludes Fixed joints).
    
    Fixed joints (locked petiolules) don't count toward budget because
    they don't contribute to simulation complexity in PhysX.
    
    Args:
        branches: List of branch configurations
    
    Returns:
        Total D6 joint count
    """
    total = 0
    for branch in branches:
        # Skip Fixed joints (locked petiolules)
        if branch.get("joint_type", "d6").lower() == "fixed":
            continue
        total += branch.get("n_links", 1)
    return total


@dataclass
class OptimizationReport:
    """Report for a single technique application."""
    technique_name: str
    joints_before: int
    joints_after: int
    joints_saved: int
    details: Dict[str, any] = field(default_factory=dict)
    
    def __str__(self) -> str:
        """Human-readable report string."""
        lines = [
            f"Technique: {self.technique_name}",
            f"  Joints: {self.joints_before} → {self.joints_after} (-{self.joints_saved})",
        ]
        if self.details:
            lines.append("  Details:")
            for key, value in self.details.items():
                lines.append(f"    - {key}: {value}")
        return "\n".join(lines)


@dataclass
class ValidationResult:
    """Result of geometry/collision validation."""
    valid: bool
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    
    def __str__(self) -> str:
        """Human-readable validation result."""
        status = "✓ VALID" if self.valid else "✗ INVALID"
        lines = [f"Validation: {status}"]
        
        if self.errors:
            lines.append("  Errors:")
            for error in self.errors:
                lines.append(f"    - {error}")
        
        if self.warnings:
            lines.append("  Warnings:")
            for warning in self.warnings:
                lines.append(f"    - {warning}")
        
        return "\n".join(lines)


class OptimizationTechnique(ABC):
    """
    Abstract base class for all optimization techniques.
    
    Each concrete technique must implement:
        - name: Technique name for reporting
        - priority: Priority level (lower = applied first)
        - can_apply: Check if technique is applicable
        - estimate_reduction: Estimate joints saved
        - apply: Apply technique and return modified config
        - validate: Validate result (geometry, collisions, structural)
    
    Example:
        class MyTechnique(OptimizationTechnique):
            @property
            def name(self) -> str:
                return "My Custom Technique"
            
            @property
            def priority(self) -> int:
                return 6  # Lower than existing techniques
            
            def can_apply(self, branches, current_joints, budget):
                # Check if applicable
                return True
            
            # ... implement other methods
    """
    
    def __init__(self, params: Dict = None):
        """
        Initialize technique with optional parameters from config.
        
        Args:
            params: Technique-specific parameters from budget_config.yaml
        """
        self.params = params or {}
    
    @property
    @abstractmethod
    def name(self) -> str:
        """Technique name for reporting."""
        pass
    
    @property
    @abstractmethod
    def priority(self) -> int:
        """
        Priority level (lower number = applied first).
        
        Standard priorities:
            1 = Petiole Lock
            2 = Lateral Reduce
            3 = Stem Collapse
            4 = Truss Static
            5 = Leaf Branch Reduce
        """
        pass
    
    @abstractmethod
    def can_apply(self, branches: List[Dict]) -> bool:
        """
        Check if technique can be applied to current configuration.
        
        Args:
            branches: Current branches configuration
        
        Returns:
            True if technique is applicable, False otherwise
        
        Example:
            def can_apply(self, branches):
                # Check if there are lateral branches with > min_segments
                min_seg = self.params.get("min_segments", 1)
                for b in branches:
                    if self._is_lateral_branch(b) and b["n_links"] > min_seg:
                        return True
                return False
        """
        pass
    
    @abstractmethod
    def estimate_reduction(self, branches: List[Dict]) -> int:
        """
        Estimate how many joints this technique would save.
        
        Args:
            branches: Current branches configuration
        
        Returns:
            Estimated number of joints saved (positive integer)
        
        Note:
            This is an estimate - actual reduction may differ after validation.
        
        Example:
            def estimate_reduction(self, branches):
                # Count how many links can be reduced
                reduction = 0
                for b in branches:
                    if self._is_lateral_branch(b):
                        reduction += max(0, b["n_links"] - self.params["min_segments"])
                return reduction
        """
        pass
    
    @abstractmethod
    def apply(self, branches: List[Dict]) -> Tuple[List[Dict], OptimizationReport]:
        """
        Apply the technique and return modified configuration + report.
        
        Args:
            branches: Current branches configuration
        
        Returns:
            Tuple (modified_branches, report):
                modified_branches: Updated branches configuration
                report: OptimizationReport with details
        
        Raises:
            ValueError: If technique cannot be applied or validation fails
        
        Example:
            def apply(self, branches):
                joints_before = self._count_joints(branches)
                modified = self._reduce_branches(branches)
                joints_after = self._count_joints(modified)
                
                report = OptimizationReport(
                    technique_name=self.name,
                    joints_before=joints_before,
                    joints_after=joints_after,
                    joints_saved=joints_before - joints_after,
                    details={"branches_modified": 5}
                )
                
                return (modified, report)
        """
        pass
    
    @abstractmethod
    def validate(self, original: List[Dict], modified: List[Dict]) -> ValidationResult:
        """
        Validate that the modified configuration is structurally correct.

        Called by the optimizer after each apply() to decide whether to keep
        the modification. If validation fails, the modification is discarded.

        Args:
            original: Branch configuration BEFORE apply() was called
            modified: Branch configuration AFTER apply() was called

        Returns:
            ValidationResult with validation status, errors, and warnings

        Example:
            def validate(self, original, modified):
                errors = []
                warnings = []

                # Check branch count unchanged
                if len(original) != len(modified):
                    errors.append("Branch count changed unexpectedly")

                # Check no orphaned parents
                mod_ids = {b["id"] for b in modified}
                for b in modified:
                    if b.get("parent") and b["parent"] not in mod_ids:
                        errors.append(f"{b['id']}: parent missing")

                return ValidationResult(
                    valid=len(errors) == 0,
                    errors=errors,
                    warnings=warnings
                )
        """
        pass
