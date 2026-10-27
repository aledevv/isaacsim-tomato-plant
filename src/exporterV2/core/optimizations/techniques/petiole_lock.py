"""
petiole_lock.py - Petiole Lock Optimization Technique

Converts petiolule joints from D6 (articulated, 6 DOF) to Fixed (static, 0 DOF).
This reduces computational cost without changing geometry or visual appearance.

Rationale:
- Petiolules (leaflet stems) are small and have limited movement in practice
- Converting to fixed joints maintains visual fidelity while reducing DOF
- Minimal impact on simulation realism for plant modeling

Usage:
    technique = PetioleLockTechnique()
    if technique.can_apply(branches):
        reduction = technique.estimate_reduction(branches)
        modified, report = technique.apply(branches)
"""

try:
    from .lock_base import BaseLockTechnique
except ImportError:
    import sys
    import os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from lock_base import BaseLockTechnique


class PetioleLockTechnique(BaseLockTechnique):
    """
    Petiole Lock: Convert petiolule joints from D6 to Fixed.
    
    Priority: 1 (highest - minimal visual impact)
    Impact: Reduces DOF without geometry changes
    """
    
    def __init__(self, params: dict = None):
        """Initialize petiole lock technique."""
        super().__init__(params)
        self._name = "petiole_lock"
        self._priority = 1
    
    @property
    def name(self) -> str:
        """Technique identifier."""
        return self._name
    
    @property
    def priority(self) -> int:
        """Execution priority (lower = earlier)."""
        return self._priority
    
    def _is_target(self, branch: dict) -> bool:
        """
        Check if a branch is a petiolule.
        
        Petiolules are identified by an explicit PlantState ``kind`` or, for
        BRANCHES without one, by a name containing "petiolule"
        (e.g., "Leaf_r1_o0_rachis_petiolule_lat_0_left"). PlantState terminal
        petiolules are named "..._rachis_terminal_..." and need the kind.
        
        Args:
            branch: Branch configuration dict
        
        Returns:
            True if branch is a petiolule
        """
        if branch.get("kind") == "petiolule":
            return True
        branch_id = branch.get("id", "").lower()
        return "petiolule" in branch_id
