"""
Optimization Techniques

Plugin-based optimization techniques for joint budget reduction.
Each technique implements the OptimizationTechnique abstract base class.

Available Techniques (by priority):
    1.0 PetioleLockTechnique     - Convert petiolule D6 joints to Fixed
    1.5 ThinLinkLockTechnique    - Convert thin segment D6 joints to Fixed
    2.0 LateralBranchReductionTechnique - Reduce lateral branch segments
    3.0 StemCollapseTechnique    - Collapse main stem with child remapping
    4.0 TrussStaticTechnique     - Lock pedicels, then create a pre-bent truss block
    4.0 PedicelLockTechnique     - Lock pedicels only (no rachis geometry change)
    5.0 LeafBranchReductionTechnique    - Merge petiole+rachis into one segment
"""

from .base import OptimizationTechnique, OptimizationReport, ValidationResult, count_d6_joints
from .petiole_lock import PetioleLockTechnique
from .thin_link_lock import ThinLinkLockTechnique
from .lateral_reduce import LateralBranchReductionTechnique
from .stem_collapse import StemCollapseTechnique
from .leaf_branch_reduce import LeafBranchReductionTechnique
from .truss_static import PedicelLockTechnique, TrussStaticTechnique

__all__ = [
    "OptimizationTechnique",
    "OptimizationReport",
    "ValidationResult",
    "count_d6_joints",
    "PetioleLockTechnique",
    "ThinLinkLockTechnique",
    "LateralBranchReductionTechnique",
    "StemCollapseTechnique",
    "LeafBranchReductionTechnique",
    "TrussStaticTechnique",
    "PedicelLockTechnique",
]
