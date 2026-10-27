"""
remapping.py - Attachment Point Remapping

When collapsing stem segments (e.g., 10 links → 3 links), child branches
attached to the stem need their attachment points recalculated to preserve
their absolute height in the world.

Mathematical model:
    H_abs = (attach_link - 1 + p) / N_old (fraction of total height)
    V     = H_abs * N_new
    k_new = floor(V) + 1  if H<1.0,  else N_new   (1-based, edge-case guarded)
    p_new = V - floor(V)               (fraction within new link)
"""

import math
from typing import Tuple

def remap_link_attachment(
    attach_link: int,
    n_old: int,
    n_new: int,
    attach_frac: float = 1.0,
) -> Tuple[int, float]:
    """
    Remap a 1-based attach_link from n_old segments to n_new segments.
    Existing callers retain the top-of-link convention through the default
    ``attach_frac=1.0``.
    
    Args:
        attach_link: 1-based index of original link
        n_old: Number of links in original trunk
        n_new: Number of links in new reduced trunk
        
    Returns:
        (k_new, p_new):
            k_new: 1-based index of new link
            p_new: Fractional offset within new link (0.0=base, 1.0=top)
    """
    H = (attach_link - 1 + attach_frac) / n_old
    if H >= 1.0:
        return n_new, 1.0
    V = H * n_new
    k = math.floor(V) + 1
    p = V - math.floor(V)
    return k, p
