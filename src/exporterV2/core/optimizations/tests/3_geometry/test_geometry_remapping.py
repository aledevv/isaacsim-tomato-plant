"""
test_geometry_remapping.py - Unit Tests for Geometry Remapping

Tests for attachment point remapping when collapsing segments.
Validates height preservation and edge cases using the new 1-based attach_frac model.

Run with: uv run pytest src/exporterV2/core/optimizations/tests/3_geometry/test_geometry_remapping.py
"""

import sys
import os
import pytest

# Add optimizations directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
optimizations_dir = os.path.join(script_dir, "../..")
sys.path.insert(0, optimizations_dir)

from geometry.remapping import remap_link_attachment

def test_remap_link_attachment_simple():
    """Test standard remapping (e.g., 10 to 3 links)."""
    # 10 links -> 3 links. Link 9 was at H=0.9
    k_new, p_new = remap_link_attachment(9, 10, 3)
    # H = 0.9, V = 2.7, k = 3, p = 0.7
    assert k_new == 3
    assert abs(p_new - 0.7) < 1e-6
    
def test_remap_link_attachment_top():
    """Test remapping the very top link."""
    # 10 links -> 3 links. Link 10 was at H=1.0
    k_new, p_new = remap_link_attachment(10, 10, 3)
    # H = 1.0 -> should map to k=3, p=1.0
    assert k_new == 3
    assert p_new == 1.0

def test_remap_link_attachment_bottom():
    """Test remapping near the bottom."""
    # 10 links -> 3 links. Link 1 was at H=0.1
    k_new, p_new = remap_link_attachment(1, 10, 3)
    # H = 0.1, V = 0.3, k = 1, p = 0.3
    assert k_new == 1
    assert abs(p_new - 0.3) < 1e-6

if __name__ == "__main__":
    pytest.main([__file__, "-v"])
