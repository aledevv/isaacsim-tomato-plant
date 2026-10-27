"""
usd - USD Generation Module

Handles USD stage creation, geometry, joints, and collision filtering.

Main components:
- stage: Stage setup and top-level orchestration
- geometry: Link and cylinder creation
- joints: Joint creation (flexible and locked)
- collision: Collision filtering logic
"""

try:
    from .stage import build_stage, build_stage_locked, get_output_usd_path
except ImportError as exc:
    if "PhysxSchema" not in str(exc):
        raise

    def _missing_physx_schema(*args, **kwargs):
        raise ImportError("USD stage building requires Isaac Sim PhysxSchema")

    build_stage = build_stage_locked = get_output_usd_path = _missing_physx_schema

__all__ = [
    'build_stage',
    'build_stage_locked',
    'get_output_usd_path',
]
