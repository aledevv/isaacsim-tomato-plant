"""Tests for terminal-body collision filtering policy."""

import sys
import importlib
from pathlib import Path
from types import SimpleNamespace

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


SRC_DIR = Path(__file__).resolve().parents[4]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

try:
    from pxr import PhysxSchema  # noqa: F401
except ImportError:
    import pxr

    class _PhysxRigidBodyAPI:
        def __init__(self, prim):
            self.prim = prim

        @classmethod
        def Apply(cls, prim):
            return cls(prim)

        @classmethod
        def Get(cls, stage, path):
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                return None
            return cls(prim)

        def CreateSolverPositionIterationCountAttr(self):
            return self.prim.CreateAttribute(
                "physxRigidBody:solverPositionIterationCount",
                Sdf.ValueTypeNames.Int,
            )

        def CreateSolverVelocityIterationCountAttr(self):
            return self.prim.CreateAttribute(
                "physxRigidBody:solverVelocityIterationCount",
                Sdf.ValueTypeNames.Int,
            )

        def GetSolverPositionIterationCountAttr(self):
            return self.prim.GetAttribute(
                "physxRigidBody:solverPositionIterationCount"
            )

        def GetSolverVelocityIterationCountAttr(self):
            return self.prim.GetAttribute(
                "physxRigidBody:solverVelocityIterationCount"
            )

    pxr.PhysxSchema = SimpleNamespace(PhysxRigidBodyAPI=_PhysxRigidBodyAPI)

stage_module = importlib.import_module("exporterV2.core.usd.stage")
terminal_bodies_module = importlib.import_module(
    "exporterV2.core.usd.terminal_bodies"
)
validate_terminal_body_clearance = (
    terminal_bodies_module.validate_terminal_body_clearance
)


def test_stage_reexports_terminal_clearance_validator():
    assert stage_module.validate_terminal_body_clearance is validate_terminal_body_clearance


def _filtered_targets(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.HasAPI(UsdPhysics.FilteredPairsAPI):
        return []
    return UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets()


def _build_overlapping_terminal_body_stage():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Stem")
    UsdGeom.Xform.Define(stage, "/World/TerminalBodies")
    UsdGeom.Xform.Define(stage, "/World/TerminalBodies/TomatoA")
    UsdGeom.Xform.Define(stage, "/World/TerminalBodies/TomatoB")
    UsdGeom.Xform.Define(stage, "/World/Stem/Pedicel_Link_01")

    records = [
        {
            "id": "TomatoA",
            "path": "/World/TerminalBodies/TomatoA",
            "parent_branch_id": "pedicel",
            "pos": (0.0, 0.0, 0.0),
            "radius": 0.05,
        },
        {
            "id": "TomatoB",
            "path": "/World/TerminalBodies/TomatoB",
            "parent_branch_id": "pedicel",
            "pos": (0.06, 0.0, 0.0),
            "radius": 0.05,
        },
    ]
    branch_registry = {
        "pedicel": (
            ["/World/Stem/Pedicel_Link_01"],
            [Gf.Vec3d(10.0, 0.0, 0.0)],
            (0.0, 0.0, 1.0),
            None,
        ),
    }
    branches = [
        {
            "id": "pedicel",
            "radius": 0.001,
            "height": 0.01,
        },
    ]
    return stage, records, branch_registry, branches


def test_terminal_body_pair_filter_can_be_disabled_for_overlap_experiments():
    stage, records, branch_registry, branches = _build_overlapping_terminal_body_stage()

    validate_terminal_body_clearance(
        records,
        branch_registry,
        branches,
        margin=0.0,
        stage=stage,
        apply_filters=True,
        filter_terminal_body_pairs=False,
    )

    assert Sdf.Path("/World/TerminalBodies/TomatoB") not in _filtered_targets(
        stage,
        "/World/TerminalBodies/TomatoA",
    )
    assert Sdf.Path("/World/TerminalBodies/TomatoA") not in _filtered_targets(
        stage,
        "/World/TerminalBodies/TomatoB",
    )


def test_terminal_body_pairs_are_filtered_when_overlap_filter_is_enabled():
    stage, records, branch_registry, branches = _build_overlapping_terminal_body_stage()

    validate_terminal_body_clearance(
        records,
        branch_registry,
        branches,
        margin=0.0,
        stage=stage,
        apply_filters=True,
        filter_terminal_body_pairs=True,
    )

    assert Sdf.Path("/World/TerminalBodies/TomatoB") in _filtered_targets(
        stage,
        "/World/TerminalBodies/TomatoA",
    )
    assert Sdf.Path("/World/TerminalBodies/TomatoA") in _filtered_targets(
        stage,
        "/World/TerminalBodies/TomatoB",
    )
