"""Verify that tree_config runtime defaults are authored into USD."""

import sys
from pathlib import Path


simulation_app = None
if __name__ == "__main__":
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": True})

try:
    from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics
except ImportError:
    if __name__ == "__main__":
        raise
    import pytest

    pytest.skip("PhysX schema requires Isaac Sim", allow_module_level=True)


SRC_DIR = Path(__file__).resolve().parents[4]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from exporterV2.core.physics import (
    apply_physx_articulation_settings,
    apply_physx_rigid_body_solver_settings,
    apply_physx_scene_settings,
)
from exporterV2.core.tree_config import PhysicsRuntimeConfig


def test_runtime_defaults_are_authored_from_tree_config():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    stem = UsdGeom.Xform.Define(stage, "/World/Stem")
    UsdPhysics.ArticulationRootAPI.Apply(stem.GetPrim())

    apply_physx_scene_settings(stage)
    apply_physx_articulation_settings(stage, "/World/Stem")

    scene_api = PhysxSchema.PhysxSceneAPI.Get(stage, "/World/PhysicsScene")
    articulation_api = PhysxSchema.PhysxArticulationAPI.Get(stage, "/World/Stem")
    assert scene_api.GetTimeStepsPerSecondAttr().Get() == PhysicsRuntimeConfig.PHYSICS_HZ
    assert scene_api.GetEnableGPUDynamicsAttr().Get() is PhysicsRuntimeConfig.ENABLE_GPU_DYNAMICS
    assert (
        articulation_api.GetSolverPositionIterationCountAttr().Get()
        == PhysicsRuntimeConfig.SOLVER_POSITION_ITERATIONS
    )
    assert (
        articulation_api.GetSolverVelocityIterationCountAttr().Get()
        == PhysicsRuntimeConfig.SOLVER_VELOCITY_ITERATIONS
    )


def test_explicit_runtime_arguments_override_tree_config():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    stem = UsdGeom.Xform.Define(stage, "/World/Stem")
    UsdPhysics.ArticulationRootAPI.Apply(stem.GetPrim())

    apply_physx_scene_settings(stage, physics_hz=120, enable_gpu_dynamics=False)
    apply_physx_articulation_settings(stage, "/World/Stem", 8, 2)

    scene_api = PhysxSchema.PhysxSceneAPI.Get(stage, "/World/PhysicsScene")
    articulation_api = PhysxSchema.PhysxArticulationAPI.Get(stage, "/World/Stem")
    assert scene_api.GetTimeStepsPerSecondAttr().Get() == 120
    assert scene_api.GetEnableGPUDynamicsAttr().Get() is False
    assert articulation_api.GetSolverPositionIterationCountAttr().Get() == 8
    assert articulation_api.GetSolverVelocityIterationCountAttr().Get() == 2


def test_terminal_body_solver_defaults_are_authored_from_tree_config():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    tomato = UsdGeom.Sphere.Define(stage, "/World/tomato")
    UsdPhysics.RigidBodyAPI.Apply(tomato.GetPrim())

    apply_physx_rigid_body_solver_settings(stage, "/World/tomato")

    rigid_body_api = PhysxSchema.PhysxRigidBodyAPI.Get(stage, "/World/tomato")
    assert (
        rigid_body_api.GetSolverPositionIterationCountAttr().Get()
        == PhysicsRuntimeConfig.TERMINAL_BODY_SOLVER_POSITION_ITERATIONS
    )
    assert (
        rigid_body_api.GetSolverVelocityIterationCountAttr().Get()
        == PhysicsRuntimeConfig.TERMINAL_BODY_SOLVER_VELOCITY_ITERATIONS
    )


if __name__ == "__main__":
    try:
        test_runtime_defaults_are_authored_from_tree_config()
        test_explicit_runtime_arguments_override_tree_config()
        test_terminal_body_solver_defaults_are_authored_from_tree_config()
        print("RUNTIME PHYSICS CONFIG TESTS PASSED")
    finally:
        simulation_app.close()
