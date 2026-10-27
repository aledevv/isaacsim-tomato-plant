"""
physics.py - PhysX Configuration

Shared PhysX scene and articulation settings for Isaac Sim simulations.
"""

import math

from pxr import UsdPhysics, Gf, Sdf

try:  # OpenUSD wheels do not ship NVIDIA's schema plugin.
    from pxr import PhysxSchema
except ImportError:  # pragma: no cover - exercised by the serverless exporter
    PhysxSchema = None

from .tree_config import PhysicsRuntimeConfig


def _apply_schema_token(prim, token: str) -> None:
    """Author an applied PhysX schema token without requiring Isaac Sim."""

    schemas = list(prim.GetAppliedSchemas())
    if token in schemas:
        return
    # Standard OpenUSD APIs are commonly stored as explicit list items. Mixing
    # a prepended unknown NVIDIA token into that operation can discard them
    # when another standard API is applied later. Preserve the complete
    # resolved list explicitly.
    prim.SetMetadata(
        "apiSchemas",
        Sdf.TokenListOp.CreateExplicit([*schemas, token]),
    )


def _attribute(prim, name: str, value_type, value) -> None:
    prim.CreateAttribute(name, value_type).Set(value)


def apply_physx_scene_settings(
    stage,
    physics_hz: float | None = None,
    enable_gpu_dynamics: bool | None = None,
) -> None:
    """
    Configure PhysicsScene for stiff articulation drives.
    
    Settings optimized for plant stems with high stiffness joints.
    """
    if physics_hz is None:
        physics_hz = PhysicsRuntimeConfig.PHYSICS_HZ
    if enable_gpu_dynamics is None:
        enable_gpu_dynamics = PhysicsRuntimeConfig.ENABLE_GPU_DYNAMICS
    if physics_hz <= 0:
        raise ValueError("physics_hz must be positive")

    scene_path = "/World/PhysicsScene"
    usd_scene = UsdPhysics.Scene.Define(stage, scene_path)
    usd_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    usd_scene.CreateGravityMagnitudeAttr().Set(9.81)

    if PhysxSchema is not None:
        physx = PhysxSchema.PhysxSceneAPI.Apply(usd_scene.GetPrim())
        physx.CreateSolverTypeAttr().Set("TGS")
        physx.CreateTimeStepsPerSecondAttr().Set(int(physics_hz))
        physx.CreateEnableCCDAttr().Set(True)
        physx.CreateEnableStabilizationAttr().Set(True)
        physx.CreateEnableGPUDynamicsAttr().Set(enable_gpu_dynamics)
        physx.CreateBroadphaseTypeAttr().Set("MBP")
    else:
        prim = usd_scene.GetPrim()
        _apply_schema_token(prim, "PhysxSceneAPI")
        _attribute(prim, "physxScene:solverType", Sdf.ValueTypeNames.Token, "TGS")
        _attribute(prim, "physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Int, int(physics_hz))
        _attribute(prim, "physxScene:enableCCD", Sdf.ValueTypeNames.Bool, True)
        _attribute(prim, "physxScene:enableStabilization", Sdf.ValueTypeNames.Bool, True)
        _attribute(prim, "physxScene:enableGPUDynamics", Sdf.ValueTypeNames.Bool, bool(enable_gpu_dynamics))
        _attribute(prim, "physxScene:broadphaseType", Sdf.ValueTypeNames.Token, "MBP")


def apply_physx_articulation_settings(
    stage,
    stem_path: str,
    solver_position_iterations: int | None = None,
    solver_velocity_iterations: int | None = None,
) -> None:
    """
    Configure articulation iteration counts for TGS.

    The cantilever benchmark shows that the previous 255/32 setting can make
    low-load D6 joints timestep-dependent or fully locked at high update rates.
    """
    if solver_position_iterations is None:
        solver_position_iterations = PhysicsRuntimeConfig.SOLVER_POSITION_ITERATIONS
    if solver_velocity_iterations is None:
        solver_velocity_iterations = PhysicsRuntimeConfig.SOLVER_VELOCITY_ITERATIONS
    if not 1 <= solver_position_iterations <= 255:
        raise ValueError("solver_position_iterations must be in [1, 255]")
    if not 1 <= solver_velocity_iterations <= 255:
        raise ValueError("solver_velocity_iterations must be in [1, 255]")
    prim = stage.GetPrimAtPath(stem_path)
    if PhysxSchema is not None:
        art = PhysxSchema.PhysxArticulationAPI.Apply(prim)
        art.CreateSolverPositionIterationCountAttr().Set(solver_position_iterations)
        art.CreateSolverVelocityIterationCountAttr().Set(solver_velocity_iterations)
        art.CreateEnabledSelfCollisionsAttr().Set(False)
        art.CreateSleepThresholdAttr().Set(0.0)
    else:
        _apply_schema_token(prim, "PhysxArticulationAPI")
        _attribute(prim, "physxArticulation:solverPositionIterationCount", Sdf.ValueTypeNames.Int, solver_position_iterations)
        _attribute(prim, "physxArticulation:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, solver_velocity_iterations)
        _attribute(prim, "physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool, False)
        _attribute(prim, "physxArticulation:sleepThreshold", Sdf.ValueTypeNames.Float, 0.0)


def apply_physx_rigid_body_solver_settings(
    stage,
    body_path: str,
    solver_position_iterations: int | None = None,
    solver_velocity_iterations: int | None = None,
) -> None:
    """Configure solver precision for a rigid body outside the articulation."""
    if solver_position_iterations is None:
        solver_position_iterations = (
            PhysicsRuntimeConfig.TERMINAL_BODY_SOLVER_POSITION_ITERATIONS
        )
    if solver_velocity_iterations is None:
        solver_velocity_iterations = (
            PhysicsRuntimeConfig.TERMINAL_BODY_SOLVER_VELOCITY_ITERATIONS
        )
    if not 1 <= solver_position_iterations <= 256:
        raise ValueError("solver_position_iterations must be in [1, 256]")
    if not 1 <= solver_velocity_iterations <= 255:
        raise ValueError("solver_velocity_iterations must be in [1, 255]")

    prim = stage.GetPrimAtPath(body_path)
    if not prim or not prim.IsValid():
        raise ValueError(f"Rigid body prim does not exist: {body_path}")

    if PhysxSchema is not None:
        rigid_body = PhysxSchema.PhysxRigidBodyAPI(prim)
        if not rigid_body:
            rigid_body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rigid_body.CreateSolverPositionIterationCountAttr().Set(
            solver_position_iterations
        )
        rigid_body.CreateSolverVelocityIterationCountAttr().Set(
            solver_velocity_iterations
        )
    else:
        _apply_schema_token(prim, "PhysxRigidBodyAPI")
        _attribute(prim, "physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int, solver_position_iterations)
        _attribute(prim, "physxRigidBody:solverVelocityIterationCount", Sdf.ValueTypeNames.Int, solver_velocity_iterations)


def apply_physx_joint_armature(
    stage,
    joint_path: str,
    armature: float,
) -> None:
    """Author PhysX joint armature in kg*m^2.

    The OpenUSD wheel used by the serverless exporter does not include
    NVIDIA's PhysxSchema plugin, so keep a schema-token fallback just like the
    rigid-body helpers above.  Isaac Sim resolves the authored API and
    attribute when it opens the resulting layer.
    """

    armature = float(armature)
    if not math.isfinite(armature) or armature < 0.0:
        raise ValueError("joint armature must be finite and non-negative")
    prim = stage.GetPrimAtPath(joint_path)
    if not prim or not prim.IsValid():
        raise ValueError(f"Joint prim does not exist: {joint_path}")

    if PhysxSchema is not None:
        joint_api = PhysxSchema.PhysxJointAPI(prim)
        if not joint_api:
            joint_api = PhysxSchema.PhysxJointAPI.Apply(prim)
        joint_api.CreateArmatureAttr().Set(armature)
    else:
        _apply_schema_token(prim, "PhysxJointAPI")
        prim.CreateAttribute(
            "physxJoint:armature",
            Sdf.ValueTypeNames.Float,
            custom=False,
        ).Set(armature)
