"""Factorial controls must isolate coefficients and preserve other USD state."""
import pytest
from pxr import Usd, UsdGeom, UsdPhysics, Sdf

from exporterV2.drive_experiments import apply_drive_variant, drive_records, zero_velocity_iterations


def fixture(k, d):
    stage = Usd.Stage.CreateInMemory()
    for name, role in [('stem', 'internode'), ('rachis0', 'truss_rachis'),
                       ('rachis1', 'truss_rachis'), ('pedicel', 'pedicel')]:
        prim = UsdGeom.Xform.Define(stage, '/'+name).GetPrim()
        prim.CreateAttribute('autotom:role', Sdf.ValueTypeNames.String).Set(role)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(.001)
    for parent, child in [('stem', 'rachis0'), ('rachis0', 'rachis1'), ('rachis1', 'pedicel')]:
        joint = UsdPhysics.Joint.Define(stage, '/'+child+'/Joint')
        joint.CreateBody0Rel().SetTargets(['/'+parent])
        joint.CreateBody1Rel().SetTargets(['/'+child])
        for axis in ('rotX', 'rotY'):
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
            drive.CreateStiffnessAttr().Set(k)
            drive.CreateDampingAttr().Set(d)
            drive.CreateTypeAttr().Set('force')
            drive.CreateTargetPositionAttr().Set(0)
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            limit.CreateLowAttr().Set(-25)
            limit.CreateHighAttr().Set(25)
    return stage


def snapshot(stage):
    return {str(p.GetPath()): str(p.Get()) if isinstance(p, Usd.Attribute) else str(p.GetTargets())
            for prim in stage.Traverse() for p in prim.GetProperties()}


@pytest.mark.parametrize('variant,k,d', [('baseline',100,2), ('main-stiffness',3,2),
                                        ('main-damping',100,5), ('main-drives',3,5)])
def test_only_named_coefficients_change(variant, k, d):
    stage, reference = fixture(100,2), fixture(3,5)
    before, ref_before = snapshot(stage), snapshot(reference)
    changes = apply_drive_variant(stage, reference, variant)
    after = snapshot(stage)
    assert snapshot(reference) == ref_before
    actual = {key for key in before if before[key] != after[key]}
    expected = {c['joint']+'.'+c['attribute'] for c in changes}
    assert actual == expected
    assert all(row['stiffness']==k and row['damping']==d for row in drive_records(stage))


def test_missing_role_rejected_before_any_mutation():
    stage, reference = fixture(100,2), fixture(3,5)
    reference.RemovePrim('/pedicel')
    before = snapshot(stage)
    with pytest.raises(KeyError):
        apply_drive_variant(stage, reference, 'main-drives')
    assert snapshot(stage) == before


def test_zero_velocity_preserves_positions_drives_and_mass():
    stage = fixture(100, 2)
    art = stage.GetPrimAtPath('/stem')
    fruit = UsdGeom.Xform.Define(stage, '/fruit').GetPrim()
    fruit.CreateAttribute('autotom:role', Sdf.ValueTypeNames.String).Set('fruit')
    for prim, schema, velocity in [(art, 'Articulation', 4), (fruit, 'RigidBody', 1)]:
        for kind, count in [('Position', 32), ('Velocity', velocity)]:
            prim.CreateAttribute(f'physx{schema}:solver{kind}IterationCount', Sdf.ValueTypeNames.Int).Set(count)
    before = snapshot(stage)
    changes = zero_velocity_iterations(stage)
    after = snapshot(stage)
    assert {k for k in before if before[k] != after[k]} == {
        c['prim']+'.'+c['attribute'] for c in changes}
    assert len(changes) == 2
    assert all(c['modified'] == 0 for c in changes)
