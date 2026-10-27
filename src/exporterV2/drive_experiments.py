"""Independent stiffness/damping controls on private native-comparison stages."""
from pxr import UsdPhysics

from exporterV2.fruit_experiments import value

VARIANTS = {
    'baseline': (),
    'main-stiffness': ('stiffness',),
    'main-damping': ('damping',),
    'main-drives': ('stiffness', 'damping'),
}


def zero_velocity_iterations(stage):
    """Disable the TGS velocity stage for the articulation and external fruits."""
    changes = []
    for prim in stage.Traverse():
        names = ['physxArticulation:solverVelocityIterationCount']
        if value(prim, 'autotom:role') == 'fruit':
            names.append('physxRigidBody:solverVelocityIterationCount')
        for name in names:
            attr = prim.GetAttribute(name)
            if attr and attr.HasAuthoredValueOpinion():
                old = attr.Get()
                attr.Set(0)
                changes.append(dict(prim=str(prim.GetPath()), attribute=name,
                                    original=old, modified=attr.Get()))
    return changes


def drive_records(stage):
    """Match mechanical joint roles, not names or branch segment counts."""
    rows = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        a, b = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        if not a or not b:
            continue
        parent, child = stage.GetPrimAtPath(a[0]), stage.GetPrimAtPath(b[0])
        child_role = value(child, 'autotom:role')
        if child_role == 'truss_rachis':
            role = 'rachis_internal' if value(parent, 'autotom:role') == child_role else 'rachis_attachment'
        elif child_role == 'pedicel':
            role = 'pedicel_attachment'
        else:
            continue
        for axis in ('rotX', 'rotY', 'rotZ'):
            drive = UsdPhysics.DriveAPI(prim, axis)
            if drive:
                rows.append(dict(joint=str(prim.GetPath()), parent=str(a[0]), child=str(b[0]),
                                 role=role, axis=axis, stiffness=drive.GetStiffnessAttr().Get(),
                                 damping=drive.GetDampingAttr().Get(), type=drive.GetTypeAttr().Get(),
                                 target_position=drive.GetTargetPositionAttr().Get(),
                                 target_velocity=drive.GetTargetVelocityAttr().Get()))
    return rows


def apply_drive_variant(stage, reference, variant):
    """Change only requested drive attributes; never automatically couple K and D."""
    fields = VARIANTS[variant]
    if not fields:
        return []
    template = {}
    for row in drive_records(reference):
        key = row['role'], row['axis']
        if key in template and any(template[key][f] != row[f] for f in fields):
            raise ValueError(f'Nonuniform reference coefficients for {key}')
        template.setdefault(key, row)
    changes = []
    # Validate all mappings before mutating any property.
    rows = drive_records(stage)
    if not rows:
        raise ValueError('No truss drives to compare')
    for row in rows:
        ref = template[row['role'], row['axis']]
        if row['type'] != ref['type']:
            raise ValueError('Reference drive type differs')
    for row in rows:
        ref = template[row['role'], row['axis']]
        for field in fields:
            name = f"drive:{row['axis']}:physics:{field}"
            attr = stage.GetPrimAtPath(row['joint']).GetAttribute(name)
            old = attr.Get()
            attr.Set(ref[field])
            changes.append(dict(joint=row['joint'], role=row['role'], axis=row['axis'],
                                attribute=name, original=old, modified=attr.Get(),
                                reference_joint=ref['joint']))
    return changes
