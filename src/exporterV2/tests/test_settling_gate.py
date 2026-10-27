import math
from exporterV2.settling_gate import SettlingGate


def test_gate_requires_continuous_rest_and_never_relocks():
    gate=SettlingGate(minimum_s=2.,quiet_s=1.)
    assert not gate.update(1.,1.,0.,0.)
    assert not gate.update(2.,.5,0.,0.)
    assert not gate.update(2.5,.5,.1,0.)
    assert gate.quiet_elapsed == 0.
    assert not gate.update(3.,.5,0.,0.)
    assert gate.update(3.5,.5,0.,0.)
    assert gate.armed_at == 3.5
    assert not gate.update(4.,.5,10.,10.,dragging=True)
    assert gate.armed_at == 3.5


def test_drag_or_nonfinite_motion_cannot_arm():
    for speed,angular,drag in [(0.,0.,True),(math.nan,0.,False),(0.,math.inf,False)]:
        gate=SettlingGate()
        assert not gate.update(5.,2.,speed,angular,dragging=drag)
        assert gate.armed_at is None
