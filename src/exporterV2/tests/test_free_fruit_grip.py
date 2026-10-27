import numpy as np
import pytest

from exporterV2.free_fruit_grip import FreeFruitGrip


@pytest.mark.parametrize("mass", [.00635, .01063])
def test_free_grip_converges_using_only_bounded_physical_forces(mass):
    grip = FreeFruitGrip()
    position, velocity = np.zeros(3), np.array([.1, 0., 0.])
    gravity, target = np.array([0., 0., -9.81]), np.array([.5, .1, .2])
    dt = 1/60
    for _ in range(300):
        force = grip.force(position, velocity, target, mass, gravity)
        acceleration = force/mass + gravity
        assert np.linalg.norm(acceleration) <= 20 + 1e-9
        velocity += acceleration*dt
        position += velocity*dt
    assert np.linalg.norm(position-target) < 1e-4
    assert np.linalg.norm(velocity) < 1e-3


def test_distant_target_and_direction_reversal_keep_acceleration_bounded():
    grip = FreeFruitGrip()
    gravity = np.array([0., 0., -9.81])
    for target in ([8., 0., 0.], [-8., 0., 0.]):
        force = grip.force(np.zeros(3), [2., 0., 0.], target, .01, gravity)
        assert np.linalg.norm(force/.01+gravity) <= 20 + 1e-9
        assert np.linalg.norm(force) < .3
