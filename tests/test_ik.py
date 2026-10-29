import mujoco
import numpy as np
import pytest

from sandbox.sim.ik import DampedLeastSquaresIK, IKError
from sandbox.sim.world import World


@pytest.fixture(scope="module")
def world():
    return World(seed=0)


@pytest.mark.parametrize("name", [*(f"wire{i}" for i in range(7)), "crate0", "crate1"])
def test_downward_reach_at_every_wire_and_above_both_crates(world, name):
    solver = DampedLeastSquaresIK(world.model)
    target, _ = world.body_pose(name)
    if name.startswith("crate"):
        target[2] += 0.15
    before = world.data.qpos.copy()
    joints = solver.solve(target, [0, 1, 0, 0], world.data.qpos[solver.qpos])
    data = mujoco.MjData(world.model)
    data.qpos[solver.qpos] = joints
    mujoco.mj_forward(world.model, data)
    np.testing.assert_allclose(data.site_xpos[solver.site], target, atol=1e-5, rtol=0)
    np.testing.assert_allclose(
        data.site_xmat[solver.site].reshape(3, 3),
        np.diag([1, -1, -1]), atol=1e-4, rtol=0,
    )
    assert np.all(joints >= solver.limits[:, 0])
    assert np.all(joints <= solver.limits[:, 1])
    np.testing.assert_array_equal(world.data.qpos, before)


def test_unreachable_and_invalid_poses_do_not_move_physics(world):
    solver = DampedLeastSquaresIK(world.model)
    before = world.data.qpos.copy()
    with pytest.raises(IKError, match="unreachable"):
        solver.solve([3, 0, 2], [0, 1, 0, 0], world.data.qpos[solver.qpos])
    for position, quaternion in (
        ([0, 0, np.nan], [1, 0, 0, 0]),
        ([0, 0, 1], [0, 0, 0, 0]),
    ):
        with pytest.raises(ValueError):
            solver.solve(position, quaternion, world.data.qpos[solver.qpos])
    np.testing.assert_array_equal(world.data.qpos, before)
