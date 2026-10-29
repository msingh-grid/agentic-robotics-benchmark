import mujoco
import numpy as np
import pytest

from sandbox.sim.render import render_scene
from sandbox.sim.world import CAMERAS, World


@pytest.fixture
def world():
    return World(seed=42)


def test_objects_visible_in_camera_segmentation(world):
    visible_bodies = set()
    for camera in CAMERAS:
        segmentation = render_scene(world.model, world.data, camera, segmentation=True)
        geom_ids = segmentation[..., 0][segmentation[..., 1] == mujoco.mjtObj.mjOBJ_GEOM]
        visible_bodies.update(world.model.geom_bodyid[np.unique(geom_ids)].tolist())
    for name in [*(f"wire{i}" for i in range(7)), "crate0", "crate1"]:
        assert world.model.body(name).id in visible_bodies, f"{name} is not visible"


def test_idle_cell_stable_for_ten_seconds(world):
    initial_position = world.data.xpos.copy()
    initial_quaternion = world.data.xquat.copy()
    for _ in range(5000):
        world.step()
        assert np.isfinite(world.data.qpos).all()
        np.testing.assert_allclose(world.data.xpos, initial_position, atol=1e-4, rtol=0)
        np.testing.assert_allclose(world.data.xquat, initial_quaternion, atol=1e-4, rtol=0)
        assert np.max(np.abs(world.data.qvel)) < 1e-3
    assert world.data.time == pytest.approx(10)
    assert not any(warning.number for warning in world.data.warning)


def test_free_and_fixed_body_pose_updates(world):
    world.data.qvel[:] = 0.01
    position = [-0.2, -0.2, 1.0]
    quaternion = [2, 0, 0, 2]
    world.set_body_pose("wire0", position, quaternion)
    actual_position, actual_quaternion = world.body_pose("wire0")
    np.testing.assert_allclose(actual_position, position)
    np.testing.assert_allclose(actual_quaternion, np.array(quaternion) / np.sqrt(8))
    body = world.model.body("wire0")
    dof = world.model.jnt_dofadr[body.jntadr[0]]
    np.testing.assert_array_equal(world.data.qvel[dof:dof + 6], 0)
    actual_position[:] = 99
    np.testing.assert_allclose(world.body_pose("wire0")[0], position)
    world.set_body_pose("crate0", [0.4, -0.3, 0.76], [1, 0, 0, 0])
    np.testing.assert_allclose(world.body_pose("crate0")[0], [0.4, -0.3, 0.76])
    np.testing.assert_allclose(world.body_pose("wire0")[0], position)
    with pytest.raises(ValueError, match="not a free or world-fixed"):
        world.set_body_pose("robot_wrist_3_link", position, quaternion)


@pytest.mark.parametrize("position,quaternion", [
    ([0, 0], [1, 0, 0, 0]),
    ([0, 0, np.nan], [1, 0, 0, 0]),
    ([0, 0, 1], [0, 0, 0, 0]),
    ([0, 0, 1], [1, np.inf, 0, 0]),
])
def test_invalid_pose_does_not_mutate_world(world, position, quaternion):
    before = world.data.qpos.copy()
    with pytest.raises(ValueError):
        world.set_body_pose("wire0", position, quaternion)
    np.testing.assert_array_equal(world.data.qpos, before)


def test_camera_rgb_depth_and_pose(world):
    for camera in CAMERAS:
        rgb, depth = world.render(camera)
        assert rgb.shape == (480, 640, 3) and rgb.dtype == np.uint8
        assert depth.shape == (480, 640) and depth.dtype == np.float32
        assert np.isfinite(depth).all() and (depth > 0).all()
        assert np.ptp(rgb) > 100
    world.set_camera_pose("cam2", [0, -0.1, 2.9], [2, 0, 0, 0])
    np.testing.assert_allclose(world.data.cam_xpos[2], [0, -0.1, 2.9])
    np.testing.assert_allclose(world.data.cam_xmat[2].reshape(3, 3), np.eye(3))
    rgb, depth = world.render("cam2")
    # The optical axis hits the table top at z=0.76 m, not a slant-range surface.
    assert depth[240, 320] == pytest.approx(2.9 - 0.76, abs=0.002)
    with pytest.raises(KeyError):
        world.render("missing_camera")


def test_seeded_physics_repeats(world):
    other = World(seed=42)
    for _ in range(25):
        world.step()
        other.step()
    np.testing.assert_array_equal(world.data.qpos, other.data.qpos)
    values = world.rng.normal(size=8)
    world.seed(42)
    np.testing.assert_array_equal(values, world.rng.normal(size=8))
    with pytest.raises(ValueError, match="explicit nonnegative integer"):
        world.seed(None)
