import numpy as np
import pytest
import yaml

from sandbox.sim.perception import CAMERA_CONFIG, Perception, rotation_matrix
from sandbox.sim.world import CAMERAS, World


@pytest.mark.parametrize("camera", CAMERAS)
def test_default_calibration_matches_true_camera_and_seeded_observation(camera):
    world = World(seed=11)
    perception = Perception(world, camera)
    np.testing.assert_allclose(
        perception.position, world.data.cam_xpos[perception.camera], atol=1e-8, rtol=0,
    )
    np.testing.assert_allclose(
        rotation_matrix(perception.quaternion),
        world.data.cam_xmat[perception.camera].reshape(3, 3), atol=2e-8, rtol=0,
    )
    observed = perception.estimate_pose("wire2")
    true_position, true_quaternion = world.body_pose("wire2")
    assert 0 < np.linalg.norm(observed[0] - true_position) < 0.003
    np.testing.assert_allclose(
        rotation_matrix(observed[1]), rotation_matrix(true_quaternion), atol=0.01, rtol=0,
    )
    world.seed(11)
    repeated = perception.estimate_pose("wire2")
    for first, second in zip(observed, repeated):
        np.testing.assert_array_equal(first, second)
    world.seed(12)
    assert not np.array_equal(perception.estimate_pose("wire2")[0], observed[0])


def test_configured_translation_and_rotation_change_the_estimate_not_the_truth(tmp_path):
    world = World(seed=42)
    original = Perception(world).estimate_pose("wire0")
    truth = world.body_pose("wire0")
    config = yaml.safe_load(CAMERA_CONFIG.read_text())
    camera = config["cameras"]["cam2"]
    camera["position"][0] += 0.08
    camera["quaternion"] = [np.cos(0.1).item(), 0, 0, np.sin(0.1).item()]
    path = tmp_path / "cameras.yaml"
    path.write_text(yaml.safe_dump(config))
    world.seed(42)
    perception = Perception(world, config_path=path)
    translated, rotated = perception.estimate_pose("wire0")
    rotation = rotation_matrix(perception.quaternion)
    np.testing.assert_allclose(
        translated,
        perception.position + rotation @ (original[0] - world.data.cam_xpos[2]),
        atol=1e-12, rtol=0,
    )
    np.testing.assert_allclose(
        rotation_matrix(rotated), rotation @ rotation_matrix(original[1]),
        atol=1e-12, rtol=0,
    )
    for before, after in zip(truth, world.body_pose("wire0")):
        np.testing.assert_array_equal(before, after)
