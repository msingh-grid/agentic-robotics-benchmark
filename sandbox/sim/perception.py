"""Synthetic camera observations reprojected through configured calibration."""

from pathlib import Path

import mujoco
import numpy as np
import yaml

from sandbox.sim.world import _pose


CAMERA_CONFIG = Path(__file__).resolve().parents[2] / "config" / "cameras.yaml"


def rotation_matrix(quaternion):
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, quaternion)
    return matrix.reshape(3, 3)


class Perception:
    """Add seeded camera-space noise, then use the possibly wrong extrinsics."""

    # TODO(pawel): confirm camera name.
    def __init__(self, world, camera="cam2", config_path=CAMERA_CONFIG):
        self.world = world
        self.camera = world.model.camera(camera).id
        with Path(config_path).open() as source:
            config = yaml.safe_load(source)["cameras"][camera]
        self.position, self.quaternion = _pose(config["position"], config["quaternion"])

    def estimate_pose(self, obj):
        """Return an estimated world position and orientation, never true extrinsics."""
        position, quaternion = self.world.body_pose(obj)
        true_rotation = self.world.data.cam_xmat[self.camera].reshape(3, 3)
        camera_position = true_rotation.T @ (
            position - self.world.data.cam_xpos[self.camera]
        )
        camera_position += self.world.rng.normal(0, 0.0005, 3)
        camera_rotation = true_rotation.T @ rotation_matrix(quaternion)
        noise = np.empty(4)
        mujoco.mju_axisAngle2Quat(
            noise, np.array([0.0, 0.0, 1.0]), self.world.rng.normal(0, 0.002),
        )
        configured_rotation = rotation_matrix(self.quaternion)
        estimate = np.empty(4)
        mujoco.mju_mat2Quat(
            estimate, (configured_rotation @ rotation_matrix(noise) @ camera_rotation).ravel(),
        )
        return self.position + configured_rotation @ camera_position, estimate
