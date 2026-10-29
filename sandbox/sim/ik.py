"""Damped least-squares pose IK for the cell's six-axis arm."""

import mujoco
import numpy as np
from sandbox.sim.world import _pose


ARM_JOINTS = (
    "robot_shoulder_pan_joint",
    "robot_shoulder_lift_joint",
    "robot_elbow_joint",
    "robot_wrist_1_joint",
    "robot_wrist_2_joint",
    "robot_wrist_3_joint",
)
PINCH_SITE = "robot_gripper_pinch"


class IKError(ValueError):
    """The requested tool pose did not converge within the joint limits."""


class DampedLeastSquaresIK:
    """Solve on private MuJoCo data without moving the live physics world."""

    def __init__(self, model):
        self.model = model
        self.data = mujoco.MjData(model)
        self.joints = np.array([model.joint(name).id for name in ARM_JOINTS])
        self.qpos = model.jnt_qposadr[self.joints]
        self.dofs = model.jnt_dofadr[self.joints]
        self.site = model.site(PINCH_SITE).id
        self.limits = model.jnt_range[self.joints]

    def solve(self, position, quaternion, initial):
        """Return six joint angles for a world pose in metres and wxyz order."""
        position, quaternion = _pose(position, quaternion)
        initial = np.asarray(initial, dtype=float)
        if initial.shape != (6,) or not np.isfinite(initial).all():
            raise ValueError("initial must contain six finite joint angles")
        jac_position = np.zeros((3, self.model.nv))
        jac_rotation = np.zeros((3, self.model.nv))
        current = np.empty(4)
        inverse = np.empty(4)
        difference = np.empty(4)
        rotation_error = np.empty(3)
        guesses = (
            initial,
            np.array([-1.57, -1.57, 1.57, -1.57, -1.57, 0.0]),
            np.array([-1.57, -1.0, -1.57, -1.57, 1.57, 0.0]),
        )
        for guess in guesses:
            self.data.qpos[self.qpos] = np.clip(guess, *self.limits.T)
            for _ in range(300):
                mujoco.mj_kinematics(self.model, self.data)
                mujoco.mj_comPos(self.model, self.data)
                error_position = position - self.data.site_xpos[self.site]
                mujoco.mju_mat2Quat(current, self.data.site_xmat[self.site])
                mujoco.mju_negQuat(inverse, current)
                mujoco.mju_mulQuat(difference, quaternion, inverse)
                mujoco.mju_quat2Vel(rotation_error, difference, 1.0)
                if (np.linalg.norm(error_position) < 1e-5
                        and np.linalg.norm(rotation_error) < 1e-4):
                    return self.data.qpos[self.qpos].copy()
                mujoco.mj_jacSite(
                    self.model, self.data, jac_position, jac_rotation, self.site,
                )
                jacobian = np.vstack((jac_position[:, self.dofs],
                                      jac_rotation[:, self.dofs]))
                error = np.r_[error_position, rotation_error]
                delta = jacobian.T @ np.linalg.solve(
                    jacobian @ jacobian.T + 0.01**2 * np.eye(6), error,
                )
                delta *= min(1.0, 0.2 / max(np.max(np.abs(delta)), 1e-12))
                self.data.qpos[self.qpos] = np.clip(
                    self.data.qpos[self.qpos] + delta, *self.limits.T,
                )
        raise IKError(f"Tool pose is unreachable: {position.tolist()}")
