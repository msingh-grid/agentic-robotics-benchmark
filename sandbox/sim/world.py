"""The local, headless pick-and-place physics world."""

from pathlib import Path

import mujoco
import numpy as np

from sandbox.sim.render import render_scene

SCENE = Path(__file__).resolve().parents[1] / "scenes" / "cell.xml"
# TODO(pawel): confirm camera names.
CAMERAS = ("cam0", "cam1", "cam2", "cam3")


def _pose(position, quaternion):
    position = np.asarray(position, dtype=float)
    quaternion = np.asarray(quaternion, dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise ValueError("position must contain three finite coordinates")
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("quaternion must contain four finite wxyz components")
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("quaternion must have a finite, nonzero norm")
    return position, quaternion / norm


class World:
    """Settled MuJoCo cell; poses use metres and wxyz quaternions.

    Initialization runs two seconds of idle settling, then starts the clock at zero.
    """

    def __init__(self, seed=0):
        self.model = mujoco.MjModel.from_xml_path(str(SCENE))
        self.data = mujoco.MjData(self.model)
        self.seed(seed)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("robot_home").id)
        mujoco.mj_forward(self.model, self.data)
        for _ in range(1000):
            self.step()
        self.data.time = 0

    def seed(self, n):
        """Reset the world's private random generator, without changing physics."""
        if not isinstance(n, (int, np.integer)) or isinstance(n, bool) or n < 0:
            raise ValueError("seed must be an explicit nonnegative integer")
        self.rng = np.random.default_rng(n)

    def step(self):
        """Advance one 2 ms physics step and refresh derived world poses."""
        mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def render(self, cam):
        """Return 640×480 RGB uint8 and optical-axis depth float32 in metres."""
        return render_scene(self.model, self.data, cam)

    def body_pose(self, name):
        """Return independent copies of the body's world position and quaternion."""
        body = self.data.body(name)
        return body.xpos.copy(), body.xquat.copy()

    def set_body_pose(self, name, position, quaternion):
        """Teleport a free body or a world-fixed body; clear free-body velocity."""
        position, quaternion = _pose(position, quaternion)
        body = self.model.body(name)
        joint = body.jntadr[0]
        if body.jntnum[0] == 1 and self.model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE:
            address = self.model.jnt_qposadr[joint]
            self.data.qpos[address:address + 7] = np.r_[position, quaternion]
            velocity = self.model.jnt_dofadr[joint]
            self.data.qvel[velocity:velocity + 6] = 0
        elif body.id != 0 and body.parentid[0] == 0 and body.jntnum[0] == 0:
            body.pos[:] = position
            body.quat[:] = quaternion
            state_type = mujoco.mjtState.mjSTATE_INTEGRATION
            state = np.empty(mujoco.mj_stateSize(self.model, state_type))
            mujoco.mj_getState(self.model, self.data, state, state_type)
            mujoco.mj_setConst(self.model, self.data)
            mujoco.mj_setState(self.model, self.data, state, state_type)
        else:
            raise ValueError(f"{name} is not a free or world-fixed body")
        mujoco.mj_forward(self.model, self.data)

    def set_camera_pose(self, name, position, quaternion):
        """Set a fixed camera's world pose; its optical axis is local -Z."""
        position, quaternion = _pose(position, quaternion)
        camera = self.model.camera(name)
        if camera.bodyid[0] != 0:
            raise ValueError(f"{name} is not a world-fixed camera")
        camera.pos[:] = position
        camera.quat[:] = quaternion
        mujoco.mj_forward(self.model, self.data)
