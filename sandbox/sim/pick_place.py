"""Scripted actuator commands, independent of ROS and physical outcome grading."""

from dataclasses import dataclass
from enum import StrEnum

import numpy as np

from sandbox.sim.ik import DampedLeastSquaresIK
from sandbox.sim.perception import Perception, rotation_matrix


class FlowState(StrEnum):
    IDLE = "IDLE"
    APPROACH = "APPROACH"
    GRASP = "GRASP"
    LIFT = "LIFT"
    TRANSPORT = "TRANSPORT"
    PLACE = "PLACE"
    RELEASE = "RELEASE"
    DONE = "DONE"


@dataclass
class _Command:
    state: FlowState
    joints: np.ndarray
    fingers: float
    duration: float


class PickPlaceFlow:
    """Advance one physics tick per step; success means all commands ran.

    State callbacks receive a FlowState on initialization and each transition.
    No contacts or object poses are inspected after the initial perception call.
    """

    def __init__(self, world, *, perception=None, on_state=None):
        self.world = world
        self.perception = perception if perception is not None else Perception(world)
        self.ik = DampedLeastSquaresIK(world.model)
        self.actuators = np.array([
            world.model.actuator(name).id for name in (
                "robot_shoulder_pan", "robot_shoulder_lift", "robot_elbow",
                "robot_wrist_1", "robot_wrist_2", "robot_wrist_3",
            )
        ])
        self.fingers = world.model.actuator("robot_gripper_fingers_actuator").id
        self.on_state = on_state
        self.state = FlowState.IDLE
        self.result = None
        self._commands = []
        if self.on_state is not None:
            self.on_state(self.state)

    def start(self, wire, crate):
        """Plan a named pick; invalid requests leave the current flow unchanged."""
        if self.state not in (FlowState.IDLE, FlowState.DONE):
            raise RuntimeError("A pick-and-place command is already running")
        if wire not in {f"wire{i}" for i in range(7)}:
            raise ValueError(f"Unknown wire: {wire}")
        if crate not in {"crate0", "crate1"}:
            raise ValueError(f"Unknown crate: {crate}")
        source, orientation = self.perception.estimate_pose(wire)
        destination, _ = self.perception.estimate_pose(crate)
        wire_axis = rotation_matrix(orientation)[:, 0]
        yaw = np.arctan2(wire_axis[1], wire_axis[0])
        down = np.array([0.0, np.cos(yaw / 2), np.sin(yaw / 2), 0.0])
        # Closed pads extend below the pinch site; grip the sides, not the top.
        source = source + [0, 0, 0.004]
        above_source = source + [0, 0, 0.13]
        above_crate = destination + [0, 0, 0.20]
        release = destination + [0, 0, 0.15]
        joints = self.world.data.qpos[self.ik.qpos].copy()
        commands = []
        for state, position, fingers, duration in (
            (FlowState.APPROACH, above_source, 0, 3.0),
            (FlowState.APPROACH, source, 0, 1.5),
            (FlowState.GRASP, source, 255, 1.0),
            (FlowState.LIFT, above_source, 255, 2.0),
            (FlowState.TRANSPORT, above_crate, 255, 4.0),
            (FlowState.PLACE, release, 255, 1.5),
            (FlowState.RELEASE, release, 0, 1.0),
            (FlowState.RELEASE, above_crate, 0, 1.5),
        ):
            joints = self.ik.solve(position, down, joints)
            commands.append(_Command(state, joints, fingers, duration))
        self._commands = commands
        self._index = 0
        self.result = None
        self._begin_command()

    def _begin_command(self):
        command = self._commands[self._index]
        self._initial = self.world.data.ctrl[self.actuators].copy()
        self._tick = 0
        self._ticks = round(command.duration / self.world.model.opt.timestep)
        if self.state != command.state:
            self.state = command.state
            if self.on_state is not None:
                self.on_state(self.state)
        self.world.data.ctrl[self.fingers] = command.fingers

    def step(self):
        """Apply one scheduled control and advance physics; idle/done are no-ops."""
        if self.state in (FlowState.IDLE, FlowState.DONE):
            return
        command = self._commands[self._index]
        self._tick += 1
        # Leave the last fifth of each command for actuator/contact settling.
        fraction = min(1.0, self._tick / (self._ticks * 0.8))
        blend = fraction**3 * (10 - 15 * fraction + 6 * fraction**2)
        self.world.data.ctrl[self.actuators] = (
            self._initial + blend * (command.joints - self._initial)
        )
        # Feed forward bias forces so gravity does not shift the grasp target.
        self.world.data.qfrc_applied[self.ik.dofs] = (
            self.world.data.qfrc_bias[self.ik.dofs]
        )
        self.world.step()
        if self._tick >= self._ticks:
            self._index += 1
            if self._index < len(self._commands):
                self._begin_command()
            else:
                self.state = FlowState.DONE
                self.result = True
                if self.on_state is not None:
                    self.on_state(self.state)

    def run(self, wire, crate):
        """Run to completion synchronously and return command success."""
        self.start(wire, crate)
        while self.state != FlowState.DONE:
            self.step()
        return self.result
