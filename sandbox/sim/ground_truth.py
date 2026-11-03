"""Grader-only physical measurements and simulation-time events."""

import os
from pathlib import Path

import mujoco
import numpy as np

from sandbox.sim.perception import rotation_matrix
from sandbox.sim.world import CAMERAS

OBJECTS = tuple(f"wire{i}" for i in range(7)) + ("crate0", "crate1")


def bind_private(socket, directory, name):
    directory = Path(directory).resolve()
    directory.mkdir(mode=0o700, parents=True, exist_ok=True)
    if directory.stat().st_mode & 0o077:
        raise ValueError("Private directory must have mode 0700")
    path = directory / name
    # Bind under a restrictive umask: chmod alone leaves a connection race.
    previous = os.umask(0o077)
    try:
        socket.bind(f"ipc://{path}")
    finally:
        os.umask(previous)
    path.chmod(0o600)


def inside_crate(world, wire, crate):
    position, quaternion = world.body_pose(wire)
    center, crate_quaternion = world.body_pose(crate)
    axis = rotation_matrix(quaternion)[:, 0] * 0.09
    local = (np.array([position - axis, position + axis]) - center) @ rotation_matrix(
        crate_quaternion,
    )
    return bool(
        np.all(np.abs(local[:, 0]) + 0.009 < 0.16)
        and np.all(np.abs(local[:, 1]) + 0.009 < 0.11)
        and np.all(local[:, 2] >= 0.02)
        and np.all(local[:, 2] + 0.009 < 0.12)
    )


class GroundTruth:
    def __init__(self, world):
        self.world = world
        self.events = []
        self.request = None
        self.held = set()
        self.grasped = set()
        self.contact_lost = {}
        self.completed = False

    def begin(self, wire, crate):
        self.request = (wire, crate)
        self.event_start = len(self.events)
        self.held = set()
        self.grasped = set()
        self.contact_lost = {}
        self.completed = False
        self.initial_heights = {
            name: self.world.body_pose(name)[0][2] for name in OBJECTS[:7]
        }

    def contacts(self):
        contacts = []
        model = self.world.model
        for index, contact in enumerate(self.world.data.contact):
            bodies = [model.body(model.geom(geom).bodyid[0]).name
                      for geom in (contact.geom1, contact.geom2)]
            for wire, gripper in (bodies, bodies[::-1]):
                if wire in OBJECTS[:7] and gripper.startswith("robot_") and (
                    "finger" in gripper or "pad" in gripper
                ):
                    force = np.empty(6)
                    mujoco.mj_contactForce(
                        model, self.world.data, index, force,
                    )
                    if force[0] > 0:
                        contacts.append({
                            "object": wire, "gripper": gripper,
                            "normal_force_n": float(force[0]),
                        })
        return contacts

    def event(self, kind, **details):
        self.events.append({
            "kind": kind, "time_ns": round(self.world.data.time * 1e9), **details,
        })

    def observe(self, flow):
        if self.request is None or self.completed:
            return
        contacting = {item["object"] for item in self.contacts()}
        for wire in sorted(contacting - self.grasped):
            if self.world.body_pose(wire)[0][2] > self.initial_heights[wire] + 0.025:
                self.event("grasp", object=wire)
                self.grasped.add(wire)
                self.held.add(wire)
        for wire in sorted(self.held - contacting):
            lost = self.contact_lost.setdefault(wire, self.world.data.time)
            if self.world.data.time - lost >= 0.02:
                self.event("release", object=wire)
                if flow.state not in ("RELEASE", "DONE"):
                    self.event("drop", object=wire)
                self.held.remove(wire)
        for wire in contacting:
            self.contact_lost.pop(wire, None)
        if flow.state == "DONE":
            wire, crate = self.request
            self.event(
                "placement_result", object=wire, crate=crate,
                inside=inside_crate(self.world, wire, crate),
                placed=[name for name in OBJECTS[:7] if inside_crate(self.world, name, crate)],
            )
            self.completed = True

    def snapshot(self):
        objects = {}
        for name in OBJECTS:
            position, quaternion = self.world.body_pose(name)
            objects[name] = {"position": position.tolist(), "quaternion": quaternion.tolist()}
        cameras = {}
        for name in CAMERAS:
            camera = self.world.model.camera(name)
            cameras[name] = {
                "position": self.world.data.cam_xpos[camera.id].tolist(),
                "quaternion": camera.quat.tolist(),
            }
        return {
            "time_ns": round(self.world.data.time * 1e9),
            "objects": objects, "cameras": cameras,
            "gripper_contacts": self.contacts(), "events": list(self.events),
        }
