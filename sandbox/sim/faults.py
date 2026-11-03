"""Private, seeded fault injection; verification uses measured effects."""

import math

import mujoco
import numpy as np
import yaml

from sandbox.sim.ground_truth import inside_crate
from sandbox.sim.perception import rotation_matrix
from sandbox.sim.pick_place import FlowState
from sandbox.sim.world import CAMERAS

# TODO(pawel): confirm public topic names.
TOPICS = {"/clock", "/joint_states", "/tf", "/flow/state", "/diagnostics"} | {
    f"/cameras/{camera}/{suffix}"
    for camera in CAMERAS for suffix in ("color/image_raw", "depth/points")
}


def vector(value, name):
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.isfinite(array).all():
        raise ValueError(f"{name} must contain three finite numbers")
    return array


def load_scenario(path):
    with path.open() as source:
        return yaml.safe_load(source)


def validate(scenario):
    if not isinstance(scenario, dict) or set(scenario) != {"faults"}:
        raise ValueError("A scenario must contain only a faults list")
    faults = scenario["faults"]
    if not isinstance(faults, list):
        raise ValueError("faults must be a list; use [] for a healthy control")
    schemas = {
        "wrong_object": set(),
        "drop_in_transit": {"at_state", "t"},
        "place_outside_crate": {"offset_m"},
        "camera_offset": {"cam", "xyz_m", "rpy_deg"},
        "topic_delay": {"topic", "ms"},
        "topic_drop": {"topic", "rate"},
    }
    seen = set()
    for fault in faults:
        if not isinstance(fault, dict):
            raise ValueError("Each fault must be a mapping")
        kind = fault.get("type")
        if not isinstance(kind, str) or kind not in schemas or set(fault) != schemas[kind] | {"type"}:
            raise ValueError("Unknown fault or invalid fault parameters")
        if kind in seen:
            raise ValueError("Duplicate fault type")
        seen.add(kind)
        if kind == "drop_in_transit":
            if fault["at_state"] not in {FlowState.LIFT, FlowState.TRANSPORT, FlowState.PLACE}:
                raise ValueError("Drop state must be LIFT, TRANSPORT or PLACE")
            if not isinstance(fault["t"], (int, float)) or not math.isfinite(fault["t"]) or fault["t"] < 0:
                raise ValueError("Drop t must be finite nonnegative seconds after state entry")
        if kind == "place_outside_crate":
            vector(fault["offset_m"], "offset_m")
        if kind == "camera_offset":
            if fault["cam"] not in CAMERAS:
                raise ValueError("Unknown camera")
            vector(fault["xyz_m"], "xyz_m")
            vector(fault["rpy_deg"], "rpy_deg")
        if kind.startswith("topic_"):
            if fault["topic"] not in TOPICS:
                raise ValueError("Unknown or unsupported topic")
            key = "ms" if kind == "topic_delay" else "rate"
            value = fault[key]
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError("Topic impairment must be finite and nonnegative")
            if key == "rate" and value > 1:
                raise ValueError("Drop rate must be between zero and one")
    return faults


class Faults:
    """Apply metre translations and extrinsic world-axis roll/pitch/yaw rotations."""
    def __init__(self, world, perception, scenario, seed):
        self.definitions = validate(scenario)
        self.world = world
        self.perception = perception
        self.rng = np.random.default_rng(seed)
        self.seed = seed
        self.fired = {}
        self.verified = set()
        self.bridge_audit = {}
        self.episode = 0
        self.state = None
        self.state_time = 0
        self.drop_open = False
        self.request = None
        self.camera_before = {}
        self.camera_expected = {}
        for fault in self.definitions:
            if fault["type"] == "camera_offset":
                name = fault["cam"]
                camera = world.model.camera(name)
                position = camera.pos.copy()
                quaternion = camera.quat.copy()
                self.camera_before[name] = (position, quaternion)
                delta = np.empty(4)
                mujoco.mju_euler2Quat(delta, np.radians(fault["rpy_deg"]), "XYZ")
                rotated = np.empty(4)
                mujoco.mju_mulQuat(rotated, delta, quaternion)
                expected_position = position + vector(fault["xyz_m"], "xyz_m")
                self.camera_expected[name] = (expected_position.copy(), rotated.copy())
                world.set_camera_pose(name, expected_position, rotated)

    def begin(self, wire, crate):
        self.request = (wire, crate)
        self.episode += 1
        self.verified.clear()
        self.bridge_audit = {}
        self.drop_open = False
        self.fired = {}
        self.state = None
        self.actual_wire = wire
        if any(f["type"] == "wrong_object" for f in self.definitions):
            self.actual_wire = self.rng.choice([f"wire{i}" for i in range(7) if f"wire{i}" != wire]).item()

    def estimate_pose(self, obj):
        actual = self.actual_wire if self.request and obj == self.request[0] else obj
        position, quaternion = self.perception.estimate_pose(actual)
        for fault in self.definitions:
            if fault["type"] == "place_outside_crate" and self.request and obj == self.request[1]:
                position += vector(fault["offset_m"], "offset_m")
        return position, quaternion

    def before_step(self, flow):
        if self.state != flow.state:
            self.state = flow.state
            self.state_time = self.world.data.time
        for index, fault in enumerate(self.definitions):
            if fault["type"] == "drop_in_transit" and flow.state == fault["at_state"]:
                if self.world.data.time - self.state_time + 1e-9 >= fault["t"]:
                    self.drop_open = True
                    self.fired.setdefault(index, round(self.world.data.time * 1e9))
        if self.drop_open:
            self.world.data.ctrl[flow.fingers] = 0

    def bridge_plan(self):
        return {
            "seed": self.seed,
            "episode": self.episode,
            "faults": [f for f in self.definitions if f["type"].startswith("topic_")],
        }

    def verification(self, truth):
        results = []
        events = truth.events[truth.event_start:] if truth.request else []
        for index, fault in enumerate(self.definitions):
            kind = fault["type"]
            verified = False
            if kind == "wrong_object":
                verified = any(e["kind"] == "grasp" and e["object"] == self.actual_wire
                               for e in events) if self.request else False
            elif kind == "drop_in_transit":
                verified = index in self.fired and any(
                    e["kind"] == "drop" and e["time_ns"] >= self.fired[index]
                    for e in events
                )
            elif kind == "place_outside_crate" and self.request and truth.completed:
                wire, crate = self.request
                offset = vector(fault["offset_m"], "offset_m")
                actual = self.world.body_pose(wire)[0] - self.world.body_pose(crate)[0]
                verified = bool(
                    wire in truth.grasped and not inside_crate(self.world, wire, crate)
                    and np.linalg.norm(actual[:2] - offset[:2]) < 0.10
                )
            elif kind == "camera_offset":
                position, quaternion = self.camera_before[fault["cam"]]
                expected_position, expected_quaternion = self.camera_expected[fault["cam"]]
                camera = self.world.model.camera(fault["cam"])
                verified = bool(
                    (np.linalg.norm(expected_position - position) > 1e-8
                     or abs(float(expected_quaternion @ quaternion)) < 1 - 1e-10)
                    and np.allclose(
                        self.world.data.cam_xpos[camera.id], expected_position, atol=1e-9, rtol=0,
                    )
                    and np.allclose(
                        self.world.data.cam_xmat[camera.id].reshape(3, 3),
                        rotation_matrix(expected_quaternion), atol=1e-9, rtol=0,
                    )
                )
            elif kind.startswith("topic_"):
                audit = self.bridge_audit.get(fault["topic"], {})
                if audit.get("overflow", 0):
                    verified = False
                elif kind == "topic_drop":
                    verified = audit.get("dropped", 0) > 0 and audit.get("received", 0) > 0
                else:
                    verified = (
                        fault["ms"] > 0 and audit.get("delivered", 0) > 0
                        and audit.get("min_delay_ns", 0) >= round(fault["ms"] * 1e6)
                    )
            if verified:
                self.verified.add(index)
            overflow = kind.startswith("topic_") and self.bridge_audit.get(
                fault["topic"], {},
            ).get("overflow", 0)
            results.append({"type": kind, "verified": index in self.verified and not bool(overflow)})
        complete = truth.completed
        return {
            "status": ("valid" if all(r["verified"] for r in results) else "invalid")
            if complete else "pending",
            "faults": results, "bridge": self.bridge_audit,
        }
