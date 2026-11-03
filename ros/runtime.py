"""Public-only ROS readiness and action client, executed in pixi."""

import argparse
from collections import defaultdict
import time

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, JointState, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from benchmark_interfaces.action import PickPlace
from public_contract import ACTION_TOPICS, CAMERAS, FRAMES, TOPICS


class Readiness(Node):
    def __init__(self):
        super().__init__(
            "sandbox_readiness", parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.arrivals = defaultdict(list)
        self.edges = {}
        self.latest = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        # TODO(pawel): confirm public topic and action names.
        types = {
            "/clock": Clock, "/joint_states": JointState, "/tf": TFMessage,
            "/tf_static": TFMessage, "/flow/state": String, "/diagnostics": DiagnosticArray,
            **{f"/cameras/{camera}/{suffix}": kind for camera in CAMERAS
               for suffix, kind in (("color/image_raw", Image), ("depth/points", PointCloud2))},
        }
        self.subscriptions_ = [
            self.create_subscription(
                kind, topic, lambda message, topic=topic: self.observe(topic, message),
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                if topic == "/tf_static" else qos_profile_sensor_data,
            ) for topic, kind in types.items()
        ]
        self.action = ActionClient(self, PickPlace, "/pick_place")

    def observe(self, topic, message):
        self.arrivals[topic].append(time.monotonic())
        self.arrivals[topic] = self.arrivals[topic][-2:]
        self.latest[topic] = message
        if topic in ("/tf", "/tf_static"):
            for item in message.transforms:
                self.edges[item.child_frame_id] = item.header.frame_id

    def missing(self):
        now = time.monotonic()
        missing = []
        allowed = set(TOPICS) | set(ACTION_TOPICS) | {"/rosout", "/parameter_events"}
        for topic, _ in self.get_topic_names_and_types():
            if topic not in allowed:
                raise RuntimeError(f"Unexpected public ROS topic: {topic}")
        for topic in TOPICS:
            arrivals = self.arrivals[topic]
            if topic == "/tf_static":
                valid = bool(arrivals)
            else:
                max_age = 6 if topic.startswith("/cameras/") else 2
                valid = len(arrivals) == 2 and arrivals[-1] > arrivals[0] and now - arrivals[-1] < max_age
            if not valid:
                missing.append(topic)
        for child in FRAMES:
            frame, seen = child, set()
            while frame != "world" and frame in self.edges and frame not in seen:
                seen.add(frame)
                frame = self.edges[frame]
            if frame != "world" or not self.buffer.can_transform("world", child, rclpy.time.Time()):
                missing.append(f"TF:{child}")
        diagnostics = self.latest.get("/diagnostics")
        if not diagnostics or not diagnostics.status or any(
            item.level != DiagnosticStatus.OK for item in diagnostics.status
        ):
            missing.append("healthy diagnostics")
        if not self.action.server_is_ready():
            missing.append("/pick_place")
        return missing

    def wait(self, condition, deadline, description):
        while not condition():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{description}: {', '.join(self.missing())}")
            rclpy.spin_once(self, timeout_sec=0.02)

    def pick(self, deadline):
        self.wait(self.action.server_is_ready, deadline, "Action unavailable")
        goal = self.action.send_goal_async(PickPlace.Goal(object_id="wire0", crate_id="crate0"))
        self.wait(goal.done, deadline, "Goal acceptance timed out")
        if not goal.result().accepted:
            raise RuntimeError("Pick rejected")
        result = goal.result().get_result_async()
        self.wait(result.done, deadline, "Pick timed out")
        response = result.result()
        if response.status != GoalStatus.STATUS_SUCCEEDED or not response.result.success:
            raise RuntimeError("Pick commands failed")

    def destroy_node(self):
        self.action.destroy()
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("ready", "pick", "dry-pick"))
    parser.add_argument("--timeout", type=float, default=30)
    args = parser.parse_args()
    rclpy.init()
    node = Readiness()
    deadline = time.monotonic() + args.timeout
    try:
        if args.operation != "pick":
            node.wait(lambda: not node.missing(), deadline, "Readiness timed out")
        if args.operation != "ready":
            node.pick(deadline)
        print("ROS_CHECK_OK", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
