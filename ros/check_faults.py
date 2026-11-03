"""Public-only ROS assertions for seeded fault integration tests."""

import argparse
import json
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, DurabilityPolicy, QoSProfile
from action_msgs.msg import GoalStatus
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import yaml

from benchmark_interfaces.action import PickPlace


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def check(args):
    node = Node("delivery_check")
    clock = 0
    arrivals = []
    camera_positions = {}

    def on_clock(message):
        nonlocal clock
        clock = ns(message.clock)

    def on_joint(message):
        arrivals.append((time.monotonic(), ns(message.header.stamp), clock))

    def on_tf(message):
        for transform in message.transforms:
            position = transform.transform.translation
            camera_positions[transform.child_frame_id] = [position.x, position.y, position.z]

    # TODO(pawel): confirm topic and action names.
    subscriptions = [
        node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data),
        node.create_subscription(JointState, "/joint_states", on_joint, qos_profile_sensor_data),
        node.create_subscription(TFMessage, "/tf_static", on_tf, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )),
    ]
    action = ActionClient(node, PickPlace, "/pick_place")

    def wait(condition, timeout=45):
        deadline = time.monotonic() + timeout
        while not condition():
            assert time.monotonic() < deadline, "ROS fault observer timed out"
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        wait(lambda: len(arrivals) >= 5 and "cam2" in camera_positions)
        assert action.wait_for_server(timeout_sec=10)
        topics = dict(node.get_topic_names_and_types())
        assert not any(
            word in topic for topic in topics
            for word in ("truth", "fault", "scenario", "verification")
        ), topics
        configured = yaml.safe_load(args.camera_config.read_text())["cameras"]
        for camera, pose in configured.items():
            assert camera_positions[camera] == pose["position"]
        arrivals.clear()
        goal = action.send_goal_async(PickPlace.Goal(object_id="wire0", crate_id="crate0"))
        wait(goal.done)
        assert goal.result().accepted
        result = goal.result().get_result_async()
        wait(result.done)
        response = result.result()
        assert response.status == GoalStatus.STATUS_SUCCEEDED
        assert response.result.success and response.result.message == "Commands completed"
        assert len(arrivals) > 20
        duration = arrivals[-1][0] - arrivals[0][0]
        hz = (len(arrivals) - 1) / duration
        lag_ms = statistics.median((clock - source) / 1e6 for _, source, clock in arrivals)
        gaps = [second[1] - first[1] for first, second in zip(arrivals, arrivals[1:])]
        if args.mode == "topic_delay":
            assert 180 <= lag_ms <= 400, (hz, lag_ms)
            assert hz >= 35, hz
        elif args.mode == "topic_drop":
            assert 7 <= hz <= 20, hz
            assert max(gaps) >= 60_000_000, max(gaps)
            assert lag_ms < 60, lag_ms
        else:
            assert hz >= 35 and lag_ms < 60, (hz, lag_ms)
        args.output.write_text(json.dumps({
            "hz": hz, "median_lag_ms": lag_ms, "messages": len(arrivals),
            "max_stamp_gap_ns": max(gaps), "success": response.result.success,
        }))
        print("FAULT_ROS_OK " + args.output.read_text(), flush=True)
    finally:
        action.destroy()
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True)
    parser.add_argument("--camera-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    try:
        check(args)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
