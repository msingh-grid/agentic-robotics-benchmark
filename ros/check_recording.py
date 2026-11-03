"""Deserialize a public bag and prove it replays through ROS."""

import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

from public_contract import ACTION_TOPICS, CAMERAS, FRAMES, TOPICS


def audit(path, missing):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()}
    assert set(types) <= set(TOPICS) | set(ACTION_TOPICS), types
    counts = Counter()
    frames, flow = set(), set()
    while reader.has_next():
        topic, data, _ = reader.read_next()
        message = deserialize_message(data, types[topic])
        counts[topic] += 1
        if topic in ("/tf", "/tf_static"):
            for transform in message.transforms:
                assert transform.child_frame_id in FRAMES
                assert transform.header.frame_id in FRAMES | {"world"}
                frames.add(transform.child_frame_id)
        elif topic == "/joint_states":
            assert message.name and all(name.startswith("robot_") for name in message.name)
        elif topic == "/flow/state":
            flow.add(message.data)
            assert message.data in {
                "IDLE", "APPROACH", "GRASP", "LIFT", "TRANSPORT", "PLACE", "RELEASE", "DONE",
            }
        elif topic == "/diagnostics":
            assert all(item.message == "Simulation running" and not item.values
                       for item in message.status)
        elif topic.endswith("/feedback"):
            assert message.feedback.state in {
                "FORWARDING", "APPROACH", "GRASP", "LIFT", "TRANSPORT",
                "PLACE", "RELEASE", "DONE",
            }
        elif topic.endswith("image_raw"):
            assert message.encoding == "rgb8" and message.header.frame_id in {
                f"{camera}_optical" for camera in CAMERAS
            }
        elif topic.endswith("points"):
            assert [field.name for field in message.fields] == ["x", "y", "z"]
    assert all(counts[topic] > 0 for topic in TOPICS if topic != missing), counts
    if missing:
        assert counts[missing] == 0, counts
    assert frames == FRAMES and "DONE" in flow, (frames, flow)
    return types, counts


def replay(path, types, missing):
    node = Node("recording_replay_check")
    counts = Counter()
    flows = set()

    def receive(topic, message):
        counts[topic] += 1
        if topic == "/flow/state":
            flows.add(message.data)

    subscriptions = [
        node.create_subscription(
            types[topic], topic, lambda message, topic=topic: receive(topic, message),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            if topic == "/tf_static" else qos_profile_sensor_data,
        ) for topic in TOPICS
    ]
    process = subprocess.Popen([
        "ros2", "bag", "play", str(path), "--rate", "2", "--delay", "1",
        "--disable-keyboard-controls",
    ])
    try:
        deadline = time.monotonic() + 35
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if process.poll() is not None and "DONE" in flows and all(
                counts[topic] for topic in TOPICS if topic != missing
            ):
                break
        assert process.wait(timeout=3) == 0
        assert "DONE" in flows, flows
        assert all(counts[topic] for topic in TOPICS if topic != missing), counts
        assert not any(word in name for name, _ in node.get_topic_names_and_types()
                       for word in ("truth", "fault", "verification", "scenario"))
        print("REPLAY_OK " + json.dumps(counts), flush=True)
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5)
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--missing-topic")
    args = parser.parse_args()
    types, counts = audit(args.bag, args.missing_topic)
    print("BAG_AUDIT_OK " + json.dumps(counts), flush=True)
    rclpy.init()
    try:
        replay(args.bag, types, args.missing_topic)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
