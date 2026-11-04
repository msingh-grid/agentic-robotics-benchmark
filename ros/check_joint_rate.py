"""Measure joint-state arrival rates independently of camera deserialization."""

import argparse
from collections import Counter
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState, PointCloud2


def check(output, ready, camera_ready, windows):
    node = Node("joint_rate_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
    count = 0
    previous_stamp = None
    cameras = Counter()

    def record(message):
        nonlocal count, previous_stamp
        source = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        assert previous_stamp is None or source > previous_stamp, "Repeated or regressing physics state"
        previous_stamp = source
        count += 1

    node.create_subscription(
        JointState, "/joint_states", record,
        QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    # TODO(pawel): confirm public camera topic names.
    camera_topics = [
        (f"/cameras/cam{index}/{suffix}", kind)
        for index in range(4)
        for suffix, kind in (("color/image_raw", Image), ("depth/points", PointCloud2))
    ]
    for topic, kind in camera_topics:
        node.create_subscription(
            kind, topic, lambda _, topic=topic: cameras.update([topic]),
            QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT), raw=True,
        )
    try:
        deadline = time.monotonic() + 60
        while (count == 0 or len(cameras) != len(camera_topics)) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert count and len(cameras) == len(camera_topics), "Joint states or cameras did not arrive"
        if camera_ready is not None:
            camera_ready.touch()
        if ready is not None:
            while not ready.exists() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            assert ready.exists(), "Rate measurement readiness did not arrive"
        rates = []
        for index in range(windows):
            start = time.monotonic()
            before = count
            camera_before = cameras.copy()
            while time.monotonic() - start < 5:
                rclpy.spin_once(node, timeout_sec=min(0.05, max(0, start + 5 - time.monotonic())))
            rate = (count - before) / (time.monotonic() - start)
            rates.append(rate)
            print(f"Joint-state five-second window {index + 1}: {rate:.3f} Hz", flush=True)
            assert all(cameras[topic] > camera_before[topic] for topic, _ in camera_topics), cameras
        temporary = output.with_suffix(".tmp")
        temporary.write_text(json.dumps(rates))
        temporary.replace(output)
        assert min(rates) >= 48, ("Joint-state window below 48 Hz with cameras enabled", rates)
    finally:
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ready", type=Path)
    parser.add_argument("--camera-ready", type=Path)
    parser.add_argument("--windows", type=int, default=12)
    args = parser.parse_args()
    rclpy.init()
    try:
        check(args.output, args.ready, args.camera_ready, args.windows)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
