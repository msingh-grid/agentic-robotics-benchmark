"""Measure joint-state arrival rates independently of camera deserialization."""

import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


def check(output, ready):
    node = Node("joint_rate_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
    count = 0

    def record(message):
        nonlocal count
        count += 1

    node.create_subscription(
        JointState, "/joint_states", record,
        QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    try:
        deadline = time.monotonic() + 60
        while (count == 0 or not ready.exists()) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert count and ready.exists(), "Joint states or camera readiness did not arrive"
        rates = []
        for _ in range(4):
            start = time.monotonic()
            before = count
            while time.monotonic() - start < 5:
                rclpy.spin_once(node, timeout_sec=0.05)
            rate = (count - before) / (time.monotonic() - start)
            rates.append(rate)
            print(f"Joint-state five-second window: {rate:.3f} Hz", flush=True)
            assert rate >= 45, ("Joint-state window below 45 Hz with cameras enabled", rate)
        temporary = output.with_suffix(".tmp")
        temporary.write_text(json.dumps(rates))
        temporary.replace(output)
    finally:
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ready", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    try:
        check(args.output, args.ready)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
