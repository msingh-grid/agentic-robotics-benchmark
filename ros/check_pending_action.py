"""Start an action and keep its response pending for the shutdown integration test."""

import argparse
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from benchmark_interfaces.action import PickPlace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ready", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node("pending_action_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
    # TODO(pawel): confirm action name.
    action = ActionClient(node, PickPlace, "/pick_place")
    try:
        assert action.wait_for_server(timeout_sec=30)
        goal = action.send_goal_async(PickPlace.Goal(object_id="wire0", crate_id="crate0"))
        rclpy.spin_until_future_complete(node, goal, timeout_sec=10)
        assert goal.done() and goal.result().accepted
        response = goal.result().get_result_async()
        args.ready.touch()
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            assert not response.done(), "Action completed before the simulator replied"
    finally:
        action.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
