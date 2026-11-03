"""Check that an invalid simulator reply aborts only its action."""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from action_msgs.msg import GoalStatus

from benchmark_interfaces.action import PickPlace


def main():
    rclpy.init()
    node = Node("action_recovery_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
    # TODO(pawel): confirm action name.
    action = ActionClient(node, PickPlace, "/pick_place")
    try:
        assert action.wait_for_server(timeout_sec=30)
        for wire, status in (("wire0", GoalStatus.STATUS_ABORTED), ("wire1", GoalStatus.STATUS_SUCCEEDED)):
            goal = action.send_goal_async(PickPlace.Goal(object_id=wire, crate_id="crate0"))
            rclpy.spin_until_future_complete(node, goal, timeout_sec=5)
            assert goal.done() and goal.result().accepted
            result = goal.result().get_result_async()
            rclpy.spin_until_future_complete(node, result, timeout_sec=5)
            assert result.done(), "Action did not return a result"
            assert result.result().status == status, result.result()
            assert result.result().result.success == (status == GoalStatus.STATUS_SUCCEEDED)
    finally:
        action.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
