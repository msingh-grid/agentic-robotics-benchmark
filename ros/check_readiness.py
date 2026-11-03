"""Assert the real readiness gate rejects absent topics and incomplete TF."""

import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from public_contract import FRAMES, TOPICS
from runtime import Readiness
from bridge_node import transform, Time


def main():
    rclpy.init()
    node = Readiness()
    publisher = Node("partial_tf_fixture")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(publisher)
    tf = publisher.create_publisher(
        TFMessage, "/tf_static", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
    )
    missing_frame = sorted(FRAMES)[-1]
    tf.publish(TFMessage(transforms=[
        transform("world", child, [0, 0, 0], [1, 0, 0, 0], Time())
        for child in FRAMES - {missing_frame}
    ]))
    try:
        deadline = time.monotonic() + 5
        while len(node.edges) < len(FRAMES) - 1 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert len(node.edges) == len(FRAMES) - 1, node.edges
        missing = node.missing()
        assert set(TOPICS) - {"/tf_static"} <= set(missing), missing
        assert f"TF:{missing_frame}" in missing, missing
        assert not any(f"TF:{frame}" in missing for frame in FRAMES - {missing_frame}), missing
        try:
            node.wait(lambda: not node.missing(), time.monotonic() + 0.2, "Readiness timed out")
        except TimeoutError as error:
            assert "/joint_states" in str(error) and f"TF:{missing_frame}" in str(error)
        else:
            raise AssertionError("Incomplete ROS system passed readiness")
        print("READINESS_REJECTION_OK", flush=True)
    finally:
        executor.shutdown()
        node.destroy_node()
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
