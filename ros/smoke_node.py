import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

TOPIC = "/env_smoke"  # TODO(pawel): confirm name


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("role", choices=("talker", "listener"))
    parser.add_argument("payload")
    args = parser.parse_args()
    rclpy.init()
    node = Node(
        f"env_smoke_{args.role}",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    received = False

    def on_message(message):
        nonlocal received
        if message.data == args.payload:
            received = True
            print(f"SMOKE_RECEIVED:{message.data}", flush=True)

    try:
        if args.role == "talker":
            publisher = node.create_publisher(String, TOPIC, 10)
        else:
            node.create_subscription(String, TOPIC, on_message, 10)
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            if args.role == "talker":
                publisher.publish(String(data=args.payload))
            rclpy.spin_once(node, timeout_sec=0.1)
            if received:
                return
        if args.role == "listener":
            raise TimeoutError("No matching ROS smoke message received within 20 s")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
