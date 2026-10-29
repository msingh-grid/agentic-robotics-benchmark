"""ROS-only bridge for public simulation IPC; no access to private ground truth."""

import argparse
import math
import os
from pathlib import Path
import signal
import threading

import msgpack
import numpy as np
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Image, JointState, PointCloud2, PointField
from std_msgs.msg import Header, String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import yaml
import zmq

from benchmark_interfaces.action import PickPlace

ROOT = Path(__file__).resolve().parents[1]
# TODO(pawel): confirm public topic, action, world, mount and optical frame names.
WORLD_FRAME = "world"
ACTION = "/pick_place"


def stamp(nanoseconds):
    seconds, remainder = divmod(nanoseconds, 1_000_000_000)
    return Time(sec=seconds, nanosec=remainder)


def transform(parent, child, position, quaternion, timestamp):
    message = TransformStamped()
    message.header = Header(stamp=timestamp, frame_id=parent)
    message.child_frame_id = child
    message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = (
        float(value) for value in position
    )
    w, x, y, z = quaternion
    message.transform.rotation.w = float(w)
    message.transform.rotation.x = float(x)
    message.transform.rotation.y = float(y)
    message.transform.rotation.z = float(z)
    return message


def camera_transforms(config_path):
    with config_path.open() as source:
        config = yaml.safe_load(source)
    cameras = config["cameras"]
    if not cameras:
        raise ValueError("Camera configuration must not be empty")
    transforms = []
    for name, pose in cameras.items():
        position = np.asarray(pose["position"], dtype=float)
        quaternion = np.asarray(pose["quaternion"], dtype=float)
        if (
            position.shape != (3,) or quaternion.shape != (4,)
            or not np.isfinite(position).all() or not np.isfinite(quaternion).all()
            or not math.isclose(float(np.linalg.norm(quaternion)), 1, abs_tol=1e-6)
        ):
            raise ValueError(f"Invalid configured camera pose: {name}")
        transforms.extend((
            transform(WORLD_FRAME, name, position, quaternion, Time()),
            transform(name, f"{name}_optical", [0, 0, 0], [0, 1, 0, 0], Time()),
        ))
    return cameras, transforms


class Bridge(Node):
    def __init__(self, run_dir, camera_config):
        super().__init__(
            "sim_bridge", parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        cameras, static_transforms = camera_transforms(camera_config)
        self.ipc_context = zmq.Context()
        self.state_socket = self.ipc_context.socket(zmq.SUB)
        self.state_socket.setsockopt(zmq.RCVHWM, 16)
        self.state_socket.setsockopt(zmq.SUBSCRIBE, b"")
        self.state_socket.connect(f"ipc://{run_dir / 'state.sock'}")
        self.command_endpoint = f"ipc://{run_dir / 'commands.sock'}"
        self.clock_publisher = self.create_publisher(
            ClockMessage, "/clock", QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self.joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.flow_publisher = self.create_publisher(String, "/flow/state", 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.images = {}
        self.clouds = {}
        camera_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        for camera in cameras:
            self.images[camera] = self.create_publisher(
                Image, f"/cameras/{camera}/color/image_raw", camera_qos,
            )
            self.clouds[camera] = self.create_publisher(
                PointCloud2, f"/cameras/{camera}/depth/points", camera_qos,
            )
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(static_transforms)
        self.goal_lock = threading.Lock()
        self.action = ActionServer(
            self, PickPlace, ACTION, execute_callback=self.execute_pick,
            goal_callback=self.accept_goal, callback_group=ReentrantCallbackGroup(),
        )
        self.timer = self.create_timer(
            0.005, self.receive, clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

    def accept_goal(self, request):
        if not request.object_id or not request.crate_id or not self.goal_lock.acquire(blocking=False):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute_pick(self, goal):
        goal.publish_feedback(PickPlace.Feedback(state="FORWARDING"))
        socket = self.ipc_context.socket(zmq.REQ)
        socket.setsockopt(zmq.SNDTIMEO, 2000)
        socket.setsockopt(zmq.RCVTIMEO, 2000)
        socket.setsockopt(zmq.LINGER, 0)
        try:
            socket.connect(self.command_endpoint)
            socket.send(msgpack.packb({
                "op": "pick_place",
                "object_id": goal.request.object_id,
                "crate_id": goal.request.crate_id,
            }, use_bin_type=True))
            response = msgpack.unpackb(socket.recv(), raw=False)
            result = PickPlace.Result(
                success=response["success"], message=response["message"],
            )
        except zmq.ZMQError:
            result = PickPlace.Result(success=False, message="Sim command timed out or unavailable")
        finally:
            socket.close()
            self.goal_lock.release()
        if result.success:
            goal.publish_feedback(PickPlace.Feedback(state="DONE"))
            goal.succeed()
        else:
            goal.abort()
        return result

    def receive(self):
        # Bound each drain so action and shutdown callbacks remain responsive under load.
        for _ in range(32):
            try:
                kind, payload = self.state_socket.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            packet = msgpack.unpackb(payload, raw=False)
            timestamp = stamp(packet["time_ns"])
            if kind == b"state":
                self.clock_publisher.publish(ClockMessage(clock=timestamp))
                joints = packet["joints"]
                self.joint_publisher.publish(JointState(
                    header=Header(stamp=timestamp, frame_id=WORLD_FRAME),
                    name=joints["names"], position=joints["position"], velocity=joints["velocity"],
                ))
                self.tf.sendTransform([
                    transform(
                        item["parent"], item["child"], item["position"],
                        item["quaternion"], timestamp,
                    ) for item in packet["transforms"]
                ])
            elif kind == b"status":
                self.flow_publisher.publish(String(data=packet["flow"]))
                self.diagnostics_publisher.publish(DiagnosticArray(
                    header=Header(stamp=timestamp),
                    status=[DiagnosticStatus(
                        level=DiagnosticStatus.WARN, name="sim_bridge",
                        message=packet["status"], hardware_id="mujoco",
                    )],
                ))
            elif kind == b"camera":
                self.publish_camera(packet, timestamp)

    def publish_camera(self, packet, timestamp):
        name = packet["name"]
        header = Header(stamp=timestamp, frame_id=f"{name}_optical")
        height, width = packet["height"], packet["width"]
        self.images[name].publish(Image(
            header=header, height=height, width=width, encoding="rgb8",
            is_bigendian=0, step=width * 3, data=packet["rgb"],
        ))
        depth = np.frombuffer(packet["depth"], dtype="<f4").reshape(height, width)
        rows, columns = np.indices((height, width), dtype=np.float32)
        points = np.stack((
            (columns - (width - 1) / 2) * depth / packet["focal"],
            (rows - (height - 1) / 2) * depth / packet["focal"],
            depth,
        ), axis=-1).astype("<f4", copy=False)
        fields = [
            PointField(name=axis, offset=index * 4, datatype=PointField.FLOAT32, count=1)
            for index, axis in enumerate(("x", "y", "z"))
        ]
        self.clouds[name].publish(PointCloud2(
            header=header, height=height, width=width, fields=fields,
            is_bigendian=False, point_step=12, row_step=width * 12,
            data=points.tobytes(), is_dense=bool(np.isfinite(points).all()),
        ))

    def destroy_node(self):
        self.action.destroy()
        self.state_socket.close(linger=0)
        self.ipc_context.term()
        return super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--camera-config", type=Path, default=ROOT / "config" / "cameras.yaml")
    args, ros_args = parser.parse_known_args()
    # Fast DDS's 512 KiB default segment loses whole multi-megabyte best-effort camera samples.
    if not (
        os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE")
        or os.environ.get("FASTDDS_DEFAULT_PROFILES_FILE")
    ):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(ROOT / "ros" / "fastdds.xml")
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    stop = threading.Event()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: stop.set())
    node = Bridge(args.run_dir.resolve(), args.camera_config)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
