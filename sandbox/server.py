"""Public sensor telemetry and command-only actions over per-run local IPC."""

import argparse
import math
from pathlib import Path
import queue
import signal
import threading
import time

import msgpack
import mujoco
import numpy as np
import zmq

from sandbox.sim.render import SceneRenderer
from sandbox.sim.perception import CAMERA_CONFIG, Perception
from sandbox.sim.pick_place import FlowState, PickPlaceFlow
from sandbox.sim.world import CAMERAS, World

STATE_HZ = 50
CAMERA_HZ = 0.5
WORLD_FRAME = "world"  # TODO(pawel): confirm name


def robot_state(world):
    model, data = world.model, world.data
    joints = [
        i for i in range(model.njnt)
        if model.joint(i).name.startswith("robot_")
        and int(model.jnt_type[i]) in (
            int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE),
        )
    ]
    transforms = []
    # TODO(pawel): confirm robot joint and link names inherited from the cell model.
    for i in range(1, model.nbody):
        name = model.body(i).name
        if not name.startswith("robot_"):
            continue
        parent = model.body(i).parentid[0]
        rotation = data.xmat[parent].reshape(3, 3).T
        position = rotation @ (data.xpos[i] - data.xpos[parent])
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(
            quaternion, (rotation @ data.xmat[i].reshape(3, 3)).ravel(),
        )
        transforms.append({
            "parent": model.body(parent).name if parent else WORLD_FRAME,
            "child": name,
            "position": position.tolist(),
            "quaternion": quaternion.tolist(),
        })
    return {
        "names": [model.joint(i).name for i in joints],
        "position": [float(data.qpos[model.jnt_qposadr[i]]) for i in joints],
        "velocity": [float(data.qvel[model.jnt_dofadr[i]]) for i in joints],
    }, transforms


class SimServer:
    def __init__(self, run_dir, seed, camera_config=CAMERA_CONFIG):
        self.world = World(seed=seed)
        self.run_dir = Path(run_dir).resolve()
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.context = zmq.Context()
        self.state_socket = self.context.socket(zmq.PUB)
        self.state_socket.setsockopt(zmq.SNDHWM, 16)
        self.command_socket = self.context.socket(zmq.REP)
        self.command_socket.setsockopt(zmq.MAXMSGSIZE, 65536)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.frames = queue.Queue(maxsize=len(CAMERAS))
        self.render_error = None
        self.flow = PickPlaceFlow(
            self.world, perception=Perception(self.world, config_path=camera_config),
            on_state=self.on_state,
        )
        self.worker = threading.Thread(target=self.render_cameras, daemon=True)

    def on_state(self, state):
        self.flow_state = state.value

    def command(self, command):
        if not isinstance(command, dict) or command.get("op") != "pick_place":
            return {"success": False, "message": "Unknown command"}
        try:
            self.flow.start(command.get("object_id"), command.get("crate_id"))
        except (ValueError, RuntimeError, TypeError) as error:
            return {"success": False, "message": str(error)}
        return None

    def step(self):
        if self.flow.state in (FlowState.IDLE, FlowState.DONE):
            if self.flow.state == FlowState.DONE:
                # Bias forces change as the arm settles; stale compensation moves an idle arm.
                dofs = self.flow.ik.dofs
                self.world.data.qfrc_applied[dofs] = self.world.data.qfrc_bias[dofs]
            self.world.step()
        else:
            self.flow.step()

    def render_cameras(self):
        data = mujoco.MjData(self.world.model)
        try:
            with SceneRenderer(self.world.model) as renderer:
                while not self.stop.is_set():
                    deadline = time.monotonic() + 1 / CAMERA_HZ
                    with self.lock:
                        mujoco.mj_copyData(data, self.world.model, self.world.data)
                    stamp = round(data.time * 1e9)
                    for camera in CAMERAS:
                        rgb, depth = renderer.render(data, camera)
                        height, width = depth.shape
                        focal = height / (2 * math.tan(math.radians(
                            self.world.model.camera(camera).fovy[0],
                        ) / 2))
                        frame = {
                            "time_ns": stamp,
                            "name": camera,
                            "width": width,
                            "height": height,
                            "focal": focal,
                            "rgb": rgb.tobytes(),
                            "depth": depth.astype("<f4", copy=False).tobytes(),
                        }
                        try:
                            self.frames.put_nowait(frame)
                        except queue.Full:
                            pass
                    self.stop.wait(max(0, deadline - time.monotonic()))
        except Exception as error:
            self.render_error = error
            self.stop.set()

    def publish(self, kind, packet):
        self.state_socket.send_multipart([
            kind, msgpack.packb(packet, use_bin_type=True),
        ])

    def run(self):
        self.state_socket.bind(f"ipc://{self.run_dir / 'state.sock'}")
        self.command_socket.bind(f"ipc://{self.run_dir / 'commands.sock'}")
        self.worker.start()
        tick = 0
        pending_result = False
        deadline = time.monotonic()
        try:
            while not self.stop.is_set():
                if not pending_result and self.command_socket.poll(0):
                    try:
                        command = msgpack.unpackb(self.command_socket.recv(), raw=False)
                        with self.lock:
                            result = self.command(command)
                    except (ValueError, TypeError, msgpack.UnpackException):
                        result = {"success": False, "message": "Malformed command"}
                    if result is None:
                        pending_result = True
                    else:
                        self.command_socket.send(msgpack.packb(result, use_bin_type=True))
                with self.lock:
                    for _ in range(round(1 / STATE_HZ / self.world.model.opt.timestep)):
                        self.step()
                    joints, transforms = robot_state(self.world)
                    packet = {
                        "time_ns": round(self.world.data.time * 1e9),
                        "joints": joints,
                        "transforms": transforms,
                        "flow": self.flow_state,
                        "status": "Simulation running",
                    }
                if pending_result and self.flow.result is not None:
                    self.command_socket.send(msgpack.packb({
                        "success": self.flow.result, "message": "Commands completed",
                    }, use_bin_type=True))
                    pending_result = False
                self.publish(b"state", packet)
                if tick % STATE_HZ == 0:
                    self.publish(b"status", packet)
                for _ in CAMERAS:
                    try:
                        self.publish(b"camera", self.frames.get_nowait())
                    except queue.Empty:
                        break
                tick += 1
                deadline += 1 / STATE_HZ
                self.stop.wait(max(0, deadline - time.monotonic()))
            if self.render_error is not None:
                raise RuntimeError("Camera rendering failed") from self.render_error
        finally:
            self.stop.set()
            self.worker.join(timeout=10)
            self.state_socket.close(linger=0)
            self.command_socket.close(linger=0)
            self.context.term()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--camera-config", type=Path, default=CAMERA_CONFIG)
    args = parser.parse_args()
    server = SimServer(args.run_dir, args.seed, args.camera_config)
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: server.stop.set())
    server.run()


if __name__ == "__main__":
    main()
