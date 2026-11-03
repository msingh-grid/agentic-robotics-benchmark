"""Public sensor telemetry and command-only actions over per-run local IPC."""

import argparse
import math
from pathlib import Path
import queue
import signal
import tempfile
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
from sandbox.sim.faults import Faults, load_scenario
from sandbox.sim.ground_truth import GroundTruth, bind_private

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
    def __init__(self, run_dir, seed, camera_config=CAMERA_CONFIG, *, private_dir=None, scenario=None):
        self.world = World(seed=seed)
        self.seed = seed
        self.run_dir = Path(run_dir).resolve()
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.private_dir = Path(private_dir or tempfile.mkdtemp(prefix="truth-", dir="/tmp")).resolve()
        if self.private_dir == self.run_dir or self.run_dir in self.private_dir.parents:
            raise ValueError("Private storage must be outside the public run directory")
        self.context = zmq.Context()
        self.state_socket = self.context.socket(zmq.PUB)
        self.state_socket.setsockopt(zmq.SNDHWM, 16)
        self.command_socket = self.context.socket(zmq.REP)
        self.command_socket.setsockopt(zmq.MAXMSGSIZE, 65536)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.frames = queue.Queue(maxsize=len(CAMERAS))
        self.render_error = None
        self.truth = GroundTruth(self.world)
        self.faults_configured = scenario is not None
        self.faults = Faults(
            self.world, Perception(self.world, config_path=camera_config),
            scenario if scenario is not None else {"faults": []}, seed,
        )
        self.flow = PickPlaceFlow(
            self.world, perception=self.faults,
            on_state=self.on_state,
        )
        self.camera_data = mujoco.MjData(self.world.model)
        self.camera_barrier = threading.Barrier(2)
        self.workers = [
            threading.Thread(
                target=self.render_cameras, args=(CAMERAS[index::2], index == 0), daemon=True,
            ) for index in range(2)
        ]

    def on_state(self, state):
        self.flow_state = state.value

    def command(self, command):
        if not isinstance(command, dict) or command.get("op") != "pick_place":
            return {"success": False, "message": "Unknown command"}
        try:
            wire, crate = command.get("object_id"), command.get("crate_id")
            if self.flow.state not in (FlowState.IDLE, FlowState.DONE):
                raise RuntimeError("busy")
            if wire not in {f"wire{i}" for i in range(7)} or crate not in {"crate0", "crate1"}:
                raise ValueError("unknown object or crate")
            self.faults.begin(wire, crate)
            self.flow.start(wire, crate)
            self.truth.begin(wire, crate)
        except (ValueError, RuntimeError, TypeError):
            return {"success": False, "message": "Invalid or unavailable pick-and-place command"}
        return None

    def step(self):
        if self.flow.state in (FlowState.IDLE, FlowState.DONE):
            if self.flow.state == FlowState.DONE:
                # Bias forces change as the arm settles; stale compensation moves an idle arm.
                dofs = self.flow.ik.dofs
                self.world.data.qfrc_applied[dofs] = self.world.data.qfrc_bias[dofs]
            self.world.step()
        else:
            self.faults.before_step(self.flow)
            self.flow.step()
        self.truth.observe(self.flow)

    def private_requests(self, sockets):
        for name, socket in sockets.items():
            if not socket.poll(0):
                continue
            try:
                request = msgpack.unpackb(socket.recv(), raw=False)
                if name == "ground_truth" and request == {"op": "snapshot"}:
                    result = {
                        **self.truth.snapshot(),
                        "verification": self.faults.verification(self.truth),
                    }
                elif name == "faults" and request.get("op") == "configure":
                    if self.flow.state != FlowState.IDLE or self.faults_configured:
                        raise ValueError("Faults must be configured once before the first action")
                    if request["seed"] != self.seed:
                        raise ValueError("Fault seed must match the world seed")
                    self.faults = Faults(
                        self.world, self.faults.perception, request["scenario"], self.seed,
                    )
                    self.flow.perception = self.faults
                    self.faults_configured = True
                    result = {"configured": True}
                elif name == "bridge" and request.get("op") == "poll":
                    self.faults.bridge_audit = request.get("audit", {})
                    result = self.faults.bridge_plan()
                else:
                    raise ValueError("Unknown private request")
            except (ValueError, TypeError, KeyError, AttributeError, msgpack.UnpackException):
                result = {"error": "Invalid private request"}
            socket.send(msgpack.packb(result, use_bin_type=True))

    def render_cameras(self, cameras, capture):
        data = self.camera_data
        try:
            with SceneRenderer(self.world.model) as renderer:
                while not self.stop.is_set():
                    if capture:
                        with self.lock:
                            mujoco.mj_copyData(data, self.world.model, self.world.data)
                    self.camera_barrier.wait()
                    deadline = time.monotonic() + 1 / CAMERA_HZ
                    stamp = round(data.time * 1e9)
                    for camera in cameras:
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
                    # Neither worker may replace the snapshot while its peer still renders it.
                    self.camera_barrier.wait()
                    self.stop.wait(max(0, deadline - time.monotonic()))
        except threading.BrokenBarrierError:
            pass
        except Exception as error:
            self.render_error = error
            self.stop.set()
        finally:
            self.camera_barrier.abort()

    def publish(self, kind, packet):
        self.state_socket.send_multipart([
            kind, msgpack.packb(packet, use_bin_type=True),
        ])

    def run(self):
        self.state_socket.bind(f"ipc://{self.run_dir / 'state.sock'}")
        self.command_socket.bind(f"ipc://{self.run_dir / 'commands.sock'}")
        private_sockets = {}
        for name in ("ground_truth", "faults", "bridge"):
            socket = self.context.socket(zmq.REP)
            socket.setsockopt(zmq.MAXMSGSIZE, 65536)
            bind_private(socket, self.private_dir, f"{name}.sock")
            private_sockets[name] = socket
        for worker in self.workers:
            worker.start()
        tick = 0
        pending_result = False
        deadline = time.monotonic()
        try:
            while not self.stop.is_set():
                with self.lock:
                    self.private_requests(private_sockets)
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
            self.camera_barrier.abort()
            for worker in self.workers:
                worker.join(timeout=10)
            self.state_socket.close(linger=0)
            self.command_socket.close(linger=0)
            for socket in private_sockets.values():
                socket.close(linger=0)
            self.context.term()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--camera-config", type=Path, default=CAMERA_CONFIG)
    parser.add_argument("--private-dir", type=Path)
    parser.add_argument("--scenario", type=Path)
    args = parser.parse_args()
    server = SimServer(
        args.run_dir, args.seed, args.camera_config, private_dir=args.private_dir,
        scenario=load_scenario(args.scenario) if args.scenario else None,
    )
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: server.stop.set())
    server.run()


if __name__ == "__main__":
    main()
