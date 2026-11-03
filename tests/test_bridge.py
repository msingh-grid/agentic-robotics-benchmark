import ast
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

import msgpack
import numpy as np
import pytest
import yaml
import zmq

from sandbox.server import robot_state, SimServer
from sandbox.sim.world import CAMERAS, World
from sandbox.sim.pick_place import FlowState
from test_env_smoke import ros_domain, stop_process_group
from test_pick_place import inside_crate

ROOT = Path(__file__).resolve().parents[1]


def test_public_robot_state_and_command_only_flow(tmp_path):
    server = SimServer(tmp_path, seed=42)
    try:
        joints, transforms = robot_state(server.world)
        assert joints["names"] and all(name.startswith("robot_") for name in joints["names"])
        assert all(item["child"].startswith("robot_") for item in transforms)
        assert all(set(item) == {"parent", "child", "position", "quaternion"} for item in transforms)
        assert not any(
            name in {item["child"] for item in transforms}
            for name in [*CAMERAS, "table", "wire0", "crate0"]
        )
        for command in (None, {}, {"op": "ground_truth"}, {
            "op": "pick_place", "object_id": "missing", "crate_id": "crate0",
        }):
            assert not server.command(command)["success"]
        assert server.command({
            "op": "pick_place", "object_id": "wire0", "crate_id": "crate0",
        }) is None
        before = server.world.data.time
        server.step()
        assert server.world.data.time == pytest.approx(before + 0.002)
        server.world.set_body_pose("wire0", [0.7, 0.45, 0.8], [1, 0, 0, 0])
        while server.flow.result is None:
            server.step()
        assert server.flow.result is True
        assert server.flow_state == "DONE"
        assert not inside_crate(server.world, "wire0", "crate0")
    finally:
        server.state_socket.close(linger=0)
        server.command_socket.close(linger=0)
        server.context.term()


def test_completed_flow_holds_arm_between_picks(tmp_path):
    server = SimServer(tmp_path, seed=0)
    try:
        for wire, crate in (("wire0", "crate0"), ("wire1", "crate1")):
            assert server.command({
                "op": "pick_place", "object_id": wire, "crate_id": crate,
            }) is None
            while server.flow.result is None:
                server.step()
            assert inside_crate(server.world, wire, crate)
            arm = server.flow.ik.qpos
            dofs = server.flow.ik.dofs
            for _ in range(500):
                server.step()
            resting = server.world.data.qpos[arm].copy()
            before = server.world.data.time
            for _ in range(5000):
                bias = server.world.data.qfrc_bias[dofs].copy()
                server.step()
                np.testing.assert_array_equal(server.world.data.qfrc_applied[dofs], bias)
                np.testing.assert_allclose(server.world.data.qpos[arm], resting, atol=0.0001, rtol=0)
            assert server.world.data.time == pytest.approx(before + 10)
    finally:
        server.state_socket.close(linger=0)
        server.command_socket.close(linger=0)
        server.context.term()


def test_environment_import_boundaries():
    for directory in ("sandbox", "tests"):
        for path in (ROOT / directory).rglob("*.py"):
            tree = ast.parse(path.read_text())
            for item in ast.walk(tree):
                imports = []
                if isinstance(item, ast.Import):
                    imports = [alias.name for alias in item.names]
                elif isinstance(item, ast.ImportFrom):
                    imports = [item.module or ""]
                assert not any(name.startswith(("rclpy", "sensor_msgs", "tf2_ros")) for name in imports), path
    tree = ast.parse((ROOT / "ros" / "bridge_node.py").read_text())
    for item in ast.walk(tree):
        if isinstance(item, ast.ImportFrom):
            assert not (item.module or "").startswith("sandbox")


def test_sim_ros_bridge(tmp_path, ros_domain):
    pixi = shutil.which("pixi") or str(Path.home() / ".pixi" / "bin" / "pixi")
    config = yaml.safe_load((ROOT / "config" / "cameras.yaml").read_text())
    assert tuple(config["cameras"]) == CAMERAS
    config["cameras"]["cam0"]["position"] = [9, -8, 7]
    config_path = tmp_path / "cameras.yaml"
    config_path.write_text(yaml.safe_dump(config))
    world = World(seed=42)
    joint_names = [
        world.model.joint(i).name for i in range(world.model.njnt)
        if world.model.joint(i).name.startswith("robot_")
    ]
    links = [
        world.model.body(i).name for i in range(world.model.nbody)
        if world.model.body(i).name.startswith("robot_")
    ]
    expected = tmp_path / "expected.json"
    expected.write_text(json.dumps({
        "joints": joint_names, "links": links, "flow_states": [state.value for state in FlowState],
    }))
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=ros_domain, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    stop_server = tmp_path / "stop-server"
    server_stopped = tmp_path / "server-stopped"
    joint_rates = tmp_path / "joint-rates.json"
    rate_ready = tmp_path / "rate-ready"
    processes = {}
    logs = {}
    # macOS Unix socket paths have a short limit; pytest's per-test paths can exceed it.
    with tempfile.TemporaryDirectory(prefix="s4-", dir="/tmp") as run_dir:
        commands = {
            "server": [
                sys.executable, "-m", "sandbox.server", "--run-dir", run_dir, "--seed", "42",
                "--camera-config", str(config_path),
            ],
            "bridge": [
                pixi, "run", "--locked", "--manifest-path", str(ROOT / "ros" / "pixi.toml"),
                "bridge", "--run-dir", run_dir, "--camera-config", str(config_path),
            ],
            "rates": [
                pixi, "run", "--locked", "--manifest-path", str(ROOT / "ros" / "pixi.toml"),
                "bash", str(ROOT / "ros" / "run.sh"), "python", str(ROOT / "ros" / "check_joint_rate.py"),
                "--output", str(joint_rates), "--ready", str(rate_ready),
            ],
            "probe": [
                pixi, "run", "--locked", "--manifest-path", str(ROOT / "ros" / "pixi.toml"),
                "bash", str(ROOT / "ros" / "run.sh"), "python", str(ROOT / "ros" / "check_bridge.py"),
                "--expected", str(expected), "--camera-config", str(config_path),
                "--joint-rates", str(joint_rates), "--rate-ready", str(rate_ready),
                "--stop-server", str(stop_server), "--server-stopped", str(server_stopped),
            ],
        }
        try:
            for role, command in commands.items():
                logs[role] = tmp_path / f"{role}.log"
                with logs[role].open("w") as output:
                    processes[role] = subprocess.Popen(
                        command, cwd=ROOT, env=env, stdout=output,
                        stderr=subprocess.STDOUT, start_new_session=True,
                    )
            context = zmq.Context()
            socket = context.socket(zmq.REQ)
            socket.setsockopt(zmq.LINGER, 0)
            try:
                socket.connect(f"ipc://{run_dir}/commands.sock")
                socket.send(b"\xc1")
                assert socket.poll(20000), "Sim command server did not start"
                assert not msgpack.unpackb(socket.recv(), raw=False)["success"]
            finally:
                socket.close()
                context.term()
            with zmq.Context() as context:
                with context.socket(zmq.SUB) as socket:
                    socket.setsockopt(zmq.SUBSCRIBE, b"")
                    socket.connect(f"ipc://{run_dir}/state.sock")
                    packets = {}
                    deadline = time.monotonic() + 20
                    while set(packets) != {b"state", b"status", b"camera"}:
                        assert time.monotonic() < deadline, "Public telemetry did not arrive"
                        if socket.poll(100):
                            kind, payload = socket.recv_multipart()
                            packets[kind] = msgpack.unpackb(payload, raw=False)
                    for kind in (b"state", b"status"):
                        assert set(packets[kind]) == {
                            "time_ns", "joints", "transforms", "flow", "status",
                        }
                        assert set(packets[kind]["joints"]) == {"names", "position", "velocity"}
                        assert {item["child"] for item in packets[kind]["transforms"]} == set(links)
                    assert set(packets[b"camera"]) == {
                        "time_ns", "name", "width", "height", "focal", "rgb", "depth",
                    }
                    assert packets[b"camera"]["time_ns"] <= packets[b"state"]["time_ns"]
            deadline = time.monotonic() + 100
            while time.monotonic() < deadline:
                if stop_server.exists() and not server_stopped.exists():
                    stop_process_group(processes["server"])
                    server_stopped.touch()
                if processes["probe"].poll() is not None:
                    break
                for role in ("bridge", "server"):
                    if role == "server" and server_stopped.exists():
                        continue
                    assert processes[role].poll() is None, f"{role} exited early"
                time.sleep(0.05)
            assert processes["probe"].poll() == 0, "ROS bridge probe failed or timed out"
            assert processes["rates"].poll() == 0, "Joint-state rate reader failed"
            assert "BRIDGE_CHECK_OK" in logs["probe"].read_text()
            print(logs["probe"].read_text())
        except Exception as error:
            pytest.fail(f"{error}\n" + "\n".join(
                f"{role}:\n{path.read_text()}" for role, path in logs.items()
            ))
        finally:
            for process in reversed(list(processes.values())):
                stop_process_group(process)
