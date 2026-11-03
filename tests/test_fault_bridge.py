import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

import msgpack
import pytest
import zmq

from backends import create_faults, create_ground_truth
from test_env_smoke import ros_domain, stop_process_group
from test_faults import PHYSICAL_FAULTS

ROOT = Path(__file__).resolve().parents[1]

# TODO(pawel): confirm topic name.
CASES = [None, *PHYSICAL_FAULTS,
         {"type": "topic_delay", "topic": "/joint_states", "ms": 240},
         {"type": "topic_drop", "topic": "/joint_states", "rate": 0.75}]


@pytest.mark.parametrize("fault", CASES, ids=lambda fault: fault["type"] if fault else "healthy")
def test_faults_through_live_server_bridge_and_ros_action(fault, tmp_path, ros_domain):
    pixi = shutil.which("pixi") or str(Path.home() / ".pixi" / "bin" / "pixi")
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=ros_domain, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    processes = []
    logs = []
    with tempfile.TemporaryDirectory(prefix="s5-pub-", dir="/tmp") as public, \
            tempfile.TemporaryDirectory(prefix="s5-priv-", dir="/tmp") as private:
        truth = create_ground_truth(private)
        output = tmp_path / "delivery.json"
        commands = [
            [sys.executable, "-m", "sandbox.server", "--run-dir", public,
             "--private-dir", private, "--seed", "0"],
            [pixi, "run", "--locked", "--manifest-path", str(ROOT / "ros/pixi.toml"),
             "bridge", "--run-dir", public, "--private-dir", private],
            [pixi, "run", "--locked", "--manifest-path", str(ROOT / "ros/pixi.toml"),
             "bash", str(ROOT / "ros/run.sh"), "python", str(ROOT / "ros/check_faults.py"),
             "--mode", fault["type"] if fault else "healthy", "--camera-config",
             str(ROOT / "config/cameras.yaml"), "--output", str(output)],
        ]
        try:
            for index, command in enumerate(commands):
                log = tmp_path / f"process-{index}.log"
                logs.append(log)
                with log.open("w") as stream:
                    process = subprocess.Popen(
                        command, cwd=ROOT, env=env, stdout=stream,
                        stderr=subprocess.STDOUT, start_new_session=True,
                    )
                processes.append(process)
                if index == 0:
                    deadline = time.monotonic() + 25
                    while not (Path(private) / "faults.sock").exists():
                        assert process.poll() is None, log.read_text()
                        assert time.monotonic() < deadline, "Private socket never bound"
                        time.sleep(0.02)
                    for path in Path(private).glob("*.sock"):
                        assert path.stat().st_mode & 0o777 == 0o600
                    assert not list(Path(public).glob("*truth*"))
                    create_faults(private).configure({"faults": [fault] if fault else []}, 0)
                    with pytest.raises(ValueError):
                        create_faults(private).configure({"faults": []}, 99)
                    assert truth.snapshot()["verification"]["status"] == "pending"
                    with zmq.Context() as context:
                        with context.socket(zmq.REQ) as socket:
                            socket.setsockopt(zmq.LINGER, 0)
                            socket.connect(f"ipc://{public}/commands.sock")
                            for command in (
                                {"op": "snapshot"}, {"op": "configure", "scenario": {"faults": []}},
                                {"op": "ground_truth"},
                            ):
                                socket.send(msgpack.packb(command))
                                assert socket.poll(5000)
                                assert set(msgpack.unpackb(socket.recv(), raw=False)) == {"success", "message"}
            assert processes[-1].wait(timeout=65) == 0, logs[-1].read_text()
            measured = json.loads(output.read_text())
            assert measured["success"] is True
            snapshot = truth.snapshot()
            deadline = time.monotonic() + 5
            while snapshot["verification"]["status"] != "valid" and time.monotonic() < deadline:
                time.sleep(0.05)
                snapshot = truth.snapshot()
            assert snapshot["verification"]["status"] == "valid", snapshot
            assert all(item["verified"] for item in snapshot["verification"]["faults"])
            if fault and fault["type"].startswith("topic_"):
                audit = snapshot["verification"]["bridge"][fault["topic"]]
                assert audit["received"] > 100
                if fault["type"] == "topic_delay":
                    assert audit["min_delay_ns"] >= 240_000_000
                else:
                    assert audit["dropped"] > audit["delivered"] > 20
                stop_process_group(processes[1])
                with zmq.Context() as context:
                    with context.socket(zmq.REQ) as socket:
                        socket.setsockopt(zmq.LINGER, 0)
                        socket.connect(f"ipc://{public}/commands.sock")
                        socket.send(msgpack.packb({
                            "op": "pick_place", "object_id": "wire1", "crate_id": "crate1",
                        }))
                        assert socket.poll(30000)
                        assert msgpack.unpackb(socket.recv(), raw=False)["success"] is True
                later = truth.snapshot()["verification"]
                assert later["status"] == "invalid", later
                assert later["faults"] == [{"type": fault["type"], "verified": False}]
            with pytest.raises(ValueError):
                create_faults(private).configure({"faults": []}, 0)
            print(f"{fault}: {measured}; {snapshot['verification']}")
        except Exception as error:
            pytest.fail(f"{error}\n" + "\n".join(path.read_text() for path in logs))
        finally:
            for process in reversed(processes):
                stop_process_group(process)
