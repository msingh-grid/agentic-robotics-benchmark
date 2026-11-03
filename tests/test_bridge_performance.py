import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

import psutil
import pytest

from test_env_smoke import ros_domain, stop_process_group

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("private", [False, True], ids=["plain", "fault-enabled"])
def test_camera_enabled_bridge_performance(private, tmp_path, ros_domain):
    pixi = shutil.which("pixi") or str(Path.home() / ".pixi/bin/pixi")
    ros = [pixi, "run", "--locked", "--manifest-path", str(ROOT / "ros/pixi.toml")]
    env = {
        **os.environ, "ROS_DOMAIN_ID": ros_domain,
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST", "PYTHONUNBUFFERED": "1",
    }
    processes = {}
    logs = {}
    with tempfile.TemporaryDirectory(prefix="rate-pub-", dir="/tmp") as public, \
            tempfile.TemporaryDirectory(prefix="rate-priv-", dir="/tmp") as secret:
        private_args = ["--private-dir", secret] if private else []
        ready = tmp_path / "ready"
        camera_ready = tmp_path / "cameras-ready"
        rates_path = tmp_path / "rates.json"
        commands = {
            "server": [sys.executable, "-m", "sandbox.server", "--run-dir", public,
                       "--seed", "0", *private_args],
            "bridge": [*ros, "bridge", "--run-dir", public, *private_args],
            "counter": [*ros, "bash", str(ROOT / "ros/run.sh"), "python",
                        str(ROOT / "ros/check_joint_rate.py"), "--output", str(rates_path),
                        "--ready", str(ready), "--camera-ready", str(camera_ready)],
            "topic-hz": [*ros, "ros-cli", "topic", "hz", "/joint_states",
                         "--wall-time", "--window", "250"],
        }
        try:
            for role, command in commands.items():
                logs[role] = tmp_path / f"{role}.log"
                with logs[role].open("w") as output:
                    processes[role] = subprocess.Popen(
                        command, cwd=ROOT, env=env, stdout=output,
                        stderr=subprocess.STDOUT, start_new_session=True,
                    )
            deadline = time.monotonic() + 60
            bridge = None
            while (
                bridge is None or not camera_ready.exists()
                or logs["topic-hz"].read_text().count("average rate:") < 6
            ):
                assert time.monotonic() < deadline, "Bridge or topic-hz did not become ready"
                assert all(process.poll() is None for process in processes.values())
                for child in psutil.Process(processes["bridge"].pid).children(recursive=True):
                    try:
                        arguments = child.cmdline()
                    except psutil.NoSuchProcess:
                        continue
                    if any(Path(arg).name == "bridge_node.py" for arg in arguments):
                        bridge = child
                time.sleep(0.05)
            bridge.cpu_percent()
            offset = logs["topic-hz"].stat().st_size
            ready.touch()
            if private:
                logs["action"] = tmp_path / "action.log"
                with logs["action"].open("w") as output:
                    processes["action"] = subprocess.Popen(
                        [*ros, "bash", str(ROOT / "ros/run.sh"), "python",
                         str(ROOT / "ros/check_faults.py"), "--mode", "healthy",
                         "--camera-config", str(ROOT / "config/cameras.yaml"),
                         "--output", str(tmp_path / "action.json")],
                        cwd=ROOT, env=env, stdout=output, stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
            windows = []
            for index in range(12):
                start = time.monotonic()
                cli_rates = []
                while time.monotonic() - start < 5:
                    assert all(processes[role].poll() is None for role in ("server", "bridge", "topic-hz"))
                    with logs["topic-hz"].open() as stream:
                        stream.seek(offset)
                        while True:
                            line = stream.readline()
                            if not line.endswith("\n"):
                                break
                            offset = stream.tell()
                            if line.startswith("average rate:"):
                                cli_rates.append(float(line.split(":")[1]))
                    time.sleep(0.02)
                window = {
                    "window": index + 1, "seconds": time.monotonic() - start,
                    "topic_hz": cli_rates, "bridge_cpu_percent": bridge.cpu_percent(),
                }
                windows.append(window)
                print(f"BRIDGE_PERFORMANCE private={private} " + json.dumps(window), flush=True)
            assert processes["counter"].wait(timeout=10) == 0, logs["counter"].read_text()
            rates = json.loads(rates_path.read_text())
            print(f"JOINT_STATE_60S_HZ private={private} " + json.dumps(rates), flush=True)
            assert len(rates) == 12 and min(rates) >= 48, rates
            assert all(window["topic_hz"] and min(window["topic_hz"]) >= 48 for window in windows), windows
            assert all(window["bridge_cpu_percent"] < 50 for window in windows), windows
            if private:
                assert processes["action"].wait(timeout=5) == 0, logs["action"].read_text()
                print(logs["action"].read_text(), flush=True)
        except Exception as error:
            pytest.fail(f"{error}\n" + "\n".join(
                f"{role}:\n{path.read_text()}" for role, path in logs.items()
            ))
        finally:
            for process in reversed(list(processes.values())):
                stop_process_group(process)
