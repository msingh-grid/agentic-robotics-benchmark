import fcntl
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]


def stop_process_group(process):
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        pass
    finally:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=2)


@pytest.fixture
def ros_domain():
    for offset in range(102):
        domain = (os.getpid() + offset) % 102
        path = Path(tempfile.gettempdir()) / f"robotics-benchmark-ros-{domain}.lock"
        with path.open("a") as lease:
            try:
                fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                continue
            yield str(domain)
            return
    pytest.fail("All 102 local ROS test domains are reserved")


def test_mujoco_opencv_render(tmp_path):
    env = os.environ.copy()
    backends = [env.get("MUJOCO_GL", "egl")] if sys.platform == "linux" else [None]
    if backends == ["egl"]:
        backends.append("osmesa")
    failures = []
    for backend in backends:
        if backend is not None:
            env["MUJOCO_GL"] = backend
        log = tmp_path / f"render-{backend}.log"
        with log.open("w") as output:
            process = subprocess.Popen(
                [sys.executable, str(ROOT / "scripts" / "render_smoke.py")],
                cwd=ROOT,
                env=env,
                stdout=output,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            try:
                if process.wait(timeout=20) == 0:
                    return
            except subprocess.TimeoutExpired:
                failures.append(f"{backend}: render timed out after 20 s")
            finally:
                stop_process_group(process)
        failures.append(f"{backend}: exit {process.returncode}\n{log.read_text()}")
    pytest.fail("\n".join(failures))


def test_ros_talker_listener(tmp_path, ros_domain):
    pixi_bin = Path(os.environ.get("PIXI_HOME", Path.home() / ".pixi")) / "bin"
    search_path = os.pathsep.join((os.environ.get("PATH", ""), str(pixi_bin)))
    pixi = shutil.which("pixi", path=search_path)
    assert pixi is not None, "pixi is required; run bash scripts/setup.sh"
    env = os.environ.copy()
    env.update(
        ROS_DOMAIN_ID=ros_domain,
        ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
    )
    payload = f"{os.getpid()}-{tmp_path.name}"
    processes = []
    logs = {}
    deadline = time.monotonic() + 60
    failure = None
    try:
        for role in ("listener", "talker"):
            log = tmp_path / f"{role}.log"
            logs[role] = log
            with log.open("w") as output:
                processes.append(subprocess.Popen(
                    [
                        pixi, "run", "--locked", "--manifest-path",
                        str(ROOT / "ros" / "pixi.toml"), "python", "-u",
                        str(ROOT / "ros" / "smoke_node.py"), role, payload,
                    ],
                    cwd=ROOT,
                    env=env,
                    stdout=output,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                ))
        listener, talker = processes
        while time.monotonic() < deadline:
            listener_status = listener.poll()
            if listener_status is not None:
                if listener_status != 0:
                    failure = f"Listener exited with {listener_status}"
                elif f"SMOKE_RECEIVED:{payload}" not in logs["listener"].read_text():
                    failure = "Listener exited without confirming the message"
                break
            if talker.poll() is not None:
                failure = f"Talker exited before receipt with {talker.returncode}"
                break
            time.sleep(0.05)
        else:
            failure = "No ROS message received within 60 s"
    finally:
        for process in reversed(processes):
            stop_process_group(process)
    if failure is not None:
        diagnostics = "\n".join(
            f"{role} (exit {process.returncode}):\n{logs[role].read_text()}"
            for role, process in zip(("listener", "talker"), processes)
        )
        pytest.fail(f"{failure}; ROS_DOMAIN_ID={ros_domain}\n{diagnostics}")
