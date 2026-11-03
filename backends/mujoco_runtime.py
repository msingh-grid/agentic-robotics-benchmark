"""Process supervisor for the native, two-environment sandbox."""

import fcntl
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import psutil

from ros.public_contract import ACTION_TOPICS, TOPICS

ROOT = Path(__file__).resolve().parents[1]


def _group_running(group):
    for member in psutil.process_iter(["pid", "status"]):
        try:
            if member.info["status"] not in (psutil.STATUS_ZOMBIE, psutil.STATUS_DEAD):
                if os.getpgid(member.pid) == group:
                    return True
        except (ProcessLookupError, PermissionError, psutil.NoSuchProcess):
            pass
    return False


def stop_process(process, signum=signal.SIGTERM, timeout=5):
    # Children start new sessions; their PIDs remain group IDs after the leaders exit.
    group = process.pid
    for pending_signal, allowance in ((signum, timeout), (signal.SIGKILL, 5)):
        try:
            os.killpg(group, pending_signal)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + allowance
        while True:
            process.poll()
            if not _group_running(group):
                process.wait(timeout=5)
                return
            if time.monotonic() >= deadline:
                break
            time.sleep(0.02)
    raise RuntimeError("Sandbox process group did not stop")


class Lifecycle:
    def __init__(self):
        self.children = []
        self.stack = []
        self.directories = []
        self.lease = None
        self.ready = False
        pixi = shutil.which("pixi") or str(
            Path(os.environ.get("PIXI_HOME", Path.home() / ".pixi")) / "bin" / "pixi",
        )
        self.pixi = [pixi, "run", "--locked", "--manifest-path", str(ROOT / "ros/pixi.toml")]
        self.ros = [*self.pixi, "bash", str(ROOT / "ros/run.sh")]

    @property
    def ros_domain(self):
        return self.env["ROS_DOMAIN_ID"]

    def start(self, seed):
        if self.lease is not None:
            raise RuntimeError("Lifecycle is already started")
        self.started = time.monotonic()
        self.seed = seed
        self.env = os.environ.copy()
        requested = self.env.get("ROS_DOMAIN_ID")
        domains = [int(requested)] if requested is not None else [
            (os.getpid() + offset) % 102 for offset in range(102)
        ]
        for domain in domains:
            if not 0 <= domain <= 101:
                raise ValueError("ROS_DOMAIN_ID must be in 0–101")
            lease = open(Path(tempfile.gettempdir()) / f"robotics-benchmark-ros-{domain}.lock", "a")
            try:
                fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                lease.close()
                continue
            self.lease = lease
            self.env.update(ROS_DOMAIN_ID=str(domain), ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
            break
        if self.lease is None:
            raise RuntimeError("No free local ROS domain")
        self._launch()

    def _launch(self):
        directory = Path(tempfile.mkdtemp(prefix="cell-", dir="/tmp"))
        self.directories.append(directory)
        self.public = directory / "public"
        self.private = directory / "private"
        self.public.mkdir(mode=0o700)
        self.private.mkdir(mode=0o700)
        self.stack = [
            self._spawn([
                sys.executable, "-m", "sandbox.server", "--run-dir", str(self.public),
                "--private-dir", str(self.private), "--seed", str(self.seed),
            ], directory / "server.log"),
            self._spawn([
                *self.pixi, "bridge", "--run-dir", str(self.public),
                "--private-dir", str(self.private),
            ], directory / "bridge.log"),
        ]

    def _spawn(self, command, log):
        with log.open("w") as stream:
            process = subprocess.Popen(
                command, cwd=ROOT, env=self.env, stdout=stream, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        self.children.append((process, log))
        return process

    def check(self):
        for process in self.stack:
            if process.poll() is not None:
                log = next(log for child, log in self.children if child is process)
                raise RuntimeError(f"Sandbox child exited ({process.returncode}):\n{log.read_text()}")

    def _wait(self, process, deadline):
        while process.poll() is None:
            self.check()
            if time.monotonic() >= deadline:
                raise TimeoutError("Sandbox operation exceeded its readiness/episode budget")
            time.sleep(0.02)
        log = next(log for child, log in self.children if child is process)
        if process.returncode:
            raise RuntimeError(f"Sandbox command failed:\n{log.read_text()}")

    def _probe(self, operation, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Cold start exceeded the readiness budget")
        log = self.directories[-1] / f"{operation}.log"
        process = self._spawn([
            *self.ros, "python", str(ROOT / "ros/runtime.py"),
            operation, "--timeout", str(remaining),
        ], log)
        self._wait(process, deadline)

    def _stop_stack(self):
        stack, self.stack = self.stack, []
        for process in reversed(stack):
            stop_process(process)

    def wait_ready(self, scenario, timeout=60):
        from backends.mac_mujoco import Faults

        deadline = self.started + timeout
        self._probe("dry-pick", deadline)
        verifier = self._spawn([
            sys.executable, "-m", "grading.recording", "check",
            "--private-dir", str(self.private),
        ], self.directories[-1] / "healthy.log")
        self._wait(verifier, deadline)
        self._stop_stack()
        self._launch()
        self._probe("ready", deadline)
        Faults(self.private).configure(scenario, self.seed)
        if time.monotonic() > deadline:
            raise TimeoutError("Cold start exceeded the readiness budget")
        self.ready = True

    def record_episode(self, output):
        if not self.ready:
            raise RuntimeError("Wait for healthy readiness before recording")
        self.ready = False
        output = Path(output).resolve()
        output.mkdir(mode=0o700, parents=True, exist_ok=False)
        grader = output / "grader"
        grader.mkdir(mode=0o700)
        capture = self._spawn([
            sys.executable, "-m", "grading.recording", "capture",
            "--private-dir", str(self.private), "--output", str(grader / "ground_truth.jsonl"),
        ], grader / "capture.log")
        bag_log = self.directories[-1] / "bag.log"
        recorder = self._spawn([
            *self.ros, "ros2", "bag", "record", "--storage", "mcap",
            "--output", str(output / "rosbag"), "--include-hidden-topics",
            "--topics", *TOPICS, *ACTION_TOPICS,
        ], bag_log)
        try:
            deadline = time.monotonic() + 20
            while not all(f"Subscribed to topic '{topic}'" in bag_log.read_text() for topic in TOPICS):
                self.check()
                if recorder.poll() is not None or capture.poll() is not None:
                    raise RuntimeError(f"Episode recorder exited:\n{bag_log.read_text()}")
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"Bag subscriptions did not become ready:\n{bag_log.read_text()}")
                time.sleep(0.05)
            self._probe("pick", time.monotonic() + 35)
            # Allow the final status tick, camera pair and private transport audit to arrive.
            end = time.monotonic() + 3
            while time.monotonic() < end:
                self.check()
                if recorder.poll() is not None or capture.poll() is not None:
                    raise RuntimeError("Episode recorder exited before completion")
                time.sleep(0.05)
        finally:
            try:
                stop_process(recorder, signal.SIGINT, timeout=15)
            finally:
                stop_process(capture, timeout=10)
        # Pixi can return 2 after forwarding SIGINT even when rosbag finalized cleanly.
        if (
            recorder.returncode not in (0, 2) or capture.returncode != 0
            or "Recording stopped" not in bag_log.read_text()
        ):
            raise RuntimeError(
                f"Episode capture failed (bag={recorder.returncode}, truth={capture.returncode}):\n"
                f"{bag_log.read_text()}",
            )
        validator = self._spawn([
            *self.ros, "python", str(ROOT / "ros/validate_bag.py"), str(output / "rosbag"),
        ], self.directories[-1] / "validate-bag.log")
        self._wait(validator, time.monotonic() + 20)

    def stop(self):
        errors = []
        remaining = []
        for process, log in reversed(self.children):
            try:
                stop_process(process)
            except (OSError, RuntimeError) as error:
                remaining.append((process, log))
                errors.append(str(error))
        self.children = list(reversed(remaining))
        self.stack = [process for process in self.stack
                      if any(process is child for child, _ in self.children)]
        self.ready = False
        if not self.children and self.lease is not None:
            self.lease.close()
            self.lease = None
        if errors:
            raise RuntimeError("; ".join(errors))
