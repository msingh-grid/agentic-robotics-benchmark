import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time

import psutil
import pytest
import yaml

from backends import create_lifecycle
from grading.recording import check_healthy
from ros.public_contract import ROBOT_FRAMES
from sandbox.sim.world import World
from test_env_smoke import ros_domain, stop_process_group

ROOT = Path(__file__).resolve().parents[1]
PIXI = shutil.which("pixi") or str(Path.home() / ".pixi/bin/pixi")


def ros_command(script, *args):
    return [
        PIXI, "run", "--locked", "--manifest-path", str(ROOT / "ros/pixi.toml"),
        "bash", str(ROOT / "ros/run.sh"), "python", str(ROOT / "ros" / script), *map(str, args),
    ]


def test_readiness_contract_and_missing_topic_tf_rejection(ros_domain):
    world = World(seed=0)
    assert ROBOT_FRAMES == {
        world.model.body(index).name for index in range(world.model.nbody)
        if world.model.body(index).name.startswith("robot_")
    }
    env = {**os.environ, "ROS_DOMAIN_ID": ros_domain, "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"}
    result = subprocess.run(
        ros_command("check_readiness.py"), cwd=ROOT, env=env,
        capture_output=True, text=True, timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "READINESS_REJECTION_OK" in result.stdout


def test_preflight_checks_physical_placement_not_command_success():
    snapshot = {
        "verification": {"status": "valid", "faults": [], "bridge": {}},
        "events": [{"kind": "placement_result", "object": "wire0", "crate": "crate0", "inside": True}],
    }
    check_healthy(snapshot)
    snapshot["events"][0]["inside"] = False
    with pytest.raises(RuntimeError, match="physical validation"):
        check_healthy(snapshot)
    snapshot["events"][0]["inside"] = True
    snapshot["verification"]["faults"] = [{"type": "camera_offset", "verified": True}]
    with pytest.raises(RuntimeError, match="physical validation"):
        check_healthy(snapshot)


@pytest.mark.parametrize("case", ["healthy", "drop", "missing-joints"])
def test_record_command_mcap_private_truth_and_ros_replay(case, tmp_path, ros_domain):
    output = tmp_path / "recording"
    command = ["uv", "run", "record", "--episodes", "2" if case == "healthy" else "1",
               "--seed", "0", "--output", str(output)]
    missing = None
    if case != "healthy":
        scenario = tmp_path / "scenario.yaml"
        if case == "drop":
            scenario.write_text((ROOT / "tests/fixtures/drop_in_transit.yaml").read_text())
        else:
            # TODO(pawel): confirm topic name.
            missing = "/joint_states"
            scenario.write_text(yaml.safe_dump({
                "faults": [{"type": "topic_drop", "topic": missing, "rate": 1}],
            }))
        command += ["--scenario", str(scenario)]
    env = os.environ.copy()
    env.pop("ROS_DOMAIN_ID", None)
    result = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True, timeout=240)
    assert result.returncode == 0, result.stdout + result.stderr
    timings = [float(value) for value in re.findall(r"elapsed=([\d.]+)s", result.stdout)]
    assert len(timings) == (2 if case == "healthy" else 1)
    assert max(timings) < 60, result.stdout
    for episode in sorted(output.iterdir()):
        truth = episode / "grader/ground_truth.jsonl"
        assert truth.stat().st_mode & 0o777 == 0o600
        assert episode.stat().st_mode & 0o777 == 0o700
        assert truth.parent.stat().st_mode & 0o777 == 0o700
        snapshots = [json.loads(line) for line in truth.read_text().splitlines()]
        assert len(snapshots) > 50
        assert snapshots[0]["events"] == [], "Dry-pick events leaked into the fresh episode"
        final = snapshots[-1]
        assert final["verification"]["status"] == "valid", final
        placement = final["events"][-1]
        assert placement["kind"] == "placement_result"
        assert placement["inside"] == (case != "drop")
        assert any(event["kind"] == "drop" for event in final["events"]) == (case == "drop")
        assert bool(final["verification"]["faults"]) == (case != "healthy")
        assert list((episode / "rosbag").glob("*.mcap"))
        assert not list((episode / "rosbag").glob("*truth*"))
        replay_env = {
            **os.environ, "ROS_DOMAIN_ID": ros_domain,
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        }
        args = ["--missing-topic", missing] if missing else []
        replay = subprocess.run(
            ros_command("check_recording.py", episode / "rosbag", *args),
            cwd=ROOT, env=replay_env, capture_output=True, text=True, timeout=45,
        )
        assert replay.returncode == 0, replay.stdout + replay.stderr
        assert "BAG_AUDIT_OK" in replay.stdout and "REPLAY_OK" in replay.stdout
        print(result.stdout + replay.stdout)
        if case == "drop":
            metadata = episode / "rosbag/metadata.yaml"
            original = metadata.read_text()
            changed = yaml.safe_load(original)
            changed["rosbag2_bagfile_information"]["message_count"] += 1
            metadata.write_text(yaml.safe_dump(changed))
            invalid = subprocess.run(
                ros_command("validate_bag.py", episode / "rosbag"),
                cwd=ROOT, env=replay_env, capture_output=True, text=True, timeout=20,
            )
            assert invalid.returncode != 0
            assert "do not match finalized metadata" in invalid.stderr
            metadata.write_text(original)
            mcap = next((episode / "rosbag").glob("*.mcap"))
            with mcap.open("r+b") as stream:
                stream.truncate(mcap.stat().st_size - 8)
            invalid = subprocess.run(
                ros_command("validate_bag.py", episode / "rosbag"),
                cwd=ROOT, env=replay_env, capture_output=True, text=True, timeout=20,
            )
            assert invalid.returncode != 0
            assert "MCAP footer is missing" in invalid.stderr


@pytest.mark.parametrize("when", ["starting", "ready"])
def test_bringup_ctrl_c_stops_every_child(when, tmp_path):
    env = os.environ.copy()
    env.pop("ROS_DOMAIN_ID", None)
    log = tmp_path / "bringup.log"
    children = []
    with log.open("w") as stream:
        process = subprocess.Popen(
            ["uv", "run", "bringup", "--seed", "0"], cwd=ROOT, env=env,
            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
        )
    try:
        deadline = time.monotonic() + 65
        while True:
            assert process.poll() is None, log.read_text()
            children = psutil.Process(process.pid).children(recursive=True)
            if when == "ready" and "READY " in log.read_text():
                break
            if when == "starting" and any("sandbox.server" in child.cmdline() for child in children):
                break
            assert time.monotonic() < deadline, log.read_text()
            time.sleep(0.05)
        process.send_signal(signal.SIGINT)
        assert process.wait(timeout=15) == 0, log.read_text()
        assert all(not child.is_running() for child in children), [
            child.pid for child in children if child.is_running()
        ]
        if when == "starting":
            assert "READY " not in log.read_text()
    finally:
        stop_process_group(process)


def test_failed_child_and_readiness_timeout_leave_no_processes(monkeypatch):
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    for failure in ("exit", "timeout"):
        lifecycle = create_lifecycle()
        try:
            lifecycle.start(0)
            children = [process for process, _ in lifecycle.children]
            if failure == "exit":
                os.killpg(children[0].pid, signal.SIGKILL)
                children[0].wait(timeout=5)
                with pytest.raises(RuntimeError, match="child exited"):
                    lifecycle.wait_ready({"faults": []})
            else:
                with pytest.raises(TimeoutError, match="readiness budget"):
                    lifecycle.wait_ready({"faults": []}, timeout=0)
        finally:
            lifecycle.stop()
        assert all(process.poll() is not None for process in children)
        assert lifecycle.lease is None


def test_recorder_crash_fails_and_stops_the_episode(tmp_path):
    env = os.environ.copy()
    env.pop("ROS_DOMAIN_ID", None)
    log = tmp_path / "record.log"
    with log.open("w") as stream:
        process = subprocess.Popen(
            ["uv", "run", "record", "--episodes", "1", "--output", str(tmp_path / "episode")],
            cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True,
        )
    children = []
    try:
        deadline = time.monotonic() + 65
        while True:
            assert process.poll() is None, log.read_text()
            children = psutil.Process(process.pid).children(recursive=True)
            recorders = []
            for child in children:
                try:
                    command = child.cmdline()
                except (psutil.NoSuchProcess, psutil.ZombieProcess):
                    continue
                if all(word in command for word in ("bag", "record", "--storage")):
                    recorders.append(child)
            if recorders:
                recorders[-1].kill()
                break
            assert time.monotonic() < deadline, log.read_text()
            time.sleep(0.02)
        assert process.wait(timeout=20) != 0
        assert "RECORDED " not in log.read_text()
        assert "recorder exited" in log.read_text().lower()
        assert all(not child.is_running() for child in children)
    finally:
        stop_process_group(process)
