"""Capture a running server's public cameras during a real ROS pick action."""

import argparse
from bisect import bisect_right
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import cv2
import msgpack
import numpy as np
import yaml
import zmq

ROOT = Path(__file__).resolve().parents[1]
ACTION = "/pick_place"  # TODO(pawel): confirm action name.


def tile(views, cameras, state, stamp):
    images = [
        cv2.resize(cv2.cvtColor(views[camera], cv2.COLOR_RGB2BGR), (320, 240))
        for camera in cameras
    ]
    image = np.vstack([np.hstack(images[i:i + 2]) for i in range(0, len(images), 2)])
    header = np.full((28, image.shape[1], 3), 25, dtype=np.uint8)
    cv2.putText(
        header, f"ROS /pick_place | {state} | sim t={stamp / 1e9:.2f}s",
        (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
    )
    return np.vstack((header, image))


def capture(args):
    cameras = list(yaml.safe_load(args.camera_config.read_text())["cameras"])
    if len(cameras) != 4:
        raise ValueError("The cell preview requires four configured cameras")
    pixi = shutil.which("pixi") or str(Path.home() / ".pixi" / "bin" / "pixi")
    output = ROOT / "out"
    output.mkdir(exist_ok=True)
    history = {}
    groups = {}
    images = []
    stages = {}
    action = None
    started = False
    done_at = None
    deadline = time.monotonic() + 60
    with zmq.Context() as context, context.socket(zmq.SUB) as stream:
        stream.setsockopt(zmq.SUBSCRIBE, b"")
        stream.connect(f"ipc://{args.run_dir.resolve() / 'state.sock'}")
        with (output / "ros_episode_action.log").open("w") as action_log:
            try:
                while time.monotonic() < deadline:
                    if not stream.poll(100):
                        continue
                    kind, payload = stream.recv_multipart()
                    packet = msgpack.unpackb(payload, raw=False)
                    if kind == b"state":
                        history[packet["time_ns"]] = packet["flow"]
                        if action is not None and packet["flow"] not in ("IDLE", "DONE"):
                            started = True
                        if started and packet["flow"] == "DONE" and done_at is None:
                            done_at = packet["time_ns"]
                    elif kind == b"camera":
                        stamp = packet["time_ns"]
                        group = groups.setdefault(stamp, {})
                        group[packet["name"]] = np.frombuffer(packet["rgb"], dtype=np.uint8).reshape(
                            packet["height"], packet["width"], 3,
                        )
                        if set(group) != set(cameras):
                            continue
                        times = sorted(history)
                        index = bisect_right(times, stamp) - 1
                        if index < 0:
                            del groups[stamp]
                            continue
                        state = history[times[index]]
                        images.append(tile(group, cameras, state, stamp))
                        stages[state] = tile(group, [cameras[0], cameras[2]], state, stamp)
                        del groups[stamp]
                        if action is None:
                            if state != "IDLE":
                                raise RuntimeError("Restart the seeded server before capturing a new episode")
                            action = subprocess.Popen(
                                [
                                    pixi, "run", "--locked", "--manifest-path", str(ROOT / "ros/pixi.toml"),
                                    "ros-cli", "action", "send_goal", ACTION,
                                    "benchmark_interfaces/action/PickPlace",
                                    f"{{object_id: {args.wire}, crate_id: {args.crate}}}", "--feedback",
                                ],
                                cwd=ROOT, stdout=action_log, stderr=subprocess.STDOUT,
                                start_new_session=True,
                            )
                        elif done_at is not None and stamp >= done_at and action.poll() is not None:
                            if action.returncode != 0:
                                raise RuntimeError("ROS action client failed; see out/ros_episode_action.log")
                            break
                else:
                    raise TimeoutError("No completed ROS episode within 60 s; check server, bridge and ROS_DOMAIN_ID")
            finally:
                if action is not None and action.poll() is None:
                    os.killpg(action.pid, signal.SIGTERM)
                    action.wait(timeout=5)
    keys = ("IDLE", "LIFT", "TRANSPORT", "DONE")
    sheet = np.vstack([
        np.hstack([stages[state] for state in keys[i:i + 2]]) for i in (0, 2)
    ])
    if not cv2.imwrite(str(output / "ros_episode.png"), sheet):
        raise RuntimeError("Could not save ROS episode contact sheet")
    animation = cv2.Animation()
    animation.frames = images
    animation.durations = [1000] * len(images)
    animation.durations[-1] = 2000
    animation.loop_count = 0
    if not cv2.imwriteanimation(str(output / "ros_episode.gif"), animation):
        raise RuntimeError("Could not save ROS episode animation")
    print(f"Wrote {output / 'ros_episode.png'} and ros_episode.gif from the real ROS action")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--wire", choices=[f"wire{i}" for i in range(7)], default="wire0")
    parser.add_argument("--crate", choices=["crate0", "crate1"], default="crate0")
    parser.add_argument("--camera-config", type=Path, default=ROOT / "config/cameras.yaml")
    capture(parser.parse_args())


if __name__ == "__main__":
    main()
