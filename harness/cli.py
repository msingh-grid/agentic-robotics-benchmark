"""Start a ready sandbox or record isolated, replayable episodes."""

import argparse
from datetime import datetime, timezone
from pathlib import Path
import signal
import sys
import threading
import time

import yaml

from backends import create_lifecycle


def arguments(recording):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=Path)
    parser.add_argument("--seed", type=int, default=0)
    if recording:
        parser.add_argument("--episodes", type=int, required=True)
        parser.add_argument(
            "--output", type=Path,
            default=Path("out") / datetime.now(timezone.utc).strftime("recording-%Y%m%dT%H%M%S%fZ"),
        )
    args = parser.parse_args()
    if recording and args.episodes < 1:
        parser.error("--episodes must be positive")
    return args


def run(recording):
    args = arguments(recording)
    scenario = yaml.safe_load(args.scenario.read_text()) if args.scenario else {"faults": []}
    stop = threading.Event()

    def interrupt(*_):
        stop.set()
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, interrupt)
    try:
        for episode in range(args.episodes if recording else 1):
            lifecycle = create_lifecycle()
            started = time.monotonic()
            try:
                lifecycle.start(args.seed + episode)
                lifecycle.wait_ready(scenario)
                print(
                    f"READY ROS_DOMAIN_ID={lifecycle.ros_domain} "
                    f"elapsed={time.monotonic() - started:.2f}s",
                    flush=True,
                )
                if recording:
                    output = args.output / f"episode-{episode + 1:04d}"
                    lifecycle.record_episode(output)
                    print(f"RECORDED {output.resolve()}", flush=True)
                else:
                    while not stop.wait(0.1):
                        lifecycle.check()
            finally:
                lifecycle.stop()
    except KeyboardInterrupt:
        return
    except (OSError, RuntimeError, TimeoutError, ValueError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)


def bringup():
    run(recording=False)


def record():
    run(recording=True)
