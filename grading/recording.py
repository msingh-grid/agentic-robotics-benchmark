"""Private physical preflight and grader-only JSONL measurements."""

import argparse
import json
import os
from pathlib import Path
import signal
import threading

from backends import create_ground_truth


def check_healthy(snapshot):
    results = [event for event in snapshot["events"] if event["kind"] == "placement_result"]
    if (
        snapshot["verification"] != {"status": "valid", "faults": [], "bridge": {}}
        or len(results) != 1 or not results[0]["inside"]
        or results[0]["object"] != "wire0" or results[0]["crate"] != "crate0"
        or any(event["kind"] == "drop" for event in snapshot["events"])
    ):
        raise RuntimeError("Healthy dry pick failed physical validation")


def capture(truth, output):
    stop = threading.Event()
    for signum in (signal.SIGTERM, signal.SIGINT):
        signal.signal(signum, lambda *_: stop.set())
    descriptor = os.open(output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(descriptor, "w") as stream:
        while True:
            stream.write(json.dumps(truth.snapshot()) + "\n")
            stream.flush()
            if stop.is_set():
                break
            stop.wait(0.1)
        os.fsync(stream.fileno())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("check", "capture"))
    parser.add_argument("--private-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    truth = create_ground_truth(args.private_dir)
    if args.operation == "check":
        check_healthy(truth.snapshot())
    else:
        if args.output is None:
            parser.error("capture requires --output")
        capture(truth, args.output)


if __name__ == "__main__":
    main()
