"""Private IPC clients; no in-process dependency on the stand-in simulator."""

from pathlib import Path

import msgpack
import zmq


def _request(private_dir, endpoint, request):
    with zmq.Context() as context:
        with context.socket(zmq.REQ) as socket:
            socket.setsockopt(zmq.LINGER, 0)
            socket.setsockopt(zmq.SNDTIMEO, 5000)
            socket.connect(f"ipc://{Path(private_dir).resolve() / endpoint}")
            socket.send(msgpack.packb(request, use_bin_type=True))
            if not socket.poll(5000):
                raise TimeoutError("Private sandbox request timed out")
            result = msgpack.unpackb(socket.recv(), raw=False)
    if "error" in result:
        raise ValueError(result["error"])
    return result


class GroundTruth:
    def __init__(self, private_dir):
        self.private_dir = private_dir

    def snapshot(self):
        return _request(self.private_dir, "ground_truth.sock", {"op": "snapshot"})


class Faults:
    def __init__(self, private_dir):
        self.private_dir = private_dir

    def configure(self, scenario, seed):
        _request(self.private_dir, "faults.sock", {
            "op": "configure", "scenario": scenario, "seed": seed,
        })
