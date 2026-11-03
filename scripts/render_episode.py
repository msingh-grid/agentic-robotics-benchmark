"""Render one real physics episode to a contact sheet and a short GIF."""

import argparse
from pathlib import Path
import sys
import tempfile

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sandbox.server import SimServer
from sandbox.sim.faults import load_scenario
from sandbox.sim.pick_place import FlowState


# TODO(pawel): confirm camera names.
CAMERAS = ("cam0", "cam2")


def frame(world, state, wire, crate):
    views = []
    for camera in CAMERAS:
        rgb, _ = world.render(camera)
        views.append(cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), (320, 240)))
    image = np.hstack(views)
    cv2.rectangle(image, (0, 0), (640, 43), (25, 25, 25), -1)
    cv2.putText(
        image, f"{state}  t={world.data.time:.2f}s  {wire} -> {crate}",
        (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
    )
    cv2.putText(
        image, f"{CAMERAS[0]} / {CAMERAS[1]} | physics replay at 2x speed",
        (8, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (210, 210, 210), 1,
    )
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--wire", default="wire0")
    parser.add_argument("--crate", default="crate0")
    parser.add_argument("--scenario", type=Path)
    parser.add_argument("--output-prefix", default="episode")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="episode-", dir="/tmp") as directory:
        server = SimServer(
            directory, args.seed,
            scenario=load_scenario(args.scenario) if args.scenario else None,
        )
        try:
            render_episode(server, args)
        finally:
            server.state_socket.close(linger=0)
            server.command_socket.close(linger=0)
            server.context.term()


def render_episode(server, args):
    world, flow = server.world, server.flow
    images = [frame(world, flow.state, args.wire, args.crate)]
    snapshots = {FlowState.IDLE: images[0]}
    result = server.command({
        "op": "pick_place", "object_id": args.wire, "crate_id": args.crate,
    })
    if result is not None:
        raise ValueError(result["message"])
    sample_steps = round(0.25 / world.model.opt.timestep)
    tick = 0
    while flow.result is None:
        state = flow.state
        server.step()
        tick += 1
        if tick % sample_steps == 0 or flow.state == FlowState.DONE:
            image = frame(world, state, args.wire, args.crate)
            images.append(image)
            if state != FlowState.RELEASE or state not in snapshots:
                snapshots[state] = image
    final = frame(world, FlowState.DONE, args.wire, args.crate)
    snapshots[FlowState.DONE] = final
    images.append(final)
    tiles = [snapshots[state] for state in FlowState]
    sheet = np.vstack([np.hstack(tiles[i:i + 2]) for i in range(0, 8, 2)])
    output = Path(__file__).resolve().parents[1] / "out"
    output.mkdir(exist_ok=True)
    prefix = Path(args.output_prefix).name
    if not cv2.imwrite(str(output / f"{prefix}.png"), sheet):
        raise RuntimeError("Could not write episode contact sheet")
    animation = cv2.Animation()
    animation.frames = images
    animation.durations = [125] * len(images)
    animation.durations[-1] = 1500
    animation.loop_count = 0
    if not cv2.imwriteanimation(str(output / f"{prefix}.gif"), animation):
        raise RuntimeError("Could not write episode GIF")
    print(f"Command success: {flow.result}; verification: {server.faults.verification(server.truth)}")
    print(f"Wrote {output / f'{prefix}.png'} and {output / f'{prefix}.gif'}")


if __name__ == "__main__":
    main()
