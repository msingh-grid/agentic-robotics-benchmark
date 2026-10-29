# Agentic Robotics Benchmark

A benchmark for AI agents that debug a **live** robot system (simulation, ROS 2, host), not just code. Four tasks, each a step up:

1. **Catch unreported failures:** spot pick-and-place episodes that went wrong even though every action reported success.
2. **Calibrate a camera:** find a camera's true mounting pose and prove it with picks.
3. **Find a resource bottleneck:** trace a robot symptom to a host cause (CPU, memory, network, DDS) and fix the cause.
4. **Investigate a timing bug:** prove the cause of phantom obstacles around the moving arm by experiment.

Brief: [`docs/Agentic Robotics Benchmark - Team Brief.pdf`](docs/Agentic%20Robotics%20Benchmark%20-%20Team%20Brief.pdf) · Plan: [`docs/roadmap.md`](docs/roadmap.md)

## Status

Phase 1 · S4: a headless MuJoCo cell with a ROS 2 telemetry bridge and a command-only pick/place stub. The UR5e + Robotiq 2F-85, seven rigid wires, two open crates and four RGB/depth cameras remain unchanged. Until the real Isaac Sim sandbox is available, we build a Mac-native stand-in (MuJoCo + ROS 2 Jazzy) with the same interfaces.

## Getting started

Python is managed with [uv](https://docs.astral.sh/uv/). ROS 2 isn't on PyPI, so it lives in a separate [pixi](https://pixi.sh) environment under `ros/`. Both environments install from committed lockfiles.

```bash
bash scripts/setup.sh
uv run pytest
```

Works natively on Apple Silicon Macs and on Linux x86_64.

The tests write a one-box render to `out/smoke.png` and exchange a ROS message within 60 seconds (the first ROS start on a fresh Mac is slow while macOS checks the new libraries; setup does that once up front). Linux tests try EGL, then OSMesa if EGL fails; macOS uses native CGL with Apple's software renderer when no accelerated pixel format is available. Linux setup installs only the three required system render libraries and needs root or passwordless sudo if they are missing.

On a Mac, regenerate and inspect the smoke image with `bash scripts/setup.sh && uv run pytest tests/test_env_smoke.py && open out/smoke.png`.

RViz is optional and is not used by the tests. Install it with `pixi install --locked --manifest-path ros/pixi.toml --environment viz`. CI installs both `default` and `viz` on both platforms.

## Cell preview

Run `uv run python scripts/render_cell.py` to write the four views to `out/cell.png`.
On a Mac, regenerate and inspect with `bash scripts/setup.sh && uv run python scripts/render_cell.py && open out/cell.png`.
Check the attached gripper, seven distinct wires, both open crates and the four camera views.
Linux uses EGL with an in-process OSMesa fallback; set `MUJOCO_GL=osmesa` to force software rendering.
No display is required. The same renderer serves the smoke test and `World.render()`.

`World` starts after two seconds of idle settling at simulation time zero. `step()` advances 2 ms.
Body and camera setters accept a position in metres and a quaternion in **wxyz** order.
`render("cam0")` returns RGB `uint8` (480×640×3) and optical-axis depth in metres `float32` (480×640), both with a top-left origin.
`seed(n)` resets a private NumPy generator; this step does not randomize the layout.
Robot assets are vendored with original licenses and a pinned upstream revision in `sandbox/assets/PROVENANCE.txt`.
Scripted picking remains S3 work; the S4 action does not move the arm.

## Sim server and ROS bridge (S4)

`bash scripts/setup.sh` also compiles the small `benchmark_interfaces/PickPlace` action
with native pixi compilers. Rebuild it after an interface or ROS environment change:
`pixi run --locked --manifest-path ros/pixi.toml build-interfaces`.
Generated files stay under ignored `ros/build/` and `ros/install/`.

Start from the repository root in two terminals with the same short, unique run directory
and an unused domain in 0–101. For example, in the first terminal:

```bash
RUN_DIR=$(mktemp -d /tmp/robotics-s4.XXXXXX)
printf '%s\n' "$RUN_DIR"
uv run python -m sandbox.server --run-dir "$RUN_DIR" --seed 42
```

In the second terminal, set `RUN_DIR` to the path printed above:

```bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
pixi run --locked --manifest-path ros/pixi.toml bridge --run-dir "$RUN_DIR"
```

The server owns `state.sock` (PUB, msgpack multipart sensor stream) and `commands.sock`
(REP, msgpack command dictionaries) inside that directory. Do not share a run directory
between servers. Stop both processes with Ctrl+C. The bridge never opens a private socket.
The uv environment does not import ROS packages; the pixi bridge does not import the sim.

Default rates: `/clock`, `/joint_states`, and `/tf` are 50 Hz; `/flow/state` and
`/diagnostics` are 1 Hz; each `/cameras/cam0`–`cam3` RGB image and depth cloud is 0.5 Hz.
`/tf_static` is transient-local, not periodic. Camera publishers use best-effort queues of
two messages. Other streams use their standard ROS QoS; `/clock` is best-effort.
The bridge defaults to a Fast DDS profile with a 16 MiB shared-memory segment:
the 512 KiB default loses full-resolution samples for best-effort readers, including
`ros2 topic hz`. Only the publishing process needs the larger segment; it retains UDP
for discovery/fallback. Operator-supplied Fast DDS profiles take precedence, and other
middleware ignores this profile. No system buffer or privileged tuning is required.
Full 640×480 rendering runs independently of physics, with a bounded four-frame queue;
overload drops frames rather than blocking commands. Camera stamps identify the copied
simulation state, not the later render completion time. These conservative camera rates
keep the existing full-resolution software-rendered scene usable headlessly.
The camera worker retains one native GL context and uploaded model across views and samples,
instead of rebuilding them for each frame on software-rendered Macs.

`config/cameras.yaml` is the small shared schema for the S3 integration:
`cameras.<name>.position` is world XYZ in metres and `quaternion` is unit **wxyz**.
It describes a configured MuJoCo/OpenGL mount frame (local -Z forward, +Y up).
The bridge publishes `world → camN → camN_optical`; the optical frame is +Z forward,
+X right, +Y down. It reads only this configuration for camera TF, never true extrinsics.
The defaults equal the S2 mounting poses, but changing this file must not move the sim's
cameras. Restart the bridge after a config edit. All public names still need confirmation
with the real stack (`TODO(pawel)` in the implementation).

Public packets contain only sim time, named robot joints/link transforms, rendered RGB
and depth with intrinsics, command flow state, and bridge status. No object poses,
camera extrinsics, contacts, segmentation labels, or placement results cross this channel.
Point clouds contain organized optical-frame XYZ in metres, derived only from depth.

In another terminal with the same ROS environment variables, inspect rates and send a goal:

```bash
pixi run --locked --manifest-path ros/pixi.toml ros-cli topic hz /joint_states
pixi run --locked --manifest-path ros/pixi.toml ros-cli topic hz /cameras/cam0/color/image_raw
pixi run --locked --manifest-path ros/pixi.toml ros-cli action send_goal /pick_place benchmark_interfaces/action/PickPlace '{object_id: wire0, crate_id: crate0}' --feedback
```

The goal has `object_id` and `crate_id`; the result has `success` and `message`;
feedback has `state`. **The server currently returns command-only stub success without
moving the arm.** Flow changes from `IDLE` to `DONE`, and diagnostics warns about the stub.
Unknown IDs abort, empty IDs are rejected, and unavailable commands abort after a bounded
timeout. Cancellation is not supported by the stub. Follow-up: wire the explicit server
TODO to S3's `PickPlaceFlow`; do not use action success to grade physical placement.

On a Mac, run `bash scripts/setup.sh && uv run pytest` or the focused
`uv run pytest tests/test_bridge.py -v -s`. This test leases a unique domain, starts the
real server and bridge, checks all stream rates (35% startup/load tolerance), the complete
robot/camera TF tree, optical cloud geometry and stamps, configured camera offsets,
action results, and sim disconnection. It tears down every process.
Use the terminal commands above to check rates manually; RViz is optional.
If RViz works, select fixed frame `world`, then add TF, Image and PointCloud2 displays.
A RobotModel display also needs a URDF/robot_description, which S4 does not add.
There is no scene or robot-motion change in this step; the cell preview command above
still regenerates the same image.

## Contributing

Read [`AGENTS.md`](AGENTS.md) first. It holds the rules for humans and coding agents alike.
