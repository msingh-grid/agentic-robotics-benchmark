# Agentic Robotics Benchmark

A benchmark for AI agents that debug a **live** robot system (simulation, ROS 2, host), not just code. Four tasks, each a step up:

1. **Catch unreported failures:** spot pick-and-place episodes that went wrong even though every action reported success.
2. **Calibrate a camera:** find a camera's true mounting pose and prove it with picks.
3. **Find a resource bottleneck:** trace a robot symptom to a host cause (CPU, memory, network, DDS) and fix the cause.
4. **Investigate a timing bug:** prove the cause of phantom obstacles around the moving arm by experiment.

Brief: [`docs/Agentic Robotics Benchmark - Team Brief.pdf`](docs/Agentic%20Robotics%20Benchmark%20-%20Team%20Brief.pdf) · Plan: [`docs/roadmap.md`](docs/roadmap.md)

## Status

Phase 1 · S4: a headless MuJoCo cell with ROS 2 telemetry and real scripted pick/place actions. The UR5e + Robotiq 2F-85, seven rigid wires, two open crates and four RGB/depth cameras use the S3 flow. Until the real Isaac Sim sandbox is available, we build a Mac-native stand-in (MuJoCo + ROS 2 Jazzy) with the same interfaces.

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
The S4 ROS action drives the S3 scripted pick-and-place flow.

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

`config/cameras.yaml` is shared by S3 perception and S4 camera TF:
`cameras.<name>.position` is world XYZ in metres and `quaternion` is unit **wxyz**.
It describes a configured MuJoCo/OpenGL mount frame (local -Z forward, +Y up).
The bridge publishes `world → camN → camN_optical`; the optical frame is +Z forward,
+X right, +Y down. It reads only this configuration for camera TF, never true extrinsics.
The defaults equal the S2 mounting poses, but changing this file must not move the sim's
cameras. Restart both server and bridge after a config edit. Pass the same `--camera-config`
path to both when using a custom file. All public names still need confirmation
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
feedback has `state`. The server calls `PickPlaceFlow.start()` for each goal and advances
the flow once per 2 ms physics tick. Results wait for `flow.result`, which means commands
completed, never verified placement. State callbacks drive public flow state and action
feedback through `APPROACH`, `GRASP`, `LIFT`, `TRANSPORT`, `PLACE`, `RELEASE`, and `DONE`.
Idle ticks refresh gravity compensation after a pick so the arm holds its final target.
Unknown IDs abort; empty or concurrent goals are rejected. A missing server fails within
2 seconds; an accepted command has a 30-second response timeout for the 15.5-second flow.
Cancellation is not supported, and stopping the client does not stop the active sim flow.
The REP command reply is deferred until completion while sensor publication continues.
Do not use action success to grade physical placement.

On a Mac, run `bash scripts/setup.sh && uv run pytest` or the focused
`uv run pytest tests/test_bridge.py -v -s`. This test leases a unique domain, starts the
real server and bridge, checks all stream rates (35% startup/load tolerance), the complete
robot/camera TF tree, optical cloud geometry and stamps, configured camera offsets,
action results, and sim disconnection. It tears down every process.
Use the terminal commands above to check rates manually; RViz is optional.
If RViz works, select fixed frame `world`, then add TF, Image and PointCloud2 displays.
A RobotModel display also needs a URDF/robot_description, which S4 does not add.
The cell geometry is unchanged; the ROS action now moves the arm through the existing flow.

To regenerate proof from the actual ROS action on a Mac, start a fresh server above with
`--seed 0` and its bridge. Use the same `RUN_DIR` and ROS variables, then run:

```bash
uv run python scripts/render_ros_episode.py --run-dir "$RUN_DIR" --wire wire0 --crate crate0
open out/ros_episode.png out/ros_episode.gif
```

The helper sends a real ROS goal and captures only public camera packets, paired to
their simulation-time flow states. The contact sheet shows idle, lift, transport and done
in cam0/cam2; the GIF shows all four cameras. Check that the red wire lifts between the
pads, travels to crate0, and is released. Command output is in `out/ros_episode_action.log`.

## Scripted pick and place

`PickPlaceFlow(World(seed=n)).run("wire0", "crate0")` runs a pick synchronously.
Import `PickPlaceFlow` from `sandbox.sim.pick_place` and `World` from `sandbox.sim.world`.
The fingers grip through physics contacts; no objects are attached or teleported.
The result means **commands completed**, not that a grasp or placement succeeded.
Only tests inspect the physical outcome.

For an event loop, call `flow.start(wire, crate)`, then `flow.step()` until `flow.result is not None`.
Each `step()` advances physics once (2 ms); do not also call `World.step()` for that tick.
The `on_state` callback receives a `FlowState` at initialization and each transition; `flow.state` is also readable.
The flow has no ROS dependency. The S4 server/bridge drives the same event-loop API.

`Perception(world).estimate_pose(name)` uses the world's explicitly seeded generator.
It adds 0.5 mm position noise and 0.002 rad yaw noise in camera coordinates, then transforms through `config/cameras.yaml`.
Those configured extrinsics initially match the real cameras; changing them biases estimates without moving the cameras.

Run `uv run python scripts/render_episode.py --seed 0 --wire wire0 --crate crate0` for a two-view contact sheet (`out/episode.png`) and a 2×-speed GIF (`out/episode.gif`).
On a Mac, regenerate with `bash scripts/setup.sh && uv run python scripts/render_episode.py --seed 0 --wire wire0 --crate crate0 && open out/episode.png out/episode.gif`.
Check that the red wire lifts between the pads, travels above the crate walls and settles inside crate0 after release.
`uv run pytest tests/test_ik.py tests/test_perception.py tests/test_pick_place.py` checks downward reach, configured calibration, 20 seeded placements, determinism and command-only results.

## Contributing

Read [`AGENTS.md`](AGENTS.md) first. It holds the rules for humans and coding agents alike.
