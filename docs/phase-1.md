# Phase 1 · Stand-in sandbox

Approved plan for Phase 1. One step = one factory task = one PR. Read AGENTS.md first.

## Decisions

- Robot: **UR5e + Robotiq 2F-85** from MuJoCo Menagerie, vendored under `sandbox/assets/` with license files.
- Sim (uv) ↔ ROS bridge (pixi): **ZeroMQ + msgpack** over `ipc://` sockets. Ground truth on a separate private socket, never on ROS.
- Linux rendering: system render libraries in CI and `scripts/setup.sh` only (the single apt exception).
- Wires: **rigid capsules** for now.
- Names of topics, frames, cameras, actions: guessed and marked `TODO(pawel)` until the real list arrives.
- Visual review: images attached to each PR **and** a command to regenerate them on a Mac.

## How it runs

```
MuJoCo sim (uv) ──ZeroMQ──► ROS bridge (pixi) ──ROS 2──► (agent, later)
      │
      └──private ZeroMQ──► grader (ground truth)
```

Order: S1 → S2 → (S3 ∥ S4) → S5 → S6.

## S1 · Environments: uv for Python, pixi for ROS 2

Add MuJoCo and OpenCV to the uv project and create the ROS 2 environment in `ros/`. A smoke test proves both work on Mac and Linux.

- `uv add mujoco numpy opencv-python-headless pyzmq msgpack pyyaml psutil`.
- `ros/pixi.toml`: channels `robostack-jazzy`, `conda-forge`; platforms `osx-arm64`, `linux-64`; deps `ros-jazzy-ros-base`, `ros-jazzy-rclpy`, `ros-jazzy-tf2-ros-py`, `ros-jazzy-rosbag2-storage-mcap`, `ros-jazzy-sensor-msgs-py`, `pyzmq`, `msgpack-python`. Optional feature `viz` with `ros-jazzy-rviz2`. All verified available for osx-arm64 and linux-64.
- CI: add `prefix-dev/setup-pixi` (manifest `ros/pixi.toml`) next to setup-uv; Linux job installs the render library per decision; `MUJOCO_GL=egl` (fallback `osmesa`) on Linux only.
- `tests/test_env_smoke.py`: render one frame of a 1-box MuJoCo scene to `out/smoke.png`; start a ROS talker and listener via `pixi run --manifest-path ros/pixi.toml` with a unique `ROS_DOMAIN_ID` and assert one message arrives within 20 s.

**Done when:** `bash scripts/setup.sh && uv run pytest` passes on your Mac; both CI jobs green.

## S2 · The cell scene in MuJoCo

**Needs:** S1  
Build the pick-and-place cell: table, arm with gripper, 4 fixed cameras, 7 wires, 2 crates. Add a small `World` class to step physics and render cameras.

- Vendor the Menagerie model files under `sandbox/assets/` (with their license files) instead of downloading at runtime.
- `sandbox/scenes/cell.xml` includes arm + gripper; wires as rigid capsules with distinct colors; crates as open boxes; 4 cameras `cam0`–`cam3` (RGB + depth), names marked `TODO(pawel)`.
- `sandbox/sim/world.py` → `World`: `step()`, `render(cam) -> rgb, depth`, `body_pose(name)`, `set_body_pose()`, `set_camera_pose()`, `seed(n)`.
- `scripts/render_cell.py` writes a 2×2 contact sheet of the 4 views to `out/cell.png`.

**Done when:** Test: each wire and crate visible (non-zero pixels by segmentation) in at least one camera; scene stable for 10 s of sim with nothing moving. **You** look at `out/cell.png` on your Mac.

## S3 · Scripted pick-and-place that can 'lie'

**Needs:** S2  
The robot picks a named wire and drops it into a crate with scripted moves. It reports success from its own commands, like the real system, even if the wire is not where it should be.

- `sandbox/sim/ik.py` → damped-least-squares IK on the MuJoCo model.
- `sandbox/sim/pick_place.py` → `PickPlaceFlow` state machine IDLE→APPROACH→GRASP→LIFT→TRANSPORT→PLACE→RELEASE→DONE, publishes current state; result = commands completed, never checks the outcome.
- `sandbox/sim/perception.py` → `estimate_pose(obj)` = true pose + seeded noise, transformed through the **configured** camera pose from `config/cameras.yaml` (initially equal to the truth).
- `config/cameras.yaml` holds the configured camera poses.

**Done when:** 20 seeded episodes, no faults: ≥ 18 wires end inside the target crate (checked from sim state); same seed gives the same result twice. Short GIF/contact sheet of one episode for you to review.

## S4 · Sim server and ROS 2 bridge

**Needs:** S2  
Run the sim as a server, and a bridge in the ROS environment that publishes everything the real stack would: joints, frames, camera images, point clouds, flow state, plus a pick-and-place action.

- `sandbox/server.py` (uv): runs `World`, serves state over ZeroMQ PUB, commands over REP; sim-time clock.
- `ros/bridge_node.py` (pixi): `/clock`, `/joint_states`, `/tf` + `/tf_static` (camera frames from `config/cameras.yaml`), `/cameras/<n>/color/image_raw`, `/cameras/<n>/depth/points`, `/flow/state`, `/diagnostics`; action server `/pick_place` (goal: object id, crate id; result: success from commands only).
- Can be built in parallel with S3: the action forwards to the server, which wires it to `PickPlaceFlow` once S3 lands (stub until then).
- Test launches server + bridge with a unique domain, checks every topic publishes at its expected rate and the TF tree is complete.

**Done when:** Headless test green on Linux and Mac; `ros2 topic hz` shows expected rates. If RViz starts on your Mac, it shows the robot, camera frames and clouds.

## S5 · Private ground truth and seeded faults

**Needs:** S3, S4  
Two side doors the agent may never use: one for the grader to read what really happened, one for the harness to break things on purpose, reproducibly.

- `sandbox/sim/ground_truth.py`: separate ZeroMQ REP socket (file mode 0600) returning true object poses, true camera poses, gripper contacts, and an event log (grasp, release, drop, placement result with timestamps).
- `sandbox/sim/faults.py`: faults from `(scenario yaml, seed)`: `wrong_object`, `drop_in_transit(at_state, t)`, `place_outside_crate(offset_m)`, `camera_offset(cam, xyz_m, rpy_deg)`, `topic_delay(topic, ms)`, `topic_drop(topic, rate)`. Delay/drop are applied in the bridge.
- Each fault verified after injection from ground truth; a fault that silently fails marks the run invalid.

**Done when:** Test runs every fault once and confirms from ground truth that it happened, while action results still say success; a no-fault run stays healthy; same seed → same event log.

## S6 · One-command bring-up, readiness, recording

**Needs:** S5  
One command starts sim and bridge, waits until the system is truly ready, and can record every episode for replay.

- `uv run bringup [--scenario f.yaml --seed n]`: starts server + bridge, warm-up period, readiness = all topics publishing, TF complete, one healthy dry pick; prints a ready line; clean shutdown on Ctrl+C.
- `uv run record --episodes n`: MCAP bag per episode via `ros2 bag record` in the pixi env; ground truth saved next to it in a grader-only file.
- `docs/sandbox.md`: how to start, stop, record and replay.

**Done when:** Cold start to ready in under 60 s on your Mac; a recorded episode replays with `ros2 bag play`; Phase 1 demo: you start the cell, watch an episode, break it with a fault, replay it.
