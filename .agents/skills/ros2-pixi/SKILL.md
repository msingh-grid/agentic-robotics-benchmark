---
name: ros2-pixi
description: How Python (uv) and ROS 2 Jazzy (pixi, ROS only) are split in this repo, and how to install, run and test ROS on macOS arm64 and Linux, headless. Read before adding any dependency, writing ROS nodes, or launching anything ROS.
---

# Python with uv, ROS 2 with pixi

ROS 2 is not on PyPI, so uv can't install it. On a Mac the only native ROS 2 is the **RoboStack** conda build, installed by pixi. So the repo has two environments:

| Environment | Manifest | Holds | Run with |
|---|---|---|---|
| Python (main) | `pyproject.toml` + `uv.lock` | sim (MuJoCo), harness, grading, agent, tests | `uv run ...` |
| ROS | `ros/pixi.toml` + `ros/pixi.lock` | ROS 2 Jazzy, the ROS bridge node, `ros2` CLI | `pixi run --manifest-path ros/pixi.toml ...` |

## Rules

- Python dependencies go in `pyproject.toml` (`uv add <pkg>`, dev tools with `uv add --dev`). Commit `uv.lock`. Never add Python packages to the ROS env unless the bridge itself needs them.
- Code in the uv environment **never imports `rclpy`** or any ROS package. It reaches ROS through the bridge (local IPC) or by running `ros2` CLI commands in the ROS env as subprocesses.
- Only the bridge and ROS helper nodes live under `ros/` and run in the pixi env.
- ROS packages use the `ros-jazzy-` prefix from the `robostack-jazzy` channel, then `conda-forge`. Platforms: `osx-arm64`, `linux-64`. Before adding one, check both: `pixi search -c robostack-jazzy ros-jazzy-<name> -p osx-arm64` and `-p linux-64`.
- Never `source /opt/ros/...`, `apt`, `brew`, `sudo`, `rosdep install`, or `pip install` outside uv.

## Running ROS

- Write nodes in Python with `rclpy`, as plain modules started by our scripts; no `colcon` packages until a step needs one.
- Isolate each test or attempt with its own `ROS_DOMAIN_ID` (0–101) and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, so parallel runs don't see each other.
- Simulated time: the sim drives `/clock`, every node sets `use_sim_time=True`.
- In tests, start ROS processes as subprocesses, wait for readiness (topics/TF, with a timeout), and always kill them in teardown.
- RViz2 (`ros-jazzy-rviz2`) is never required: it has open startup bugs on Apple Silicon. Keep it in an optional pixi feature; tests never use it.

## Rendering backends

| Platform | `MUJOCO_GL` |
|---|---|
| Linux runner / CI | `egl` (fallback `osmesa`) |
| macOS | leave unset (uses `cgl`) |

Set it in test fixtures or the CI workflow environment, not inside library code.
