---
name: ros2-pixi
description: How to install, run and test ROS 2 Jazzy in this repo through pixi (RoboStack), on macOS arm64 and Linux, headless. Read before adding ROS packages, writing nodes, or launching anything ROS.
---

# ROS 2 through pixi

ROS 2 Jazzy comes from the **RoboStack** conda channel, installed by pixi. There is no system ROS.

## Do

- Add ROS packages in `pixi.toml` under `[dependencies]` with the `ros-jazzy-` prefix, e.g. `ros-jazzy-tf2-ros-py = "*"`. Channels: `robostack-jazzy`, then `conda-forge`. Platforms: `osx-arm64`, `linux-64`.
- Run everything through pixi: `pixi run <task>` or `pixi run python ...`. The ROS environment is activated automatically; never `source /opt/ros/...`.
- Write nodes in Python with `rclpy`. Plain Python modules launched by our own scripts are preferred over `colcon` packages until a step needs a built package.
- Isolate each test or attempt with its own `ROS_DOMAIN_ID` (0–101) and set `ROS_LOCALHOST_ONLY=1` (or `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`) so parallel runs and other machines don't see each other.
- Use simulated time: the sim publishes `/clock`, every node sets `use_sim_time=True`.
- In tests, start nodes as subprocesses, wait for readiness by checking topics/TF (with a timeout), and always kill them in teardown.
- Before adding a package, check it exists for **both** platforms: `pixi search -c robostack-jazzy ros-jazzy-<name> -p osx-arm64` and `-p linux-64`.

## Don't

- No `apt`, `brew`, `sudo`, `rosdep install`, or `pip install` outside pixi.
- Don't make RViz2 (`ros-jazzy-rviz2`) a required dependency; it has open startup bugs on Apple Silicon. It lives in an optional `viz` feature and is never used by tests.
- Don't depend on a display. For images use MuJoCo offscreen rendering; for "RViz-like" views render point clouds or voxels with matplotlib/Open3D.

## Rendering backends

| Platform | `MUJOCO_GL` |
|---|---|
| Linux runner / CI | `egl` (fallback `osmesa`) |
| macOS | leave unset (uses `cgl`) |

Set `MUJOCO_GL` inside pixi tasks via `[target.linux-64.activation.env]`, not in code.
