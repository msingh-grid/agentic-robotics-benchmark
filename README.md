# Agentic Robotics Benchmark

A benchmark for AI agents that debug a **live** robot system (simulation, ROS 2, host), not just code. Four tasks, each a step up:

1. **Catch unreported failures:** spot pick-and-place episodes that went wrong even though every action reported success.
2. **Calibrate a camera:** find a camera's true mounting pose and prove it with picks.
3. **Find a resource bottleneck:** trace a robot symptom to a host cause (CPU, memory, network, DDS) and fix the cause.
4. **Investigate a timing bug:** prove the cause of phantom obstacles around the moving arm by experiment.

Brief: [`docs/Agentic Robotics Benchmark - Team Brief.pdf`](docs/Agentic%20Robotics%20Benchmark%20-%20Team%20Brief.pdf) · Plan: [`docs/roadmap.md`](docs/roadmap.md)

## Status

Phase 1 · S3: a headless MuJoCo cell with seeded, scripted wire picking using UR5e + Robotiq 2F-85, two open crates and four RGB/depth cameras. Until the real Isaac Sim sandbox is available, we build a Mac-native stand-in (MuJoCo + ROS 2 Jazzy) with the same interfaces.

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
ROS publication remains a later Phase 1 step.

## Scripted pick and place

`PickPlaceFlow(World(seed=n)).run("wire0", "crate0")` runs a pick synchronously.
Import `PickPlaceFlow` from `sandbox.sim.pick_place` and `World` from `sandbox.sim.world`.
The fingers grip through physics contacts; no objects are attached or teleported.
The result means **commands completed**, not that a grasp or placement succeeded.
Only tests inspect the physical outcome.

For an event loop, call `flow.start(wire, crate)`, then `flow.step()` until `flow.result is not None`.
Each `step()` advances physics once (2 ms); do not also call `World.step()` for that tick.
The `on_state` callback receives a `FlowState` at initialization and each transition; `flow.state` is also readable.
The flow has no ROS dependency. The server/bridge integration belongs to S4.

`Perception(world).estimate_pose(name)` uses the world's explicitly seeded generator.
It adds 0.5 mm position noise and 0.002 rad yaw noise in camera coordinates, then transforms through `config/cameras.yaml`.
Those configured extrinsics initially match the real cameras; changing them biases estimates without moving the cameras.

Run `uv run python scripts/render_episode.py --seed 0 --wire wire0 --crate crate0` for a two-view contact sheet (`out/episode.png`) and a 2×-speed GIF (`out/episode.gif`).
On a Mac, regenerate with `bash scripts/setup.sh && uv run python scripts/render_episode.py --seed 0 --wire wire0 --crate crate0 && open out/episode.png out/episode.gif`.
Check that the red wire lifts between the pads, travels above the crate walls and settles inside crate0 after release.
`uv run pytest tests/test_ik.py tests/test_perception.py tests/test_pick_place.py` checks downward reach, configured calibration, 20 seeded placements, determinism and command-only results.

## Contributing

Read [`AGENTS.md`](AGENTS.md) first. It holds the rules for humans and coding agents alike.
