# Agentic Robotics Benchmark

A benchmark for AI agents that debug a **live** robot system (simulation, ROS 2, host), not just code. Four tasks, each a step up:

1. **Catch unreported failures:** spot pick-and-place episodes that went wrong even though every action reported success.
2. **Calibrate a camera:** find a camera's true mounting pose and prove it with picks.
3. **Find a resource bottleneck:** trace a robot symptom to a host cause (CPU, memory, network, DDS) and fix the cause.
4. **Investigate a timing bug:** prove the cause of phantom obstacles around the moving arm by experiment.

Brief: [`docs/Agentic Robotics Benchmark - Team Brief.pdf`](docs/Agentic%20Robotics%20Benchmark%20-%20Team%20Brief.pdf) · Plan: [`docs/roadmap.md`](docs/roadmap.md)

## Status

Phase 1 · S1: Python and ROS 2 environments with headless smoke tests. Until the real Isaac Sim sandbox is available, we build a Mac-native stand-in (MuJoCo + ROS 2 Jazzy) with the same interfaces.

## Getting started

Python is managed with [uv](https://docs.astral.sh/uv/). ROS 2 isn't on PyPI, so it lives in a separate [pixi](https://pixi.sh) environment under `ros/`. Both environments install from committed lockfiles.

```bash
bash scripts/setup.sh
uv run pytest
```

Works natively on Apple Silicon Macs and on Linux x86_64.

The tests write a one-box render to `out/smoke.png` and exchange a ROS message within 20 seconds. Linux tests try EGL, then OSMesa if EGL fails; macOS uses native CGL with Apple's software renderer when no accelerated pixel format is available. Linux setup installs only the three required system render libraries and needs root or passwordless sudo if they are missing.

On a Mac, regenerate and inspect the smoke image with `bash scripts/setup.sh && uv run pytest tests/test_env_smoke.py && open out/smoke.png`.

RViz is optional and is not used by the tests. Install it with `pixi install --locked --manifest-path ros/pixi.toml --environment viz`. CI installs both `default` and `viz` on both platforms.

## Contributing

Read [`AGENTS.md`](AGENTS.md) first. It holds the rules for humans and coding agents alike.
