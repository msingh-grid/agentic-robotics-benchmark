# Agentic Robotics Benchmark

A benchmark for AI agents that debug a **live** robot system (simulation, ROS 2, host), not just code. Four tasks, each a step up:

1. **Catch unreported failures:** spot pick-and-place episodes that went wrong even though every action reported success.
2. **Calibrate a camera:** find a camera's true mounting pose and prove it with picks.
3. **Find a resource bottleneck:** trace a robot symptom to a host cause (CPU, memory, network, DDS) and fix the cause.
4. **Investigate a timing bug:** prove the cause of phantom obstacles around the moving arm by experiment.

Brief: [`docs/Agentic Robotics Benchmark - Team Brief.pdf`](docs/Agentic%20Robotics%20Benchmark%20-%20Team%20Brief.pdf) · Plan: [`docs/roadmap.md`](docs/roadmap.md)

## Status

Phase 0: repo and factory setup. Until the real Isaac Sim sandbox is available, we build a Mac-native stand-in (MuJoCo + ROS 2 Jazzy via pixi) with the same interfaces.

## Getting started

```bash
bash scripts/setup.sh   # installs pixi and the environment
pixi run test
```

Works natively on Apple Silicon Macs and on Linux x86_64.

## Contributing

Read [`AGENTS.md`](AGENTS.md) first. It holds the rules for humans and coding agents alike.
