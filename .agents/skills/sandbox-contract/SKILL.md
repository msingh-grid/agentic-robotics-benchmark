---
name: sandbox-contract
description: The five interfaces between the sandbox and our code (harness, grading, agent), and the rules for backends. Read before touching sandbox/, interfaces/, backends/, harness/ or agent/.
---

# The sandbox contract

The real sandbox (Isaac Sim + platform stack, provided later as a Docker image) and our Mac stand-in (MuJoCo + ROS 2) must look the same to our code. That sameness is defined in `interfaces/`.

## The five interfaces

| Interface | What it carries | Who may use it |
|---|---|---|
| `Observe` | ROS topics, TF, camera images, RViz-like renders, logs, host metrics | agent, harness |
| `Act` | shell commands, config edits, ROS actions (e.g. `/pick_place`) | agent, harness |
| `GroundTruth` | true object and camera poses, what really happened | **grading only** |
| `Faults` | inject a seeded fault through config or sim parameters | **harness only** |
| `Lifecycle` | bring up, wait until ready (incl. warm-up), tear down | harness only |

The backend is chosen by the `BENCH_BACKEND` env var (`mac_mujoco` now, `isaac_docker` later) through one factory function in `backends/`.

## Rules

- `agent/` gets only `Observe` and `Act`. It must not be able to discover ground truth, not even by listing ROS topics: ground truth is served over a private channel (socket or file per attempt), never a ROS topic.
- The configured system can be **wrong on purpose** (e.g. `config/cameras.yaml` holds a camera pose that differs from the true one). The stand-in's fake perception and TF must use the *configured* values, exactly like the real stack would. Only `GroundTruth` knows the true values.
- Action results report success from commands, not from outcomes. Do not "fix" this in the sandbox; Task 1 depends on it.
- New sandbox capabilities go behind an interface method first, then into `backends/mac_mujoco.py`, with a `NotImplementedError` stub plus a TODO in `backends/isaac_docker.py`.
- Keep ROS topic and frame names close to the real stack's. When unsure, leave a `# TODO(pawel): confirm name` comment instead of inventing a convention.
- A lint test enforces the import boundaries; keep it passing.
