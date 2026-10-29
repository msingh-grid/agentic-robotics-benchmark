# AGENTS.md

Rules for every coding agent working in this repo (Warp Factory agents, Claude Code, Codex).

## What this project is

A benchmark that tests whether an AI agent can debug a **live, running robot system**: watch cameras and ROS topics, read host metrics, run experiments, fix the cause. See `docs/Agentic Robotics Benchmark - Team Brief.pdf` for the brief and `docs/roadmap.md` for the phased plan. Work one roadmap step per task and per PR.

Until the real Isaac Sim sandbox arrives, we build a **stand-in sandbox** (MuJoCo + ROS 2 Jazzy) that exposes the same five interfaces. Everything above the sandbox must work unchanged when the real one is plugged in.

## Golden rules

1. **Ground truth never reaches the benchmarked agent.** Code in `agent/` must never import `sandbox/` or read ground-truth files or sockets. Only `grading/` may read ground truth. If you are unsure whether something leaks truth, assume it does and ask.
2. **Talk through the interfaces.** `harness/`, `grading/` and `agent/` import only `interfaces/` (and `backends/` via the factory function). Never reach into `sandbox/` directly.
3. **Mac and Linux, both native.** Supported platforms: `osx-arm64` (developer Macs) and `linux-64` (factory runners, CI). **Python dependencies go through uv** (`pyproject.toml` + `uv.lock`). **pixi is only for ROS 2** (`ros/pixi.toml`), because ROS 2 isn't on PyPI. uv-side code never imports `rclpy`. Never use `apt`, `brew`, `sudo`, `/opt/ros`, `pip install`, or system Python. No code path may work on only one platform without a fallback for the other.
4. **Headless first.** Everything must run without a display. Rendering uses offscreen backends (`MUJOCO_GL=egl` or `osmesa` on Linux, `cgl` on macOS). RViz is optional and only for humans.
5. **Reproducible.** Every fault and every scenario is driven by an explicit seed. No unseeded randomness anywhere in `sandbox/`, `harness/` or `grading/`.
6. **Healthy controls are first-class.** Every scenario set mixes in runs with nothing wrong. Graders must penalise an agent that reports a problem on a healthy run.
7. **Grade by physics, not by claims.** Graders use ground truth and measurements, never the agent's own report or ROS action results.

## How to work

- Set up the environment: `bash scripts/setup.sh` (installs uv, and pixi once `ros/pixi.toml` exists, then both environments; safe to rerun).
- Run tests before opening a PR: `uv run pytest`. A PR with failing tests is not done.
- Add Python dependencies with `uv add` (dev tools: `uv add --dev`) and commit `uv.lock`. Add ROS packages only in `ros/pixi.toml` and commit `ros/pixi.lock`.
- Read the matching skill before working in these areas:
  - dependencies, uv/pixi split, ROS 2, running nodes: `.agents/skills/ros2-pixi/SKILL.md`
  - sandbox, interfaces, backends: `.agents/skills/sandbox-contract/SKILL.md`
  - faults, scenarios, graders, results: `.agents/skills/grading-rules/SKILL.md`
- Visual output (camera frames, scene renders) goes to `out/` (git-ignored). In the PR description, say which images a human should look at and how to regenerate them; a human checks visuals on a Mac before merging.
- Keep the PR to the one roadmap step it implements. Note anything out of scope as follow-up in the PR description instead of doing it.

## Commits and PRs

- Plain, imperative commit messages ("Add MuJoCo cell scene").
- **No `Co-Authored-By` trailers and no AI or tool attribution lines** in commit messages.
- PR description: which roadmap step, what changed, how it was tested (exact commands), what a human should check on Mac.

## Repo layout (target)

```
docs/          brief, roadmap
sandbox/       stand-in sandbox: MuJoCo scene, sim, fake perception, faults, ground truth (uv env)
ros/           ROS 2 env (pixi) and the ROS bridge node
config/        configured (possibly wrong) system config, e.g. camera poses
interfaces/    the five interfaces: Observe, Act, GroundTruth, Faults, Lifecycle
backends/      mac_mujoco (stand-in), isaac_docker (real sandbox, later)
harness/       runs isolated, seeded attempts; scenarios/
grading/       one grader per task; report tables
agent/         the benchmarked agent: MCP tool server, agent loop, tool profiles
tests/         headless tests
scripts/       setup and helper scripts
```
