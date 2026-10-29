# Roadmap

Phased plan for the POC. One step = one factory task = one PR. Each step lists what it depends on and when it counts as done.

Open choices (current defaults, may change): robot = UR5e + Robotiq 2F-85 from MuJoCo Menagerie; motion = scripted moves first, MoveIt added for Task 4; benchmarked agent = our own Claude Agent SDK agent + MCP tool server (Warp Factories builds the benchmark, it is not the agent under test).

## P0 · Repo and Warp Factories setup  (Phase 0)

**Done by you, by hand.** Create the GitHub repo, write the rules file and skills every factory agent reads, connect the factory, and make the Linux runner able to install the project.

- Repo layout as in the code map; `AGENTS.md` with the golden rules: ground truth never reaches the agent; everything must pass `pixi run test-headless`; no Mac-only code paths without a Linux fallback.
- Skills (factory-wide): `ros2-pixi` (how to add ROS packages, source env, run nodes headless), `sandbox-contract` (the five interfaces, never bypass them), `grading-rules` (seeded faults, healthy controls, evidence).
- Factory definition: repo, default models, skills. Runner setup command: install pixi, then `pixi install -e default` (idempotent, runs on every workspace prep). Confirm exact file names against Warp's 'Definitions as code' docs; the schema isn't public in what I could read.
- Automation: on PR opened against main → review agent runs `pixi run test-headless` and posts a verdict.
- Work source: one GitHub issue per step below, using the step's 'done when' as acceptance criteria.

**Done when:** A trivial issue ('add a hello-world test') goes through the factory and comes back as a passing PR you merge.

## S1 · Pixi environment on Mac and Linux  (Phase 1)

**Needs:** P0  
One install command gives ROS 2 Jazzy, MuJoCo, OpenCV and test tools on both your Mac and the factory's Linux runners.

- `pixi.toml`: channels `robostack-jazzy` + `conda-forge`; platforms `osx-arm64`, `linux-64`; deps `ros-jazzy-ros-base`, `ros-jazzy-rosbag2-storage-mcap`, `ros-jazzy-tf2-ros-py`, `mujoco`, `opencv`, `psutil`, `pytest`.
- Separate `viz` feature with `ros-jazzy-rviz2` (Mac only, optional) so a broken RViz never blocks the build.
- Linux headless: `MUJOCO_GL=egl` (or `osmesa` fallback) set in the `test-headless` task.

**Done when:** `pixi run smoke` starts a ROS talker/listener and renders one MuJoCo frame to PNG, on your Mac and in a factory run.

## S2 · Build the pick-and-place cell in MuJoCo  (Phase 1)

**Needs:** S1  
A small copy of Pawel's cell: an arm with a gripper, 4 fixed cameras, 7 wires and 2 crates on a table.

- `sandbox/scenes/cell.xml` including the Menagerie UR5e + 2F-85; wires as capsule chains (or a single rigid capsule to start); crates as open boxes.
- 4 cameras with names matching Pawel's naming where known; each RGB + depth.
- `World` class: step, render(cam), get_body_pose(name), set_camera_pose(name).

**Done when:** A test renders all 4 cameras headless and checks that every wire and crate is visible in at least one view. **You** eyeball the PNGs on Mac.

## S3 · Scripted pick-and-place that can 'lie'  (Phase 1)

**Needs:** S2  
The robot picks a named wire and drops it into a crate using simple scripted moves. Like the real system, it reports success from its own commands, not from what really happened.

- Inverse kinematics via MuJoCo (damped least squares) to pre-grasp, grasp, lift, move, place poses.
- Episode flow state machine with states published as 'flow state' (IDLE → APPROACH → GRASP → LIFT → TRANSPORT → PLACE → DONE).
- Target pose comes from a 'perception' function = true pose + depth-noise + the camera's **configured** (possibly wrong) extrinsics. This is what makes Task 2 work.

**Done when:** 20 seeded episodes with no faults: ≥ 90% wires end up inside the target crate (checked from ground truth).

## S4 · ROS 2 bridge node  (Phase 1)

**Needs:** S3  
Exposes the sim the way the real stack does: robot joints, coordinate frames, camera images, point clouds, and a pick-and-place action.

- Publishes `/joint_states`, `/tf` + `/tf_static`, `/cameras/<n>/color/image_raw`, `/cameras/<n>/depth/points`, `/flow/state`, `/diagnostics`.
- Action server `/pick_place` (goal: object id, target crate) returning success per command results only.
- Camera extrinsics in TF come from `config/cameras.yaml` (the 'configured' pose), never from the sim's truth.
- Sim clock on `/clock`; `use_sim_time` everywhere so delays can be injected cleanly later.

**Done when:** `ros2 topic list/hz` shows all topics at the expected rates in a headless test; RViz on your Mac shows the robot, frames and point clouds (if RViz starts).

## S5 · Ground truth channel and fault hooks  (Phase 1)

**Needs:** S4  
Two side doors the agent is not allowed through: one for the grader to read the true state, one for the harness to break things on purpose.

- Ground truth over a separate local socket (or file) per attempt, not a ROS topic, so the agent can't discover it with `ros2 topic list`.
- Fault hooks from a seeded YAML: `wrong_object`, `drop_in_transit(t)`, `place_outside_crate(offset)`, `camera_offset(cam, xyz, rpy)`, `cpu_hog(n)`, `mem_hog(gb)`, `topic_delay(topic, ms)`, `topic_drop(topic, rate)`.
- Every fault is reproducible from (scenario, seed).

**Done when:** A test runs each fault once and confirms from ground truth that it really happened, and that a no-fault run stays healthy.

## S6 · Bring-up, readiness check, recording  (Phase 1)

**Needs:** S5  
One command starts the whole sandbox, waits until it is truly ready, and can record every episode for offline replay.

- `pixi run bringup` → sim + bridge, readiness = all topics publishing + TF tree complete + one healthy dry pick.
- Warm-up period (configurable) to mirror Isaac Sim's warm-up.
- `record.py` writes MCAP bags per episode; ground truth saved alongside in a separate, grader-only file.

**Done when:** Bring-up from cold to ready in under a minute on Mac; a recorded episode replays with `ros2 bag play`.

## S7 · Interface contract with two backends  (Phase 2)

**Needs:** S6  
Write down the five interfaces from Pawel's diagram as code. The Mac sandbox is the first backend; Pawel's Docker sandbox becomes the second later.

- `interfaces/contract.py`: `Observe` (topics, TF, images, logs, host metrics), `Act` (shell, config edit, actions), `GroundTruth`, `Faults`, `Lifecycle` (bringup, ready, teardown).
- `backends/mac_mujoco.py` implements all of them; `backends/isaac_docker.py` is a stub raising NotImplemented with TODOs from the brief.
- Backend picked by `BENCH_BACKEND` env var.

**Done when:** Harness and agent code import only `interfaces/`; a lint test fails if they import `sandbox/` directly.

## S8 · Harness: run isolated, seeded attempts  (Phase 3)

**Needs:** S7  
Runs one attempt = fresh sandbox + one fault (or none) + the agent + grading, and keeps attempts from interfering with each other.

- Isolation per attempt: fresh processes, unique `ROS_DOMAIN_ID`, temp working copy of `config/`, fixed seed.
- Scenario files `harness/scenarios/task<N>.yaml` mixing faulty and healthy-control runs (e.g. 30% healthy).
- Result bundle per attempt: agent transcript, tool calls, agent's final answer (JSON), timings, token cost, grade.

**Done when:** `pixi run bench --task 1 --n 10 --agent noop` produces 10 result bundles; same seed twice gives the same faults.

## S9 · Graders and the result format  (Phase 3)

**Needs:** S8  
Scores each attempt from what physically happened, never from what the agent claims. Healthy runs that the agent calls broken count against it.

- Task 1: per-episode verdict accuracy, false-alarm rate on healthy controls, correct failure type, failure time within ±2 s.
- Task 2: translation/rotation error vs true camera pose (e.g. pass < 5 mm, < 0.5°) + pick success over 10 validation picks.
- Task 3: culprit named correctly, cause removed (not symptom hidden: e.g. timeout raised doesn't count), cycle time back within 10% of baseline.
- Task 4 (injected): delay source named, fix removes ≥ 95% of phantom voxels near the arm.
- Report: one table per task × agent profile, plus cost and time.

**Done when:** Graders score hand-made 'perfect' and 'all wrong' answers correctly in unit tests.

## S10 · Agent tool server (MCP)  (Phase 4)

**Needs:** S7  
The agent's eyes and hands, built only on the interface contract: list topics, read messages, look up frames, grab camera and RViz images, read host metrics and logs, edit config, run shell commands.

- `agent/tools_mcp.py`: `ros_topics`, `ros_echo(topic, n)`, `ros_hz`, `tf_lookup`, `camera_snapshot(cam)`, `render_view(kind)` (RViz-like image: point cloud / voxels via Open3D/matplotlib if RViz2 is unavailable), `host_metrics`, `logs_tail`, `config_get/set`, `shell`.
- Tool profiles to switch modalities on/off for the baseline comparison.

**Done when:** Claude Code connected to the MCP server can answer 'which wire is in crate A right now?' from a camera snapshot.

## S11 · Agent loop and plain baseline  (Phase 4)

**Needs:** S10, S9  
The robotics agent itself, plus a 'plain coding agent' with only shell and file access, so we can measure what each extra tool adds.

- Claude Agent SDK, model `claude-opus-5-5` (and `claude-sonnet-5-5` for cost comparison), system prompt per task, required JSON answer schema with evidence references (tool call ids).
- Profiles: `baseline` (shell + files), `+ros`, `+ros+metrics`, `+ros+metrics+vision`.

**Done when:** All profiles complete a Task 1 attempt and produce a gradeable answer.

## S12 · Task 1 end to end: catch unreported failures  (Phase 4)

**Needs:** S11  
First real benchmark result: a stream of episodes, some silently broken, some healthy, judged by each agent profile.

- Scenario: 40 episodes, faults = wrong object, drop in transit, placed outside crate; 30–40% healthy controls.
- Also run on recorded MCAP episodes for a cheap offline mode.

**Done when:** A results table comparing profiles, which you review with the team. **First demo to Pawel.**

## S13 · Task 2: calibrate a camera  (Phase 5)

**Needs:** S12  
One camera's configured pose is a few cm/degrees off. The agent plans arm poses holding a calibration board, estimates the true pose, writes it to config, and proves it with picks.

- Add a ChArUco board attached to the gripper in the scene.
- Tools: `move_arm_to(pose)`, OpenCV hand-eye calibration helper (mirrors Pawel's calibration tools).
- Grader reads true camera pose from ground truth.

**Done when:** Agent result within the pass thresholds on ≥ 3 different seeded offsets, plus a healthy control where the camera is already right.

## S14 · Task 3 prototype: resource bottleneck  (Phase 5)

**Needs:** S12  
Add one pressure source (CPU hog, memory hog, slowed or dropped messages) and give the agent only 'picks got slow and sometimes time out'.

- Mac limits: no GPU contention, no cgroups or `/proc`; use `psutil` and process-level hogs.
- Healthy control: slow-looking but normal run.

**Done when:** Prototype runs and grades; known Mac gaps are listed for the Linux GPU host version.

## S15 · Task 4 prototype: injected timing bug  (Phase 5)

**Needs:** S12  
Delay the point clouds against the robot's joint data so phantom obstacles appear around the moving arm. The agent must prove the cause by experiment and remove the phantoms.

- Add MoveIt's occupancy map (octomap) from RoboStack, or a small stand-in voxel map with self-filtering if MoveIt is too painful on Mac.
- Injected `topic_delay` on the depth cloud or a TF timestamp offset.
- Grader counts phantom voxels near the arm before/after.

**Done when:** Phantoms visible in a rendered view with the fault on and gone without it; the agent's fix is graded by voxel count.

## S16 · Hand-off to Pawel's sandbox  (Phase 6)

**Needs:** S13, S14, S15  
When the real Docker sandbox arrives, implement the second backend and rerun the same scenarios on the GPU host.

- Fill `backends/isaac_docker.py` against Pawel's bring-up script, ground truth and fault hooks.
- Diff the topic names and frames against our stand-in; fix the stand-in to match.

**Done when:** Task 1 scenario runs unchanged on both backends.
