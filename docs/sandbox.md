# Start, record and replay the sandbox (Phase 1 · S6)

Run from the repository root after `bash scripts/setup.sh`.
All commands work headlessly on Apple Silicon macOS and Linux x86_64.
No physics, scene, or scripted pick motion changes are part of S6.

## Start and stop

```bash
uv run bringup --seed 0
```

Wait for `READY ROS_DOMAIN_ID=N elapsed=...s`. The elapsed time includes **both**
launches, warm-up, the dry pick, physical validation, and scenario configuration.
The entire sequence has a 60-second deadline; setup and dependency installation
are separate. Failure exits nonzero without printing READY.
Ctrl+C or SIGTERM stops the process groups and releases the domain lease.
Unexpected server or bridge exit also tears down the session.
Child logs remain in owner-only `/tmp/cell-*` directories for diagnosis.

Readiness requires two messages on every periodic stream, latched static TF,
every robot/camera/optical frame connected to `world`, healthy diagnostics,
and the action server. A disposable **healthy** stack performs a dry pick of
`wire0` into `crate0`. A grading-only process checks the measured placement,
not action success. A fresh stack then starts from the original seed, warms up,
passes topics/TF readiness, and receives the scenario through private IPC.
This avoids consuming a wire or advancing fault randomness during preflight.
Requested topic loss cannot prevent an intentionally faulted stack from becoming ready.

The supervisor leases a free local ROS domain in 0–101 and restricts discovery to
localhost. Set `ROS_DOMAIN_ID` to reserve a specific domain; a domain already leased
by another benchmark process fails instead of silently sharing a cell.
Other terminals must use the printed domain:

```bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
pixi run --locked --manifest-path ros/pixi.toml ros-cli topic hz /joint_states
pixi run --locked --manifest-path ros/pixi.toml ros-cli action send_goal /pick_place benchmark_interfaces/action/PickPlace '{object_id: wire0, crate_id: crate0}' --feedback
```

Replace 42 with the printed value. Topic, frame, camera and action names are still
provisional (`TODO(pawel)` in the public contract).

## Record episodes

```bash
uv run record --episodes 2 --seed 0 --output out/healthy
uv run record --episodes 1 --seed 0 --scenario tests/fixtures/drop_in_transit.yaml --output out/drop
```

`record` owns its own stack; it does not attach to a running `bringup`.
Each episode uses a new stack, the same healthy readiness sequence, and seed
`--seed + zero-based episode index`. Each episode makes one public ROS pick of
`wire0` into `crate0`. The scenario applies independently to each fresh episode.
Omit `--scenario` for a healthy control. Scenario YAML uses the S5 `faults` list;
`faults: []` explicitly selects healthy behavior.
The output defaults to a unique UTC-named directory under `out/`.
Existing episode directories are never overwritten.

Each `episode-0001/` contains:

- `rosbag/metadata.yaml` and an MCAP file produced by pixi's `ros2 bag record`.
- `grader/ground_truth.jsonl`: private snapshots at approximately 10 Hz, including
  physical poses, contacts, timestamped events and fault verification.

The bag allowlist contains only public sensor topics, TF, flow, diagnostics,
and action feedback/status. It excludes ROS logs, parameters, private files,
scenario definitions and ground-truth data. Recording waits for subscriptions
before the pick and flushes the MCAP index on shutdown. Bag timestamps use receive
time, so a fault that drops `/clock` does not block recording; messages retain
their original simulation timestamps. Three trailing seconds capture final
flow/camera messages and transport audits.
Action success still means commands completed, not successful placement.
An ineffective fault remains `verification.status: invalid` in the private file.

Episode and grader directories have mode 0700; ground-truth files have mode 0600
from creation. **Only share/mount `rosbag/` with the benchmarked agent.**
Never share the episode parent, private run directories or scenario definitions.
Mode bits do not isolate processes with the same UID: the harness must enforce
separate user/container filesystem access when running a benchmarked agent.

## Replay and Phase 1 demo

Stop live cells first; replay on an unused domain, not alongside a live `/clock`.

```bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
pixi run --locked --manifest-path ros/pixi.toml ros-cli bag info "$PWD/out/drop/episode-0001/rosbag"
pixi run --locked --manifest-path ros/pixi.toml ros-cli bag play "$PWD/out/drop/episode-0001/rosbag"
```

The recorded `/clock`, images, clouds, joints, TF and flow replay as public topics.
The private sidecar is neither opened nor published by `bag play`.
The ROS command wrapper applies the camera-sized Fast DDS transport profile to
playback as well as the live bridge; either user-supplied DDS profile takes precedence.
Replay does not re-execute robot commands or reproduce live action services.
If RViz works on your Mac, use fixed frame `world` and Image/TF/PointCloud2 displays;
it is optional. For a headless observation, run `ros-cli topic echo /flow/state`
in another terminal with the same domain.

For the Phase 1 demo, record the healthy command above, then the drop command,
and replay each MCAP. Observe the red wire placed in crate0 in the healthy episode,
but dropped during transport in the faulted episode. Both actions report command
success. Public camera streams show the difference; private measurements are for
the grader, not the observer.

## Verification and Mac check

```bash
bash scripts/setup.sh
uv run pytest
uv run record --episodes 1 --seed 0 --output out/mac-healthy
uv run record --episodes 1 --seed 0 --scenario tests/fixtures/drop_in_transit.yaml --output out/mac-drop
```

On a Mac, confirm each cold-start READY line is under 60 seconds, inspect both
replays with the commands above, and stop a running `bringup` with Ctrl+C.
The S6 tests exercise real ROS MCAP capture, deserialization, replay, healthy
preflight before a fault, missing topic/TF rejection and process shutdown.
Isaac lifecycle, full benchmark attempts and automated visual review are follow-ups.
