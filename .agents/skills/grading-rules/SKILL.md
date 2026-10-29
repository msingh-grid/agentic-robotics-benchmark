---
name: grading-rules
description: How faults, scenarios, healthy controls, grading and result bundles must work in this benchmark. Read before touching sandbox fault hooks, harness/, grading/ or the agent's answer format.
---

# Grading rules

## Faults and scenarios

- A fault is fully defined by `(scenario file, seed)`. Running the same pair twice must inject the same fault at the same moment.
- Scenario files live in `harness/scenarios/task<N>.yaml`. Each one lists attempts with a fault (or `none`) and a seed.
- Every scenario set includes **healthy controls** (roughly 30–40% of attempts) where nothing is wrong. "Nothing is wrong" is a valid, gradeable answer.
- After injecting a fault, verify from ground truth that it actually took effect; an attempt where the fault silently failed is invalid, not "healthy".

## Grading

- Grade from ground truth and measurements only. Never trust the agent's self-report or ROS action results.
- Penalise false alarms: calling a healthy run broken counts against the agent as much as missing a real failure.
- Fixing the symptom instead of the cause fails (e.g. raising a timeout instead of removing CPU pressure).
- Each grader is a pure function: `grade(answer, ground_truth, measurements) -> Grade`. Unit-test it with a hand-written perfect answer and a hand-written all-wrong answer.

## Agent answers

- The agent returns a JSON answer matching the task's schema, with **evidence**: references to the tool calls or recorded data it used. Claims without evidence score lower.

## Result bundle (one per attempt)

Saved under `results/<run-id>/<attempt-id>/`:
- `scenario.json` (task, fault, seed, backend)
- `transcript.jsonl` (agent messages and tool calls)
- `answer.json` (the agent's final answer)
- `grade.json` (score and per-criterion breakdown)
- `metrics.json` (wall time, tokens, cost)

## Baselines

Every task is run with several agent tool profiles, from `baseline` (shell and files only) up to the full set (ROS + metrics + vision). Results tables compare profiles side by side so we can see what each tool adds.
