# Factory definition snippets

This factory's definition is Warp-managed (web app sidebar → factory → **Factory definition**). Files here are copies to paste there; they are not read from this repo.

- `runners/pixi-linux.yaml`: Linux runner that installs pixi and the project environment on every workspace prep. Validated against Factory schema `v1alpha1`. To use it, add it as `runners/pixi-linux.yaml` in the factory definition and set `agentDefaults.runner: pixi-linux` in `factory.yaml`.

Agent rules and skills live in the repo itself (`AGENTS.md`, `.agents/skills/`), so every harness picks them up without definition changes.
