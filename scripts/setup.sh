#!/usr/bin/env bash
# Installs uv (Python) and, once the ROS env exists, pixi (ROS 2 only), then both environments.
# Safe to run repeatedly.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOCAL_BIN="$HOME/.local/bin"
PIXI_BIN="${PIXI_HOME:-$HOME/.pixi}/bin"
export PATH="$LOCAL_BIN:$PIXI_BIN:$PATH"

# Make the tools available to later shells (factory sandboxes, CI steps).
if ! grep -qs '.local/bin' "$HOME/.bashrc"; then
  echo 'export PATH="$HOME/.local/bin:$HOME/.pixi/bin:$PATH"' >> "$HOME/.bashrc"
fi

# Python: uv
if ! command -v uv >/dev/null 2>&1; then
  curl -LsSf https://astral.sh/uv/install.sh | env UV_NO_MODIFY_PATH=1 sh
fi
uv --version
if [ -f "$REPO_ROOT/pyproject.toml" ]; then
  (cd "$REPO_ROOT" && uv sync)
else
  echo "No pyproject.toml yet; skipping Python environment."
fi

# ROS 2: pixi, only for ros/pixi.toml
if [ -f "$REPO_ROOT/ros/pixi.toml" ]; then
  if ! command -v pixi >/dev/null 2>&1; then
    curl -fsSL https://pixi.sh/install.sh | PIXI_NO_PATH_UPDATE=1 bash
  fi
  pixi --version
  pixi install --manifest-path "$REPO_ROOT/ros/pixi.toml"
else
  echo "No ros/pixi.toml yet; skipping ROS environment."
fi
