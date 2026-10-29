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
  (cd "$REPO_ROOT" && uv sync --locked)
else
  echo "No pyproject.toml yet; skipping Python environment."
fi

# ROS 2: pixi, only for ros/pixi.toml
if [ -f "$REPO_ROOT/ros/pixi.toml" ]; then
  if ! command -v pixi >/dev/null 2>&1; then
    curl -fsSL https://pixi.sh/install.sh | PIXI_NO_PATH_UPDATE=1 bash
  fi
  pixi --version
  pixi install --locked --manifest-path "$REPO_ROOT/ros/pixi.toml"
  # Load ROS once so macOS checks the freshly installed libraries now, not during the first test.
  pixi run --locked --manifest-path "$REPO_ROOT/ros/pixi.toml" python -c "import rclpy"
  pixi run --locked --manifest-path "$REPO_ROOT/ros/pixi.toml" build-interfaces
else
  echo "No ros/pixi.toml yet; skipping ROS environment."
fi

if [ "$(uname -s)" = Linux ]; then
  missing_render_libs=()
  for package in libosmesa6 libegl1 libgl1; do
    if ! dpkg-query -W -f='${Status}' "$package" 2>/dev/null | grep -q 'install ok installed'; then
      missing_render_libs+=("$package")
    fi
  done
  if [ "${#missing_render_libs[@]}" -gt 0 ]; then
    if [ "$(id -u)" -eq 0 ]; then
      apt-get update
      apt-get install -y "${missing_render_libs[@]}"
    elif command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; then
      sudo -n apt-get update
      sudo -n apt-get install -y "${missing_render_libs[@]}"
    else
      printf 'Missing render libraries: %s. Ask an administrator to install them.\n' "${missing_render_libs[*]}" >&2
      exit 1
    fi
  fi
fi
