#!/usr/bin/env bash
# Installs pixi (if missing) and the project environment. Safe to run repeatedly.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if ! command -v pixi >/dev/null 2>&1; then
  export PIXI_HOME="${PIXI_HOME:-$HOME/.pixi}"
  if [ ! -x "$PIXI_HOME/bin/pixi" ]; then
    curl -fsSL https://pixi.sh/install.sh | PIXI_NO_PATH_UPDATE=1 bash
  fi
  export PATH="$PIXI_HOME/bin:$PATH"
fi

# Make pixi available to later shells (factory sandboxes, CI steps).
if ! grep -qs 'PIXI_HOME/bin\|\.pixi/bin' "$HOME/.bashrc"; then
  echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> "$HOME/.bashrc"
fi

pixi --version

if [ -f "$REPO_ROOT/pixi.toml" ]; then
  cd "$REPO_ROOT"
  pixi install
else
  echo "No pixi.toml yet; skipping environment install."
fi
