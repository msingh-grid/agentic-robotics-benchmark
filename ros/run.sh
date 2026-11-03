#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SETUP="$ROOT/ros/install/share/benchmark_interfaces/local_setup.bash"
if [ ! -f "$SETUP" ]; then
  printf 'Build the action first: pixi run --manifest-path ros/pixi.toml build-interfaces\n' >&2
  exit 1
fi
set +u
source "$SETUP"
set -u
# RoboStack's relocatable hooks omit the loader path needed by runtime-loaded action typesupport.
case "$(uname -s)" in
  Linux) export LD_LIBRARY_PATH="$ROOT/ros/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
  Darwin) export DYLD_LIBRARY_PATH="$ROOT/ros/install/lib${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}" ;;
esac
exec "$@"
