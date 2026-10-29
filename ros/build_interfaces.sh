#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cmake -S "$ROOT/ros/interfaces" -B "$ROOT/ros/build" -G Ninja \
  -DCMAKE_INSTALL_PREFIX="$ROOT/ros/install" \
  -DCMAKE_INSTALL_RPATH="$ROOT/ros/install/lib;$CONDA_PREFIX/lib" \
  -DPython3_EXECUTABLE="$(command -v python)" -DBUILD_TESTING=OFF
cmake --build "$ROOT/ros/build" --parallel 2
cmake --install "$ROOT/ros/build"
