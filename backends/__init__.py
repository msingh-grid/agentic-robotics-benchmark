"""Select private capability implementations without importing a simulator."""

import os

from interfaces.private import Faults, GroundTruth


def create_ground_truth(private_dir) -> GroundTruth:
    return _backend().GroundTruth(private_dir)


def create_faults(private_dir) -> Faults:
    return _backend().Faults(private_dir)


def _backend():
    name = os.environ.get("BENCH_BACKEND", "mac_mujoco")
    if name == "mac_mujoco":
        from backends import mac_mujoco
        return mac_mujoco
    if name == "isaac_docker":
        from backends import isaac_docker
        return isaac_docker
    raise ValueError(f"Unknown backend: {name}")
