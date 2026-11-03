"""Harness-only lifecycle and episode recording capabilities."""

from pathlib import Path
from typing import Protocol


class Lifecycle(Protocol):
    @property
    def ros_domain(self) -> str: ...

    def start(self, seed: int) -> None: ...

    def wait_ready(self, scenario: dict, timeout: float = 60) -> None:
        """Warm up, prove a healthy pick, then prepare a fresh seeded attempt."""
        ...

    def check(self) -> None:
        """Fail if a managed process exited."""
        ...

    def record_episode(self, output: Path) -> None:
        """Record one public pick and its separate grader-only measurements."""
        ...

    def stop(self) -> None: ...
