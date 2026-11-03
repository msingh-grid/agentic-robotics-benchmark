"""Private capabilities: GroundTruth for grading and Faults for the harness."""

from typing import Protocol


class GroundTruth(Protocol):
    def snapshot(self) -> dict:
        """Read physical measurements, events, and fault validity."""
        ...


class Faults(Protocol):
    def configure(self, scenario: dict, seed: int) -> None:
        """Configure a seeded attempt before its first public action."""
        ...
