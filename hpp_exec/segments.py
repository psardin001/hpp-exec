"""Lightweight segment data structures."""

from dataclasses import dataclass, field
from typing import Callable


@dataclass
class Segment:
    """A trajectory segment with optional pre/post actions.

    Actions are callables that return True on success, False on failure.
    Any callable works: bound methods, lambdas, or free functions.
    """

    start_index: int
    """Start config index (inclusive)."""

    end_index: int
    """End config index (exclusive)."""

    pre_actions: list[Callable[[], bool]] = field(default_factory=list)
    """Actions to run before sending this segment's trajectory."""

    post_actions: list[Callable[[], bool]] = field(default_factory=list)
    """Actions to run after this segment's trajectory completes."""

    start_time: float = 0.0
    """Start time in the sampled trajectory."""

    end_time: float = 0.0
    """End time in the sampled trajectory."""

    transition_name: str = ""
    """HPP graph transition name for this segment."""

    state_before: str = ""
    """Nominal source state of the graph transition."""

    state_after: str = ""
    """Nominal target state of the graph transition."""

    actual_state_before: str = ""
    """State observed on the path near the beginning of the segment."""

    actual_state_after: str = ""
    """State observed on the path near the end of the segment."""

    @property
    def duration(self) -> float:
        """Segment duration in seconds."""

        return self.end_time - self.start_time
