from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import numpy as np

from hpp_exec.segments import Segment

_PARAM_EPS = 1e-9
_STATE_SAMPLE_EPS = 1e-5


@dataclass(frozen=True)
class _GraphSegment:
    start_time: float
    end_time: float
    transition_name: str
    state_before: str
    state_after: str
    actual_state_before: str
    actual_state_after: str


def _path_ranges(path) -> list[tuple[float, float]]:
    ranges = []
    cursor = 0.0
    for rank in range(int(path.numberPaths())):
        length = float(path.pathAtRank(rank).length())
        end = cursor + length
        if end - cursor > _PARAM_EPS:
            ranges.append((cursor, end))
        cursor = end
    return ranges


def _sample_path_at(path, t: float) -> np.ndarray:
    q, success = path(t)
    if not success:
        raise RuntimeError(f"HPP failed to evaluate path at t={t:.6f}")
    return np.asarray(q)


def _state_sample_times(start: float, end: float) -> tuple[float, float]:
    margin = min(_STATE_SAMPLE_EPS, 0.25 * (end - start))
    return start + margin, end - margin


def _state_at(path, graph, t: float) -> str:
    return str(graph.getStateFromConfiguration(_sample_path_at(path, t)))


def _graph_segments(path, graph) -> list[_GraphSegment]:
    graph_segments = []
    for start, end in _path_ranges(path):
        midpoint = start + 0.5 * (end - start)
        transition = graph.transitionAtParam(path, midpoint)
        state_before, state_after = graph.getNodesConnectedByTransition(transition)
        actual_start, actual_end = _state_sample_times(start, end)

        graph_segments.append(
            _GraphSegment(
                start_time=start,
                end_time=end,
                transition_name=str(transition.name()),
                state_before=str(state_before),
                state_after=str(state_after),
                actual_state_before=_state_at(path, graph, actual_start),
                actual_state_after=_state_at(path, graph, actual_end),
            )
        )
    return graph_segments


def _regular_sample_times(
    length: float,
    *,
    n_per_unit: int,
    min_samples: int,
) -> list[float]:
    n_samples = max(int(length * n_per_unit), min_samples)
    return [(i / n_samples) * length for i in range(n_samples + 1)]


def _unique_times(times: Iterable[float]) -> list[float]:
    values = sorted(float(t) for t in times)
    unique = []
    for value in values:
        if not unique or abs(value - unique[-1]) > _PARAM_EPS:
            unique.append(value)
    return unique


def _time_index(times: list[float], t: float) -> int:
    return min(range(len(times)), key=lambda i: abs(times[i] - t))


def segments_from_graph(
    path,
    graph,
    *,
    n_per_unit: int = 50,
    min_samples: int = 50,
    sample_params: Iterable[float] | None = None,
) -> tuple[list[np.ndarray], list[float], list[Segment]]:
    length = float(path.length())
    graph_segments = _graph_segments(path, graph)
    base_times = (
        list(sample_params)
        if sample_params is not None
        else _regular_sample_times(
            length, n_per_unit=n_per_unit, min_samples=min_samples
        )
    )
    boundary_times = [0.0, length]
    for segment in graph_segments:
        boundary_times.extend((segment.start_time, segment.end_time))

    times = _unique_times([*base_times, *boundary_times])
    configs = [_sample_path_at(path, t) for t in times]

    segments = []
    for graph_segment in graph_segments:
        start_index = _time_index(times, graph_segment.start_time)
        end_index = _time_index(times, graph_segment.end_time) + 1
        segments.append(
            Segment(
                start_index,
                end_index,
                start_time=graph_segment.start_time,
                end_time=graph_segment.end_time,
                transition_name=graph_segment.transition_name,
                state_before=graph_segment.state_before,
                state_after=graph_segment.state_after,
                actual_state_before=graph_segment.actual_state_before,
                actual_state_after=graph_segment.actual_state_after,
            )
        )

    if not segments:
        segments.append(Segment(0, len(configs), start_time=0.0, end_time=length))

    return configs, times, segments


def format_segments(segments: Iterable[Segment]) -> str:
    def state_pair(before: str, after: str) -> str:
        if before and after:
            return f"{before} -> {after}"
        return before or after or "-"

    rows = []
    for index, segment in enumerate(segments):
        rows.append(
            [
                str(index),
                f"{segment.start_time:.6f} -> {segment.end_time:.6f}",
                f"{segment.duration:.6f}",
                segment.transition_name or "-",
                state_pair(
                    segment.actual_state_before,
                    segment.actual_state_after,
                ),
                f"{len(segment.pre_actions)}/{len(segment.post_actions)}",
            ]
        )

    if not rows:
        return "(no segments)"

    headers = [
        "#",
        "time",
        "duration",
        "transition",
        "observed",
        "pre/post",
    ]
    widths = [
        max(len(row[column]) for row in [headers, *rows])
        for column in range(len(headers))
    ]

    def line(values: list[str]) -> str:
        return "  ".join(
            value.ljust(widths[index]) for index, value in enumerate(values)
        )

    separator = "  ".join("-" * width for width in widths)
    return "\n".join([line(headers), separator, *(line(row) for row in rows)])


def print_segments(segments: Iterable[Segment]) -> None:
    print(format_segments(segments))
