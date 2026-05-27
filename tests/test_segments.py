"""Tests for graph segment execution."""

import numpy as np


class MockTransition:
    def __init__(self, name: str, state_from: str, state_to: str):
        self._name = name
        self.state_from = state_from
        self.state_to = state_to

    def name(self):
        return self._name


class MockSubPath:
    def __init__(self, length: float):
        self._length = length

    def length(self):
        return self._length


class MockPathVector:
    def __init__(self, subpath_lengths: list[float]):
        self._subpaths = [MockSubPath(length) for length in subpath_lengths]
        self._length = sum(subpath_lengths)

    def length(self):
        return self._length

    def numberPaths(self):
        return len(self._subpaths)

    def pathAtRank(self, rank):
        return self._subpaths[rank]

    def __call__(self, param):
        return np.array([float(param)]), True


class PathAwareGraph:
    def __init__(self, transition_ranges, state_windows):
        self.queried_params = []
        self._transition_ranges = transition_ranges
        self._state_windows = state_windows

    def transitionAtParam(self, path, param):
        del path
        self.queried_params.append(param)
        for start, end, transition in self._transition_ranges:
            if start <= param <= end:
                return transition
        raise ValueError(f"No transition at path parameter {param}")

    def getNodesConnectedByTransition(self, transition):
        return (transition.state_from, transition.state_to)

    def getStateFromConfiguration(self, config):
        param = float(config[0])
        for start, end, state in self._state_windows:
            if start <= param <= end:
                return state
        raise ValueError(f"No state at path parameter {param}")


def windowed_pick_place_graph():
    grasped = "gripper grasps box/handle"
    return PathAwareGraph(
        [
            (0.0, 4.8, MockTransition("free loop | approach", "free", "free")),
            (4.8, 5.2, MockTransition("grasp stage | f_12", "free", grasped)),
            (5.2, 10.0, MockTransition("carry loop | g", grasped, grasped)),
            (10.0, 10.4, MockTransition("release stage | 12_f", grasped, "free")),
        ],
        [
            (0.0, 5.0, "free"),
            (5.0, 10.2, grasped),
            (10.2, 10.4, "free"),
        ],
    )


def test_segments_from_graph_returns_hpp_segments():
    from hpp_exec.graph_segments import segments_from_graph

    path = MockPathVector([4.8, 0.4, 4.8, 0.4])
    graph = windowed_pick_place_graph()

    configs, times, segments = segments_from_graph(
        path,
        graph,
        sample_params=[0.0, 10.4],
    )

    assert graph.queried_params == [2.4, 5.0, 7.6, 10.2]
    assert times == [0.0, 4.8, 5.2, 10.0, 10.4]
    assert np.allclose([config[0] for config in configs], times)

    assert [(seg.start_index, seg.end_index) for seg in segments] == [
        (0, 2),
        (1, 3),
        (2, 4),
        (3, 5),
    ]
    assert [seg.transition_name for seg in segments] == [
        "free loop | approach",
        "grasp stage | f_12",
        "carry loop | g",
        "release stage | 12_f",
    ]
    assert segments[1].actual_state_before == "free"
    assert segments[1].actual_state_after == "gripper grasps box/handle"
    assert segments[3].actual_state_before == "gripper grasps box/handle"
    assert segments[3].actual_state_after == "free"
    assert all(segment.pre_actions == [] for segment in segments)
    assert all(segment.post_actions == [] for segment in segments)


def test_user_adds_actions_manually_to_chosen_segments():
    from hpp_exec.graph_segments import segments_from_graph

    calls = []
    _, _, segments = segments_from_graph(
        MockPathVector([4.8, 0.4, 4.8, 0.4]),
        windowed_pick_place_graph(),
        sample_params=[0.0, 10.4],
    )

    segments[1].pre_actions.append(lambda: calls.append("close") or True)
    segments[3].post_actions.append(lambda: calls.append("open") or True)

    assert segments[1].pre_actions[0]()
    assert segments[3].post_actions[0]()
    assert calls == ["close", "open"]


def test_format_segments_shows_timing_state_and_actions():
    from hpp_exec.graph_segments import format_segments, segments_from_graph

    _, _, segments = segments_from_graph(
        MockPathVector([4.8, 0.4, 4.8, 0.4]),
        windowed_pick_place_graph(),
        sample_params=[0.0, 10.4],
    )
    segments[1].pre_actions.append(lambda: True)

    table = format_segments(segments)

    assert "#  time" in table
    assert "nominal" not in table
    assert "4.800000 -> 5.200000" in table
    assert "grasp stage | f_12" in table
    assert "free -> gripper grasps box/handle" in table
    assert "1/0" in table


def test_segment_defaults_and_duration():
    from hpp_exec import Segment

    seg = Segment(0, 100, start_time=1.25, end_time=3.75)

    assert seg.start_index == 0
    assert seg.end_index == 100
    assert seg.pre_actions == []
    assert seg.post_actions == []
    assert seg.duration == 2.5
