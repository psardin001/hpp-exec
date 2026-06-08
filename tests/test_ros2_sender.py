"""Tests for ROS 2 sender internals."""

import threading
import time

import numpy as np
import pytest


def _ros2_sender():
    pytest.importorskip("control_msgs")
    pytest.importorskip("rclpy")

    import hpp_exec.ros2_sender as ros2_sender

    return ros2_sender


def _action(calls, name):
    return lambda: calls.append(name) or True


class MockTransition:
    def __init__(self, name):
        self._name = name

    def name(self):
        return self._name


def test_trajectory_type_support_import_is_serialized(monkeypatch):
    ros2_sender = _ros2_sender()

    active_calls = 0
    max_active_calls = 0
    calls_lock = threading.Lock()

    def fake_check_for_type_support(action_type):
        nonlocal active_calls, max_active_calls
        assert action_type is ros2_sender.FollowJointTrajectory
        with calls_lock:
            active_calls += 1
            max_active_calls = max(max_active_calls, active_calls)
        time.sleep(0.01)
        with calls_lock:
            active_calls -= 1

    monkeypatch.setattr(
        ros2_sender, "check_for_type_support", fake_check_for_type_support
    )

    threads = [
        threading.Thread(target=ros2_sender._ensure_trajectory_type_support)
        for _ in range(2)
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=1.0)

    assert all(not thread.is_alive() for thread in threads)
    assert max_active_calls == 1


def test_execute_segments_runs_single_transition_actions(monkeypatch):
    ros2_sender = _ros2_sender()

    calls = []

    def fake_send_trajectory(*args, **kwargs):
        del args, kwargs
        calls.append("trajectory")
        return True

    monkeypatch.setattr(ros2_sender, "send_trajectory", fake_send_trajectory)

    segments = [
        ros2_sender.Segment(0, 2, transition_name="approach"),
        ros2_sender.Segment(1, 3, transition_name="grasp"),
    ]
    configs = [np.array([0.0]), np.array([1.0]), np.array([2.0])]
    times = [0.0, 1.0, 2.0]

    assert ros2_sender.execute_segments(
        segments,
        configs,
        times,
        joint_names=["joint"],
        pre_actions_by_transition={"grasp": _action(calls, "pre-grasp")},
        post_actions_by_transition={"grasp": _action(calls, "post-grasp")},
    )

    assert calls == ["trajectory", "pre-grasp", "trajectory", "post-grasp"]


def test_execute_segments_accepts_transition_objects_in_action_maps(monkeypatch):
    ros2_sender = _ros2_sender()

    calls = []
    monkeypatch.setattr(
        ros2_sender,
        "send_trajectory",
        lambda *args, **kwargs: calls.append("trajectory") or True,
    )

    segments = [
        ros2_sender.Segment(0, 2, transition_name="approach"),
        ros2_sender.Segment(1, 3, transition_name="grasp"),
    ]

    assert ros2_sender.execute_segments(
        segments,
        [np.array([0.0]), np.array([1.0]), np.array([2.0])],
        [0.0, 1.0, 2.0],
        joint_names=["joint"],
        pre_actions_by_transition={
            MockTransition("approach"): _action(calls, "pre-approach"),
            "grasp": _action(calls, "pre-grasp"),
        },
        post_actions_by_transition={
            MockTransition("grasp"): _action(calls, "post-grasp"),
        },
    )

    assert calls == [
        "pre-approach",
        "trajectory",
        "pre-grasp",
        "trajectory",
        "post-grasp",
    ]


def test_execute_segments_runs_transition_action_lists_in_order(monkeypatch):
    ros2_sender = _ros2_sender()

    calls = []
    monkeypatch.setattr(
        ros2_sender,
        "send_trajectory",
        lambda *args, **kwargs: calls.append("trajectory") or True,
    )

    segment = ros2_sender.Segment(
        0,
        2,
        pre_actions=[_action(calls, "segment-pre")],
        post_actions=[_action(calls, "segment-post")],
        transition_name="handoff",
    )

    assert ros2_sender.execute_segments(
        [segment],
        [np.array([0.0]), np.array([1.0])],
        [0.0, 1.0],
        joint_names=["joint"],
        pre_actions_by_transition={
            "handoff": [
                _action(calls, "map-pre-1"),
                _action(calls, "map-pre-2"),
            ]
        },
        post_actions_by_transition={
            "handoff": (
                _action(calls, "map-post-1"),
                _action(calls, "map-post-2"),
            )
        },
    )

    assert calls == [
        "segment-pre",
        "map-pre-1",
        "map-pre-2",
        "trajectory",
        "segment-post",
        "map-post-1",
        "map-post-2",
    ]
    assert segment.pre_actions and len(segment.pre_actions) == 1
    assert segment.post_actions and len(segment.post_actions) == 1


def test_execute_segments_rejects_unknown_transition_before_running(monkeypatch):
    ros2_sender = _ros2_sender()

    calls = []
    monkeypatch.setattr(
        ros2_sender,
        "send_trajectory",
        lambda *args, **kwargs: calls.append("trajectory") or True,
    )

    segment = ros2_sender.Segment(
        0,
        2,
        pre_actions=[_action(calls, "segment-pre")],
        transition_name="known",
    )

    assert not ros2_sender.execute_segments(
        [segment],
        [np.array([0.0]), np.array([1.0])],
        [0.0, 1.0],
        joint_names=["joint"],
        pre_actions_by_transition={"missing": _action(calls, "map-pre")},
    )
    assert calls == []


def test_execute_segments_without_action_maps_keeps_segment_api(monkeypatch):
    ros2_sender = _ros2_sender()

    calls = []
    monkeypatch.setattr(
        ros2_sender,
        "send_trajectory",
        lambda *args, **kwargs: calls.append("trajectory") or True,
    )

    assert ros2_sender.execute_segments(
        [
            ros2_sender.Segment(
                0,
                2,
                pre_actions=[_action(calls, "segment-pre")],
                post_actions=[_action(calls, "segment-post")],
            )
        ],
        [np.array([0.0]), np.array([1.0])],
        [0.0, 1.0],
        joint_names=["joint"],
    )
    assert calls == ["segment-pre", "trajectory", "segment-post"]
