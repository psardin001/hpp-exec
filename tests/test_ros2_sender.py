"""Tests for ROS 2 sender internals."""

import threading
import time

import pytest


def test_trajectory_type_support_import_is_serialized(monkeypatch):
    pytest.importorskip("control_msgs")
    pytest.importorskip("rclpy")

    import hpp_exec.ros2_sender as ros2_sender

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
