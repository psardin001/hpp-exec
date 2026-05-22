"""Tests for reusable segment actions."""

import threading
import time


def test_background_action_start_returns_before_action_finishes():
    from hpp_exec import BackgroundAction

    started = threading.Event()
    release = threading.Event()
    calls = []

    def action():
        calls.append("start")
        started.set()
        release.wait(timeout=1.0)
        calls.append("finish")
        return True

    background = BackgroundAction(action, name="slow_action")

    assert background.start()
    assert started.wait(timeout=1.0)
    assert calls == ["start"]

    release.set()
    assert background.wait(timeout_sec=1.0)
    assert calls == ["start", "finish"]


def test_background_action_does_not_duplicate_running_action():
    from hpp_exec import BackgroundAction

    started = threading.Event()
    release = threading.Event()
    calls = []

    def action():
        calls.append("run")
        started.set()
        release.wait(timeout=1.0)
        return True

    background = BackgroundAction(action)

    assert background.start()
    assert started.wait(timeout=1.0)
    assert background.start()
    release.set()
    assert background.wait(timeout_sec=1.0)
    assert calls == ["run"]


def test_background_action_wait_before_start_runs_synchronously():
    from hpp_exec import BackgroundAction

    calls = []

    background = BackgroundAction(lambda: calls.append("run") or True)

    assert background.wait()
    assert calls == ["run"]


def test_background_action_reports_false_and_exceptions_as_failure():
    from hpp_exec import BackgroundAction

    def raise_error():
        raise RuntimeError("boom")

    assert not BackgroundAction(lambda: False).wait()
    assert not BackgroundAction(raise_error).wait()


def test_background_action_can_restart_after_completion():
    from hpp_exec import BackgroundAction

    calls = []

    def action():
        calls.append(time.monotonic())
        return True

    background = BackgroundAction(action)

    assert background.start()
    assert background.wait(timeout_sec=1.0)
    assert background.start()
    assert background.wait(timeout_sec=1.0)
    assert len(calls) == 2
