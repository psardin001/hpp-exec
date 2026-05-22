"""Helpers for segment actions."""

from __future__ import annotations

import logging
import threading
from typing import Callable

logger = logging.getLogger(__name__)


def _callable_name(action: Callable[[], bool]) -> str:
    return (
        getattr(action, "__qualname__", None)
        or getattr(action, "__name__", None)
        or repr(action)
    )


class BackgroundAction:
    """Run a blocking action in the background and synchronize later."""

    def __init__(self, action: Callable[[], bool], name: str | None = None):
        self.action = action
        self.name = name or _callable_name(action)
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._result: bool | None = None
        self._exception: BaseException | None = None

    def start(self) -> bool:
        """Start the action in a daemon thread without waiting for it."""
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return True

            self._result = None
            self._exception = None
            thread = threading.Thread(
                target=self._run,
                name=f"BackgroundAction[{self.name}]",
                daemon=True,
            )
            self._thread = thread

        try:
            thread.start()
        except RuntimeError:
            with self._lock:
                if self._thread is thread:
                    self._thread = None
            logger.exception("Failed to start background action '%s'", self.name)
            return False

        return True

    def wait(self, timeout_sec: float | None = None) -> bool:
        """Wait for the background action, or run it now if it was not started."""
        with self._lock:
            thread = self._thread

        if thread is None:
            return self._run_sync()

        thread.join(timeout=timeout_sec)
        if thread.is_alive():
            logger.error("Background action '%s' timed out", self.name)
            return False

        with self._lock:
            exception = self._exception
            result = self._result

        if exception is not None:
            return False
        return bool(result)

    def _run(self) -> None:
        result = False
        exception = None
        try:
            result = bool(self.action())
        except Exception as exc:
            exception = exc
            logger.exception("Background action '%s' failed", self.name)

        with self._lock:
            self._result = result
            self._exception = exception

    def _run_sync(self) -> bool:
        try:
            return bool(self.action())
        except Exception:
            logger.exception("Background action '%s' failed", self.name)
            return False
