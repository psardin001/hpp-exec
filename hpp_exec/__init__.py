"""
hpp_exec - ROS2 execution utilities for HPP trajectories.

Send HPP-generated trajectories to ros2_control.

Example:
    from hpp_exec import send_trajectory

    # Your HPP script generates configs...
    configs = [...]
    times = [...]

    # Execute on robot (times are already seconds)
    send_trajectory(
        configs, times,
        joint_names=["joint1", "joint2", ...],
    )
"""

from hpp_exec.graph_segments import format_segments, print_segments, segments_from_graph
from hpp_exec.segments import Segment

__version__ = "0.1.0"

_ROS_MODULES = {"rclpy", "control_msgs", "trajectory_msgs", "builtin_interfaces"}
_ROS_IMPORT_ERROR = None

try:
    from hpp_exec.ros2_sender import (
        execute_segments,
        send_trajectory,
        send_trajectory_async,
    )
    from hpp_exec.trajectory_utils import (
        configs_to_joint_trajectory,
        extract_joint_config,
    )
except ModuleNotFoundError as exc:
    if exc.name not in _ROS_MODULES:
        raise
    _ROS_IMPORT_ERROR = exc

    def _missing_ros2(*args, **kwargs):
        raise ModuleNotFoundError(
            "ROS 2 Python dependencies are required for trajectory execution"
        ) from _ROS_IMPORT_ERROR

    send_trajectory = _missing_ros2
    send_trajectory_async = _missing_ros2
    execute_segments = _missing_ros2
    configs_to_joint_trajectory = _missing_ros2
    extract_joint_config = _missing_ros2

__all__ = [
    "send_trajectory",
    "send_trajectory_async",
    "execute_segments",
    "Segment",
    "configs_to_joint_trajectory",
    "extract_joint_config",
    "segments_from_graph",
    "format_segments",
    "print_segments",
]
