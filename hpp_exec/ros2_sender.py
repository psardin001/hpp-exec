"""
Send trajectories to ros2_control.

Simple API for executing HPP-generated trajectories on ROS2 robots.

Example:
    from hpp_exec import send_trajectory, execute_segments, Segment

    # Your HPP script generates configs...
    configs = [np.array([0, 0, 0, 0, 0, 0]), np.array([1, 1, 1, 1, 1, 1])]
    times = [0.0, 2.0]

    # Simple execution (no actions between segments):
    send_trajectory(
        configs, times,
        joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    )

    # With pre/post actions between segments:
    segments = [
        Segment(0, 150),
        Segment(150, 300, pre_actions=[gripper.close]),
        Segment(300, 462, pre_actions=[gripper.open]),
    ]
    execute_segments(segments, configs, times, joint_names=[...])

    # Or attach actions by HPP graph transition name or object when executing:
    execute_segments(
        segments,
        configs,
        times,
        joint_names=[...],
        pre_actions_by_transition={grasp_transition: gripper.close},
        post_actions_by_transition={"release transition": gripper.open},
    )
"""

import logging
import threading
from itertools import count
from typing import Callable, List, Mapping, Optional, Protocol, Sequence

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.type_support import check_for_type_support

from hpp_exec.actions import BackgroundAction
from hpp_exec.segments import Segment
from hpp_exec.trajectory_utils import configs_to_joint_trajectory

logger = logging.getLogger(__name__)
_NODE_IDS = count()
_RCLPY_INIT_LOCK = threading.Lock()
_TYPE_SUPPORT_LOCK = threading.Lock()
Action = Callable[[], bool]


class TransitionLike(Protocol):
    def name(self) -> object:
        ...


TransitionKey = str | TransitionLike
TransitionActionMap = Mapping[TransitionKey, Action | Sequence[Action]]


def _ensure_rclpy_initialized() -> None:
    if rclpy.ok():
        return

    with _RCLPY_INIT_LOCK:
        if not rclpy.ok():
            rclpy.init()


def _ensure_trajectory_type_support() -> None:
    with _TYPE_SUPPORT_LOCK:
        check_for_type_support(FollowJointTrajectory)


def _action_name(action) -> str:
    action_owner = getattr(action, "__self__", None)
    if isinstance(action_owner, BackgroundAction):
        method_name = getattr(action, "__name__", None) or repr(action)
        return f"{action_owner.name}.{method_name}"

    return (
        getattr(action, "__qualname__", None)
        or getattr(action, "__name__", None)
        or repr(action)
    )


def _transition_name(transition: TransitionKey) -> str:
    if isinstance(transition, str):
        return transition
    return str(transition.name())


def _normalize_transition_action_map(
    actions_by_transition: TransitionActionMap | None,
) -> dict[str, Action | Sequence[Action]]:
    if not actions_by_transition:
        return {}

    return {
        _transition_name(transition): actions
        for transition, actions in actions_by_transition.items()
    }


def _actions_for_transition(
    actions_by_transition: Mapping[str, Action | Sequence[Action]],
    transition_name: str,
) -> list[Action]:
    if not actions_by_transition:
        return []

    actions = actions_by_transition.get(transition_name)
    if actions is None:
        return []
    if callable(actions):
        return [actions]
    return list(actions)


def _unmatched_transition_names(
    segments: Sequence[Segment],
    *action_maps: Mapping[str, Action | Sequence[Action]],
) -> set[str]:
    transition_names = {segment.transition_name for segment in segments}
    action_transition_names = {
        transition_name
        for action_map in action_maps
        if action_map
        for transition_name in action_map
    }
    return action_transition_names - transition_names


class _TrajectorySenderNode(Node):
    """Internal node for sending trajectories."""

    def __init__(
        self,
        controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
    ):
        super().__init__(f"hpp_trajectory_sender_{next(_NODE_IDS)}")
        _ensure_trajectory_type_support()
        self.client = ActionClient(self, FollowJointTrajectory, controller_topic)
        self._result = None

    def send_and_wait(self, trajectory, timeout_sec: float = 60.0) -> bool:
        """Send trajectory and wait for execution to complete."""
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        try:
            if not self.client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error("Trajectory controller not available")
                return False

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = trajectory

            # Compute expected duration from last trajectory point
            last_point = trajectory.points[-1]
            duration = (
                last_point.time_from_start.sec
                + last_point.time_from_start.nanosec * 1e-9
            )

            self.get_logger().info(
                f"Sending trajectory: {len(trajectory.points)} points, "
                f"{len(trajectory.joint_names)} joints, {duration:.1f}s"
            )

            future = self.client.send_goal_async(goal)
            executor.spin_until_future_complete(future, timeout_sec=10.0)

            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("Trajectory goal rejected")
                return False

            self.get_logger().info("Trajectory accepted, executing...")

            # Wait for execution to complete
            result_future = goal_handle.get_result_async()
            executor.spin_until_future_complete(result_future, timeout_sec=timeout_sec)

            result = result_future.result()
            if result is None:
                self.get_logger().error("Trajectory execution timed out")
                return False

            if result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(
                    "Trajectory execution failed with error code %d: %s",
                    result.result.error_code,
                    result.result.error_string,
                )
                return False

            self.get_logger().info("Trajectory execution complete")
            return True
        finally:
            executor.remove_node(self)
            executor.shutdown()


def send_trajectory(
    configs: List[np.ndarray],
    times: List[float],
    joint_names: List[str],
    controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
    joint_indices: Optional[List[int]] = None,
) -> bool:
    """
    Send a trajectory to ros2_control.

    Args:
        configs: List of configuration vectors (numpy arrays).
        times: List of timestamps in seconds.
        joint_names: ROS2 joint names in order.
        controller_topic: FollowJointTrajectory action topic.
        joint_indices: Indices to extract from each config (default: 0..len(joint_names)).

    Returns:
        True if trajectory executed successfully.

    Example:
        # From your HPP script:
        path = planner.solve()
        timed_path = time_optimizer.optimize(path)
        configs = [
            np.array(timed_path(t)[0])
            for t in np.linspace(0, timed_path.length(), 100)
        ]
        times = list(np.linspace(0, timed_path.length(), 100))

        send_trajectory(
            configs, times,
            joint_names=["shoulder_pan", "shoulder_lift", "elbow", ...],
        )
    """
    # Convert to ROS2 message
    trajectory = configs_to_joint_trajectory(
        configs,
        times,
        joint_names,
        joint_indices=joint_indices,
    )

    _ensure_rclpy_initialized()

    node = _TrajectorySenderNode(controller_topic)
    try:
        return node.send_and_wait(trajectory)
    finally:
        node.destroy_node()


def send_trajectory_async(
    configs: List[np.ndarray],
    times: List[float],
    joint_names: List[str],
    controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
    joint_indices: Optional[List[int]] = None,
):
    """
    Send trajectory without waiting for completion.

    Returns the goal handle for later status checking.
    Caller is responsible for ROS2 lifecycle (rclpy.init/shutdown).

    Args:
        times: List of timestamps in seconds.
    """
    trajectory = configs_to_joint_trajectory(
        configs,
        times,
        joint_names,
        joint_indices=joint_indices,
    )

    _ensure_rclpy_initialized()

    node = _TrajectorySenderNode(controller_topic)

    if not node.client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("Trajectory controller not available")
        return None

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = trajectory

    future = node.client.send_goal_async(goal)
    return future, node


# ---------------------------------------------------------------------------
# Segment-based execution with pre/post action hooks
# ---------------------------------------------------------------------------


def execute_segments(
    segments: List[Segment],
    configs: List[np.ndarray],
    times: List[float],
    joint_names: List[str],
    joint_indices: Optional[List[int]] = None,
    controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
    *,
    pre_actions_by_transition: TransitionActionMap | None = None,
    post_actions_by_transition: TransitionActionMap | None = None,
) -> bool:
    """Execute trajectory segments with pre/post action hooks.

    For each segment:
        1. Run all pre_actions (stop on first failure)
        2. Send the arm trajectory for this segment
        3. Run all post_actions (stop on first failure)

    Args:
        segments: Ordered list of Segment objects defining trajectory slices
            and their associated actions.
        configs: Full HPP configuration vectors.
        times: Timestamps in seconds for each config.
        joint_names: ROS2 joint names for the arm.
        joint_indices: Indices of arm DOFs in the HPP config vector.
            Default: 0..len(joint_names).
        controller_topic: FollowJointTrajectory action topic.
        pre_actions_by_transition: Optional mapping from HPP graph transition
            names or transition objects to one action or an ordered sequence of
            actions to run before matching segments.
        post_actions_by_transition: Optional mapping from HPP graph transition
            names or transition objects to one action or an ordered sequence of
            actions to run after matching segments.

    Returns:
        True if all segments and actions succeeded.
    """
    pre_actions_by_transition = _normalize_transition_action_map(
        pre_actions_by_transition
    )
    post_actions_by_transition = _normalize_transition_action_map(
        post_actions_by_transition
    )

    unmatched_transition_names = _unmatched_transition_names(
        segments,
        pre_actions_by_transition,
        post_actions_by_transition,
    )
    if unmatched_transition_names:
        logger.error(
            "Transition action map contains unknown transition names: %s",
            ", ".join(sorted(unmatched_transition_names)),
        )
        return False

    for i, segment in enumerate(segments):
        # 1. Pre-actions
        pre_actions = [
            *segment.pre_actions,
            *_actions_for_transition(
                pre_actions_by_transition,
                segment.transition_name,
            ),
        ]
        for action in pre_actions:
            action_name = _action_name(action)
            logger.info("Segment %d: running pre-action '%s'", i, action_name)
            if not action():
                logger.error("Segment %d: pre-action '%s' failed", i, action_name)
                return False

        # 2. Send arm trajectory
        seg_configs = configs[segment.start_index : segment.end_index]
        seg_times = times[segment.start_index : segment.end_index]

        if len(seg_configs) >= 2:
            # Normalize times to start from 0
            t0 = seg_times[0]
            seg_times = [t - t0 for t in seg_times]

            logger.info(
                "Segment %d: sending %d configs (%.2fs)",
                i,
                len(seg_configs),
                seg_times[-1],
            )

            success = send_trajectory(
                seg_configs,
                seg_times,
                joint_names,
                controller_topic=controller_topic,
                joint_indices=joint_indices,
            )

            if not success:
                logger.error("Segment %d: arm trajectory failed", i)
                return False
        else:
            logger.info("Segment %d: single point, skipping trajectory", i)

        # 3. Post-actions
        post_actions = [
            *segment.post_actions,
            *_actions_for_transition(
                post_actions_by_transition,
                segment.transition_name,
            ),
        ]
        for action in post_actions:
            action_name = _action_name(action)
            logger.info("Segment %d: running post-action '%s'", i, action_name)
            if not action():
                logger.error("Segment %d: post-action '%s' failed", i, action_name)
                return False

    logger.info("All %d segments completed successfully", len(segments))
    return True
