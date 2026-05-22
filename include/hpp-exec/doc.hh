//
// Copyright (c) 2026 CNRS
// Author: Paul Sardin
//
// BSD 2-Clause License

/// \mainpage hpp-exec
/// \anchor hpp_exec_documentation
///
/// \section hpp_exec_overview Overview
///
/// \c hpp-exec is a thin Python layer that takes a path produced by HPP and
/// runs it on a real or simulated robot through \c ros2_control. It does
/// **no planning** of its own: you write your HPP script using \c pyhpp,
/// sample the resulting path into a list of configurations, then hand them
/// over to \c hpp-exec to be packaged as a \c JointTrajectory message and
/// executed via the \c FollowJointTrajectory action.
///
/// The package source code is on
/// <a href="https://github.com/humanoid-path-planner/hpp-exec">GitHub</a>.
/// End-to-end examples are provided as tutorials 6 and 7 of
/// <a href="https://github.com/humanoid-path-planner/hpp-tutorial">hpp-tutorial</a>.
///
/// \section hpp_exec_workflow Typical workflow
///
/// A minimal usage looks like this:
///
/// \code{.py}
/// import numpy as np
/// from pyhpp.core import TrapezoidalTimeParameterization
/// from hpp_exec import send_trajectory
///
/// # 1. Plan with HPP (using pyhpp directly).
/// path = planner.solve()
///
/// # 2. Add timing in HPP, then sample the timed path into waypoints.
/// optimizer = TrapezoidalTimeParameterization(problem)
/// optimizer.maxVelocity = 0.5
/// optimizer.maxAcceleration = 0.5
/// timed_path = optimizer.optimize(path)
///
/// n = 100
/// configs = [
///     np.array(timed_path(t * timed_path.length() / n)[0])
///     for t in range(n + 1)
/// ]
/// times = [t * timed_path.length() / n for t in range(n + 1)]
///
/// # 3. Send to ros2_control.
/// send_trajectory(
///     configs, times,
///     joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
/// )
/// \endcode
///
/// The \c configs / \c times lists are the only data structure exchanged
/// between HPP and \c hpp-exec. \c configs holds full HPP configuration
/// vectors (which may include object DOFs, gripper fingers, etc.); the
/// \c joint_indices argument selects which entries belong to the arm. The
/// \c times list is the timestamp of each config in seconds.
///
/// \section hpp_exec_api Quick API reference
///
/// Import the public API from \c hpp_exec:
///
/// \code{.py}
/// from hpp_exec import (
///     Segment,
///     send_trajectory,
///     send_trajectory_async,
///     execute_segments,
///     segments_from_graph,
///     format_segments,
///     print_segments,
///     configs_to_joint_trajectory,
///     extract_joint_config,
/// )
/// \endcode
///
/// Main execution functions:
///
/// \code{.py}
/// send_trajectory(
///     configs: list[np.ndarray],
///     times: list[float],
///     joint_names: list[str],
///     controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
///     joint_indices: list[int] | None = None,
/// ) -> bool
///
/// send_trajectory_async(
///     configs: list[np.ndarray],
///     times: list[float],
///     joint_names: list[str],
///     controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
///     joint_indices: list[int] | None = None,
/// )
///
/// execute_segments(
///     segments: list[Segment],
///     configs: list[np.ndarray],
///     times: list[float],
///     joint_names: list[str],
///     joint_indices: list[int] | None = None,
///     controller_topic: str = "/joint_trajectory_controller/follow_joint_trajectory",
/// ) -> bool
/// \endcode
///
/// Segment and grasp helpers:
///
/// \code{.py}
/// Segment(
///     start_index: int,
///     end_index: int,
///     pre_actions: list[Callable[[], bool]] = [],
///     post_actions: list[Callable[[], bool]] = [],
///     state_before: str = "",
///     state_after: str = "",
///     actual_state_before: str = "",
///     actual_state_after: str = "",
/// )
///
/// segments_from_graph(
///     path,
///     graph,
///     n_per_unit: int = 50,
///     min_samples: int = 50,
///     sample_params: Iterable[float] | None = None,
/// ) -> tuple[list[np.ndarray], list[float], list[Segment]]
///
/// format_segments(segments: Iterable[Segment]) -> str
/// print_segments(segments: Iterable[Segment]) -> None
/// \endcode
///
/// Lower-level conversion utilities:
///
/// \code{.py}
/// configs_to_joint_trajectory(
///     configs: list[np.ndarray],
///     times: list[float],
///     joint_names: list[str],
///     joint_indices: list[int] | None = None,
///     velocities: list[np.ndarray] | None = None,
///     accelerations: list[np.ndarray] | None = None,
/// ) -> trajectory_msgs.msg.JointTrajectory
///
/// extract_joint_config(
///     hpp_config: np.ndarray,
///     n_joints: int,
///     offset: int = 0,
/// ) -> list[float]
/// \endcode
///
/// \section hpp_exec_time Time parameterization
///
/// HPP paths are parameterized by a path parameter \f$s\f$, not by time.
/// Before sending a trajectory to \c ros2_control, the parameter values have
/// to be turned into seconds by HPP. \c hpp-exec always assumes that the
/// \c times list already contains timestamps in seconds. Use an HPP path
/// optimizer such as \c SimpleTimeParameterization or
/// \c TrapezoidalTimeParameterization, then sample the optimized path and pass
/// those sample times unchanged to \c send_trajectory.
///
/// The per-point velocity is set to zero only at the first and last waypoint;
/// intermediate velocities are left empty so that the joint trajectory
/// controller smooths between configurations.
///
/// \c send_trajectory returns \c True only when the action goal is accepted
/// and the controller reports a successful \c FollowJointTrajectory result.
/// It returns \c False when the action server is unavailable, the goal is
/// rejected, execution times out, or the controller reports a non-success
/// result code.
///
/// \subsection hpp_exec_time_async Async sending
///
/// \c send_trajectory blocks until the controller reports completion, which
/// is the right default for sequencing actions. When you need to send a
/// trajectory and do something else in parallel (e.g. monitor a sensor while
/// the arm is moving), \c send_trajectory_async returns immediately with the
/// \c (future, node) pair from the underlying \c FollowJointTrajectory action
/// client. The caller is responsible for spinning the node, reading the
/// future, and shutting things down. This is a convenience wrapper around
/// \c rclpy.action.ActionClient.send_goal_async; if you need finer control,
/// build the action client yourself.
///
/// \subsection hpp_exec_time_controller_topic The controller_topic argument
///
/// All sending functions default to
/// \c "/joint_trajectory_controller/follow_joint_trajectory", which is the
/// topic exposed by a \c ros2_control \c JointTrajectoryController spawned
/// under its default name. If your controller is registered under a different
/// name (e.g. \c gripper_controller, or a per-arm \c left_arm_controller in a
/// dual-arm setup), pass the corresponding topic explicitly. Use
/// \c "ros2 control list_controllers" to discover the active controllers and
/// their action topics.
///
/// \section hpp_exec_segments Segments and pre/post actions
///
/// A \c FollowJointTrajectory action is a one-shot command: once the goal
/// is sent, the controller executes it from beginning to end without any
/// way of pausing in the middle. Manipulation tasks, however, need to do
/// things between trajectory pieces --- most commonly close or open a
/// gripper at a grasp or release point. \c hpp-exec handles this by
/// splitting the trajectory into **segments** and attaching **actions** to
/// segment boundaries.
///
/// A \c Segment is a slice <tt>[start_index, end_index)</tt> of the
/// \c configs / \c times lists, with two optional lists of callables:
///
/// \li \c pre_actions are run **before** the segment's trajectory is sent.
/// \li \c post_actions are run **after** the segment's trajectory has
///     finished executing on the controller.
///
/// An action is any zero-argument callable returning a \c bool: bound
/// methods (e.g.\ <tt>gripper.close</tt>), free functions, or lambdas all
/// work. The action returns \c True on success and \c False on failure;
/// any \c False short-circuits \c execute_segments, which then aborts
/// the rest of the plan.
///
/// Concretely, \c execute_segments iterates over the segments and for
/// each one runs all \c pre_actions, sends the corresponding sub-
/// trajectory and waits for its completion, then runs all \c post_actions.
/// Segment timestamps are normalized so that each sub-trajectory starts
/// at <tt>t = 0</tt>, which is what \c FollowJointTrajectory expects.
/// A segment containing fewer than two configurations is treated as a
/// pure action point: the trajectory is skipped and only the actions run.
///
/// \c execute_segments assumes that the \c times list already contains
/// seconds for the whole path. Apply time parameterization in HPP before
/// calling \c segments_from_graph; each segment will then inherit the sampled
/// timing and be normalized to start at <tt>t = 0</tt> before it is sent.
///
/// \section hpp_exec_graph_segments Graph segments
///
/// For paths produced by an HPP manipulation planner, useful execution
/// boundaries are encoded in the HPP \c PathVector and constraint graph.
/// The \c segments_from_graph helper inspects each continuous subpath,
/// asks the graph which transition owns it with
/// <tt>graph.transitionAtParam(path, s)</tt>, inserts every graph boundary
/// into the sampled waypoint list, and returns one \c Segment per HPP graph
/// segment.
///
/// The printed table shows the transition names, states, timing, and action
/// counts for each segment.
///
/// \code{.py}
/// from hpp_exec import execute_segments, print_segments
/// from hpp_exec.graph_segments import segments_from_graph
///
/// configs, times, segments = segments_from_graph(path, graph)
/// print_segments(segments)
///
/// # For this known pick-and-place graph:
/// segments[2].pre_actions.append(gripper.close)
/// segments[5].pre_actions.append(gripper.open)
///
/// execute_segments(
///     segments,
///     configs,
///     times,
///     joint_names=[...],
/// )
/// \endcode
///
/// Each segment carries:
///
/// \li \c start_index, \c end_index: indices into the sampled \c configs /
///     \c times arrays.
/// \li \c start_time, \c end_time, \c duration: timing information in
///     seconds.
/// \li \c transition_name: graph edge name.
/// \li \c state_before, \c state_after: graph state names returned by
///     \c graph.getNodesConnectedByTransition(edge).
/// \li \c actual_state_before, \c actual_state_after: states observed by
///     sampling the path near the start and end of the segment with
///     \c graph.getStateFromConfiguration(q).
///
/// \subsection hpp_exec_grasps_initial Synchronizing the initial state
///
/// If the real-world initial state of the gripper is uncertain (a previous
/// run that aborted mid-trajectory, a simulator that spawned the fingers at
/// an arbitrary pose), prepend the appropriate action to the first segment:
///
/// \code{.py}
/// segments[0].pre_actions.insert(0, gripper.open)
/// \endcode
///
/// Because \c send_trajectory blocks until its action reports
/// completion, the arm motion will only start once the gripper has
/// reached the requested state.
///
/// If the action can run while the arm is already travelling, wrap it in
/// \c BackgroundAction. Append \c start where the action may begin and
/// \c wait where the plan must synchronize:
///
/// \code{.py}
/// from hpp_exec import BackgroundAction
///
/// background_open = BackgroundAction(open_gripper, name="open_gripper")
/// segments[0].pre_actions.append(background_open.start)
/// segments[2].pre_actions.append(background_open.wait)
/// segments[2].pre_actions.append(gripper.close)
/// \endcode
///
/// \c BackgroundAction.start returns immediately after launching the action
/// in a daemon thread. \c BackgroundAction.wait blocks until that action
/// reports success. If \c wait is called before \c start, it runs the action
/// synchronously, which keeps it safe as a blocking fallback.
///
/// \subsection hpp_exec_grasps_custom_actions Writing segment actions
///
/// Real grippers, simulated grippers, detachable Gazebo objects, cameras, or
/// force sensors all have different ROS interfaces. Segment actions use a
/// small contract: a zero-argument callable that returns \c True on success
/// and \c False on failure.
///
/// For a Gazebo gripper driven by a \c JointTrajectoryController, the action
/// can call \c send_trajectory with the finger joint names and a short
/// two-point trajectory:
///
/// \code{.py}
/// def open_gripper():
///     return send_trajectory(
///         [np.array([0.0]), np.array([0.035])],
///         [0.0, 0.5],
///         joint_names=["fr3_finger_joint1"],
///         controller_topic="/gripper_controller/follow_joint_trajectory",
///     )
/// \endcode
///
/// For a real Franka gripper, the action can instead call the native
/// \c franka_msgs \c Move and \c Grasp actions. The segment code is the
/// same; only the callable changes:
///
/// \code{.py}
/// gripper = FrankaGripperController("fr3")
/// configs, times, segments = segments_from_graph(timed_path, graph)
/// print_segments(segments)
/// segments[0].pre_actions.insert(0, gripper.open)
/// segments[2].pre_actions.append(gripper.close)
/// segments[5].pre_actions.append(gripper.open)
/// execute_segments(
///     segments,
///     configs,
///     times,
///     joint_names=[...],
/// )
/// \endcode
///
/// \section hpp_exec_low_level Lower-level helpers
///
/// \c configs_to_joint_trajectory is the function \c send_trajectory uses
/// internally to build the ROS message. Call it directly if you need the
/// \c trajectory_msgs.msg.JointTrajectory object (e.g. to publish to a
/// topic instead of an action, or to attach it to a custom goal). It
/// projects each HPP config down to \c joint_indices, sets per-point
/// velocities to zero only on the first and last waypoint (intermediate
/// waypoints are left empty so the controller smooths between them), and
/// passes \c times through unchanged - the caller is responsible for
/// providing real seconds.
///
/// \c extract_joint_config is a one-line slicing helper that returns
/// <tt>[hpp_config[offset + i] for i in range(n_joints)]</tt> as a list of
/// floats. \c send_trajectory and \c configs_to_joint_trajectory already
/// project configs through \c joint_indices, so most users will not need
/// this; it is exposed for callers that build messages by hand.
///
/// \section hpp_exec_troubleshooting Troubleshooting
///
/// \par "Trajectory controller not available"
/// The \c FollowJointTrajectory action server did not respond within the
/// 10 s wait. Check that the controller is loaded and active with
/// \c "ros2 control list_controllers", and that the topic name passed via
/// \c controller_topic matches one of the listed action endpoints.
///
/// \par "Trajectory goal rejected"
/// The controller accepted the connection but refused the goal. The most
/// common cause is a mismatch between \c joint_names and the joints the
/// controller manages; verify the spelling and order against the controller
/// configuration YAML.
///
/// \par execute_segments aborts on a pre-action
/// The pre-action returned \c False. Run the action standalone in a Python
/// REPL to see which step failed. For \c send_trajectory-based gripper
/// actions, the same "controller not available" / "goal rejected"
/// diagnostics apply against the gripper controller.
///
/// \par The arm starts before the gripper finishes
/// Make sure the gripper action is a synchronous \c send_trajectory call (it
/// blocks until completion) and that it returns \c True only on success. An
/// action returning \c True early - or a fire-and-forget topic publish -
/// will let the arm move before the gripper has reached its target.
/// When using \c BackgroundAction, add the corresponding \c wait action
/// before the segment that requires the completed gripper motion.
///
/// \par segments_from_graph returns a single segment
/// The HPP path contains a single graph segment. Check the table printed by
/// \c print_segments and the \c PathVector produced by the planner if more
/// execution boundaries were expected.
///
/// \section hpp_exec_code_map How the Python code is organized
///
/// The package is intentionally small:
///
/// \li \c actions.py defines reusable helpers for segment actions, including
///     \c BackgroundAction for overlapping a blocking action with arm motion.
/// \li \c segments.py defines the lightweight \c Segment data structure.
/// \li \c trajectory_utils.py extracts selected joints from full HPP
///     configuration vectors and converts them to a ROS 2
///     \c JointTrajectory message.
/// \li \c ros2_sender.py creates the \c FollowJointTrajectory action client,
///     sends the trajectory, waits for the result, and implements the
///     \c Segment / \c execute_segments loop.
/// \li \c graph_segments.py reads HPP path segments and constraint-graph
///     transitions, samples the path at graph boundaries, formats segment
///     tables, and turns them into executable \c Segment objects.
///
/// The important separation is that HPP remains responsible for planning and
/// time parameterization, \c hpp-exec is responsible for packaging and
/// sequencing execution, and the robot application remains responsible for
/// robot-specific actions such as opening a gripper or attaching a simulated
/// object.
