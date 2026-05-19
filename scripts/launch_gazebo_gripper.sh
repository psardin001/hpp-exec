#!/bin/bash
#
# Launch Gazebo with FR3 robot + gripper and ros2_control
#
# This launches the FR3 with hand:=true so the gripper fingers are
# available in Gazebo, then spawns both arm and gripper controllers.
#
# Usage (inside hpp-exec container):
#   ./hpp-exec/scripts/launch_gazebo_gripper.sh
#

set -e

echo "============================================"
echo "Launching Gazebo with FR3 Robot + Gripper"
echo "============================================"

source /opt/ros/jazzy/setup.bash
source /opt/franka_ws/install/setup.bash

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo ""
echo "Starting Gazebo simulation with gripper..."
echo "This may take 20-30 seconds to fully load."
echo ""

# Launch Gazebo WITH gripper (load_gripper:=true)
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py load_gripper:=true &
GAZEBO_PID=$!

# Wait for controller_manager to appear
echo "Waiting for controller_manager service..."
for i in $(seq 1 30); do
    if ros2 service list 2>/dev/null | grep -q /controller_manager/list_controllers; then
        echo "  controller_manager found after ${i}s"
        break
    fi
    if ! kill -0 $GAZEBO_PID 2>/dev/null; then
        echo ""
        echo "ERROR: Gazebo process died!"
        echo "Try running without gripper first to check if Gazebo works:"
        echo "  ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py"
        echo ""
        echo "If that also crashes, this is a Gazebo/GPU issue (not gripper-related)."
        echo "If it works, the issue is with load_gripper:=true."
        exit 1
    fi
    sleep 1
done

# Extra wait for controller_manager to be fully ready
sleep 3

# Load joint_trajectory_controller for the arm
echo "Loading arm joint_trajectory_controller..."
ros2 param set /controller_manager joint_trajectory_controller.type \
    "joint_trajectory_controller/JointTrajectoryController" || {
    echo "ERROR: Could not set controller type. Is controller_manager running?"
    echo "Check if Gazebo is still alive. Try: ros2 service list | grep controller_manager"
    exit 1
}
ros2 run controller_manager spawner joint_trajectory_controller \
    --param-file "${SCRIPT_DIR}/controllers_gripper.yaml" &
ARM_SPAWNER_PID=$!

# Wait for arm controller to be loaded before spawning gripper
wait $ARM_SPAWNER_PID || {
    echo "WARNING: arm controller spawner had issues, continuing anyway..."
}

# Load gripper controller (JointTrajectoryController for finger joint)
echo "Loading gripper_controller..."
ros2 param set /controller_manager gripper_controller.type \
    "joint_trajectory_controller/JointTrajectoryController"
ros2 run controller_manager spawner gripper_controller \
    --param-file "${SCRIPT_DIR}/controllers_gripper.yaml" &
GRIPPER_SPAWNER_PID=$!

wait $GRIPPER_SPAWNER_PID || {
    echo "WARNING: gripper controller spawner had issues"
}

echo ""
echo "============================================"
echo "Gazebo + gripper ready!"
echo ""
echo "Verify controllers are active:"
echo "  ros2 control list_controllers"
echo ""
echo "In another terminal:"
echo "  docker exec -it hpp-exec bash"
echo "  cd ~/devel/src/hpp_tutorial/tutorial_7 && python -i init.py"
echo "============================================"

wait $GAZEBO_PID
