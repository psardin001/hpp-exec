# Gazebo Gripper Launch Helper

Launch FR3 with a gripper and ros2_control controllers in Gazebo.

## Setup

```bash
cd ~/devel/hpp-exec
./run.sh
```

## Run

```bash
./hpp-exec/scripts/launch_gazebo_gripper.sh
```

Wait until you see "Gazebo + gripper ready!".

Use `hpp_tutorial/tutorial_7` for the pick-and-place execution script.

## Troubleshooting

If the test fails with "Trajectory controller not available", check controllers:

```bash
ros2 control list_controllers
ros2 action list
```

You should see both `joint_trajectory_controller` and `gripper_controller` active.
If needed, spawn them manually:

```bash
SCRIPT_DIR=$HOME/devel/hpp-exec/scripts
ros2 param set /controller_manager joint_trajectory_controller.type "joint_trajectory_controller/JointTrajectoryController"
ros2 run controller_manager spawner joint_trajectory_controller --param-file "$SCRIPT_DIR/controllers_gripper.yaml"
ros2 param set /controller_manager gripper_controller.type "joint_trajectory_controller/JointTrajectoryController"
ros2 run controller_manager spawner gripper_controller --param-file "$SCRIPT_DIR/controllers_gripper.yaml"
```
