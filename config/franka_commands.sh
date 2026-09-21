#!/usr/bin/env bash

# ============================================================
# FRANKA / MoveIt shared development commands
# Source this file from ~/.bashrc or ~/.bash_aliases
# ============================================================

export FRANKA_WS="${FRANKA_WS:-$HOME/ws_moveit}"
export FRANKA_REPO="$FRANKA_WS/src/FRANKA"

alias launchit='ros2 launch panda_moveit_config demo.launch.py'
alias cartesian_pose='ros2 launch franka_motion_app cartesian_pose_goal.launch.py'

alias build_franka='cd "$FRANKA_WS" && source /opt/ros/jazzy/setup.bash && colcon build --packages-select panda_moveit_config franka_motion_app && source "$FRANKA_WS/install/setup.bash"'

alias franka_sweep='ros2 launch franka_motion_app reachability_sweep.launch.py'
alias franka_collision='ros2 launch franka_motion_app collision_aware_pose.launch.py'
alias franka_approach='ros2 launch franka_motion_app cartesian_approach.launch.py'
alias franka_gripper='ros2 launch franka_motion_app gripper_cycle.launch.py'
alias franka_cube='ros2 launch franka_motion_app spawn_cube.launch.py'
alias franka_grasp='ros2 launch franka_motion_app grasp_attach.launch.py'
alias franka_grasp_approach='ros2 launch franka_motion_app grasp_approach.launch.py'
alias franka_lift='ros2 launch franka_motion_app lift_attached.launch.py'
alias franka_day2_finish='ros2 launch franka_motion_app day2_cleanup.launch.py'

# Convenience Git commands
alias franka_repo='cd "$FRANKA_REPO"'
alias franka_status='git -C "$FRANKA_REPO" status'
alias franka_pull='git -C "$FRANKA_REPO" pull --ff-only'
