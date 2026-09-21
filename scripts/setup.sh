#!/usr/bin/env bash
set -e

FRANKA_WS="${FRANKA_WS:-$HOME/ws_moveit}"

echo "===== FRANKA SETUP ====="
echo "Workspace: $FRANKA_WS"

if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "ERROR: ROS 2 Jazzy was not found."
    exit 1
fi

source /opt/ros/jazzy/setup.bash

cd "$FRANKA_WS"

echo
echo "===== INSTALL ROS DEPENDENCIES ====="
rosdep install \
    --from-paths src/FRANKA \
    --ignore-src \
    -r \
    -y

echo
echo "===== BUILD FRANKA PACKAGES ====="
colcon build \
    --packages-select panda_moveit_config franka_motion_app

echo
echo "===== SETUP COMPLETE ====="
echo "Run:"
echo "source $FRANKA_WS/install/setup.bash"
