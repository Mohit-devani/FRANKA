# FRANKA

A ROS 2 + MoveIt learning project for understanding and implementing a complete robotic manipulation pipeline using the Franka Panda robot.

## Current Pipeline

The project currently explores:

- MoveIt motion planning
- Cartesian pose goals
- Cartesian approach motion
- Collision-aware motion
- Reachability testing
- Planning-scene objects
- Cube spawning
- Gripper control
- Grasp approach
- Object attachment
- Attached-object lifting

## ROS 2 Package

`franka_motion_app`

## Structure

- `src/` — C++ manipulation nodes
- `launch/` — ROS 2 launch files
- `include/` — package headers
- `CMakeLists.txt` — build configuration
- `package.xml` — ROS 2 dependencies

## Purpose

This repository is being developed as a practical robotics engineering project to understand the manipulation pipeline from motion planning and Cartesian control through collision-aware grasping and object manipulation using ROS 2 and MoveIt.
