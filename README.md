# swervebotSim

<p align="center">
  <img width="400" height="300" alt="swervebot" src="https://github.com/user-attachments/assets/8fcf753a-40f0-4492-bbeb-70f6b778aca6" />
</p>

swervebotSim is a ROS 2 workspace for simulating and testing a swerve-drive mobile robot in Gazebo. It combines the robot model, controller, teleoperation, scan merging, navigation, and local planning pieces needed to drive the robot through the included depot environment.

## Overview

The workspace is organized as a set of ROS 2 packages that work together as a full navigation stack:

- `swervebot_description` contains the robot URDF/Xacro, Gazebo world assets, bridge configuration, and simulation launch files.
- `swervebot_controller` converts `cmd_vel` commands into wheel speed and caster steering commands, and publishes odometry and TF data.
- `dbot_teleop` provides a keyboard teleoperation node for manual control.
- `scan_merger` fuses multiple laser scans into a single `LaserScan` topic for downstream navigation and planning.
- `swervebot_navigation` launches Navigation2 with the depot map and parameter file.
- `neo_mpc_planner2` provides a custom MPC-based local planner / optimizer server for Nav2.
- `robot_localization`, `neo_srvs2`, `neo_common2`, and `neo_nav2_py_costmap2D` supply localization, service definitions, shared utilities, and costmap helpers used by the stack.

The repository also includes a depot map, a Gazebo depot world, and AprilTag / scene assets so the robot can be exercised in a realistic simulated environment.
