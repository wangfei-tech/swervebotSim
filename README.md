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

---

# Chapter 1. Environment Setup

## 1.1 Project Overview

This project is based on:

```text
Ubuntu 22.04
ROS 2 Humble
Gazebo Sim / Ignition Gazebo Fortress
ros2_control
gz_ros2_control
Nav2
```

It is mainly used to study and verify:

```text
Four-wheel independent steering
Four-wheel independent drive
Gazebo simulation
ros2_control integration
Nav2 navigation
Dual-LiDAR scan merging
Swerve drive kinematics
Coverage planning
Potential lawn mower robot applications
```

Reference repository:

```text
https://github.com/user247-tai/swervebotSim
```

---

## 1.2 Clone the Repository with Git Submodules

Some directories in the repository are Git submodules, for example:

```text
neo_common2 @ 8d5a3c8
neo_mpc_planner2 @ 1cd77b5
neo_nav2_py_costmap2D @ e74892c
neo_srvs2 @ d181553
robot_localization @ 6db1ac3
```

A directory shown with:

```text
@ commit_hash
```

is usually a Git submodule rather than a normal source directory.

### Clone the repository and all submodules

Recommended:

```bash
git clone --recursive <repository_url>
```

For example:

```bash
git clone --recursive https://github.com/user247-tai/swervebotSim.git
```

### Initialize submodules after cloning

If the main repository has already been cloned:

```bash
cd SwervebotSim
git submodule update --init --recursive
```

Check submodule status:

```bash
git submodule status
```

Status meaning:

```text
-commit    Submodule has not been initialized
+commit    Current submodule commit differs from the commit recorded by the parent repository
 commit    Submodule is in the expected state
```

Check the submodule configuration:

```bash
cat .gitmodules
```

---

## 1.3 ROS 2 Humble CMake Compatibility

Some source packages use modern CMake imported targets such as:

```cmake
tf2_sensor_msgs::tf2_sensor_msgs
nav2_core::nav2_core
```

However, ROS 2 Humble does not export all ROS packages as namespaced CMake targets.

This may cause errors such as:

```text
Target "xxx" links to:

    xxx::xxx

but the target was not found.
```

A more compatible approach for ROS 2 Humble is:

```cmake
find_package(xxx REQUIRED)

ament_target_dependencies(target_name
  xxx
)
```

instead of blindly using:

```cmake
target_link_libraries(target_name
  xxx::xxx
)
```

Note:

> Not every `xxx::xxx` target is invalid. Availability depends on the ROS 2 distribution and package export configuration.

Search all namespaced targets in the workspace:

```bash
grep -R "::" -n src/*/CMakeLists.txt
```

---

## 1.4 Fix `tf2_sensor_msgs::tf2_sensor_msgs` Not Found

Typical error:

```text
Target "scan_merger" links to:

    tf2_sensor_msgs::tf2_sensor_msgs

but the target was not found.
```

Original code:

```cmake
target_link_libraries(scan_merger PUBLIC
  tf2_sensor_msgs::tf2_sensor_msgs
)
```

Recommended Humble-compatible version:

```cmake
find_package(tf2_sensor_msgs REQUIRED)

ament_target_dependencies(scan_merger
  tf2_sensor_msgs
)
```

Example:

```cmake
cmake_minimum_required(VERSION 3.20)
project(scan_merger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(laser_geometry REQUIRED)

add_executable(scan_merger
  src/scan_merger.cpp
)

ament_target_dependencies(scan_merger
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  message_filters
  laser_geometry
)

install(
  TARGETS scan_merger
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

## 1.5 Fix `nav2_core::nav2_core` Not Found

Typical error:

```text
Target "neo_mpc_planner2" links to:

    nav2_core::nav2_core

but the target was not found.
```

Original:

```cmake
target_link_libraries(neo_mpc_planner2
  nav2_core::nav2_core
)
```

Recommended:

```cmake
find_package(nav2_core REQUIRED)

ament_target_dependencies(neo_mpc_planner2
  nav2_core
)
```

For `neo_mpc_planner2`, a more Humble-compatible dependency section can look like:

```cmake
ament_target_dependencies(${library_name}
  rclcpp
  geometry_msgs
  nav_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  angles
  tf2
  tf2_ros
  tf2_eigen
  tf2_sensor_msgs
  tf2_geometry_msgs
  neo_srvs2
)
```

Also make sure the following packages are explicitly found:

```cmake
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
```

---

## 1.6 Install `ros_gz_sim`

If launching the simulation produces:

```text
package 'ros_gz_sim' not found
```

install the ROS-Gazebo integration packages:

```bash
sudo apt update
sudo apt install ros-humble-ros-gz
```

Or install the main packages individually:

```bash
sudo apt install \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-interfaces
```

`ros_gz_sim` is mainly responsible for:

```text
Launching Gazebo Sim
Spawning robot entities
Deleting entities
Controlling the simulation world
Integrating Gazebo startup into ROS 2 launch files
```

Message bridging is mainly handled by:

```text
ros_gz_bridge
```

Example:

```text
Gazebo LiDAR
    ↓
ros_gz_bridge
    ↓
sensor_msgs/msg/LaserScan
```

---

## 1.7 Gazebo Classic, Ignition Gazebo, Gazebo Sim, `ign`, and `gz`

The naming history is easy to confuse.

### Gazebo Classic

Older Gazebo versions use:

```bash
gazebo
gzserver
gzclient
```

ROS integration usually uses:

```text
gazebo_ros
gazebo_ros_pkgs
```

### Ignition Gazebo / Gazebo Sim

The newer simulator was originally called:

```text
Ignition Gazebo
```

and later renamed to:

```text
Gazebo Sim
```

Older command:

```bash
ign gazebo
```

Newer command:

```bash
gz sim
```

ROS 2 Humble commonly uses:

```text
ROS 2 Humble
+
Gazebo Fortress
+
Gazebo Sim 6
```

Therefore, seeing:

```text
ign gazebo ... --force-version 6
```

is normal in a ROS 2 Humble environment.

---

# Chapter 2. Troubleshooting

## 2.1 Gazebo World File Not Found

Typical error:

```text
Unable to find or download file

ign gazebo depot.sdf -r --force-version 6
```

Search for the world file:

```bash
find . -name "depot.sdf"
```

Launch using an absolute path:

```bash
ros2 launch ros_gz_sim gz_sim.launch.py \
  gz_args:="/home/hibo/Personal/SwervebotSim/src/swervebotSim/swervebot_description/worlds/depot.sdf"
```

---

## 2.2 SDF Version Compatibility

Typical error:

```text
Unable to convert from SDF version 1.12 to 1.9
```

Original:

```xml
<sdf version='1.12'>
```

For Gazebo Sim 6 / Fortress, a compatible version may be:

```xml
<sdf version='1.9'>
```

Important:

> Simply changing the SDF version number does not guarantee compatibility. If the file uses elements introduced in SDF 1.10, 1.11, or 1.12, additional modifications may still be required.

---

## 2.3 `model://xxx` Resources Not Found

Typical errors:

```text
Unable to find uri[model://Depot]
Unable to find uri[model://Apriltag36_11_00002]
Unable to find uri[model://cart_model_2]
Unable to find uri[model://A_letter]
Unable to find uri[model://B_letter]
```

Example project structure:

```text
swervebot_description/
├── models/
│   ├── A_letter/
│   ├── Apriltag36_11_00002/
│   ├── Apriltag36_11_00003/
│   ├── Apriltag36_11_00004/
│   ├── Apriltag36_11_00005/
│   ├── Apriltag36_11_00006/
│   ├── Apriltag36_11_00007/
│   ├── B_letter/
│   ├── cart_model_2/
│   └── Depot/
├── urdf/
└── worlds/
```

If the world contains:

```xml
<include>
  <uri>model://Depot</uri>
</include>
```

Gazebo searches for:

```text
${RESOURCE_PATH}/Depot
```

Therefore, add the parent `models` directory to the resource path:

```bash
MODEL_PATH=/home/hibo/Personal/SwervebotSim/src/swervebotSim/swervebot_description/models

export IGN_GAZEBO_RESOURCE_PATH=$MODEL_PATH:$IGN_GAZEBO_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=$MODEL_PATH:$GZ_SIM_RESOURCE_PATH
```

Check:

```bash
echo $IGN_GAZEBO_RESOURCE_PATH
echo $GZ_SIM_RESOURCE_PATH
```

---

## 2.4 Invalid `<physics default="...">` Value

Typical error:

```text
Invalid boolean value
Unable to read attribute[default]
```

Incorrect:

```xml
<physics default="0.02" name="default_physics" type="ode">
```

The `default` attribute is boolean and should be:

```text
true
false
1
0
```

Correct example:

```xml
<physics default="true" name="default_physics" type="ode">
  <max_step_size>0.02</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>50</real_time_update_rate>
</physics>
```

---

## 2.5 Controller Update Rate Faster Than Gazebo Physics Rate

Typical error:

```text
Desired controller update period (0.01 s)
is faster than the gazebo simulation period (0.02 s)
```

Gazebo configuration:

```xml
<max_step_size>0.02</max_step_size>
<real_time_update_rate>50</real_time_update_rate>
```

This means:

```text
Physics period = 0.02 s
Physics frequency = 50 Hz
```

If `controller_manager` is configured as:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
```

then:

```text
Controller period = 0.01 s
Controller frequency = 100 Hz
```

The controller is therefore requesting updates faster than Gazebo can simulate.

Recommended fix:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50
```

Result:

```text
Gazebo Physics = 50 Hz
Controller Manager = 50 Hz
```

---

## 2.6 Low RTF Performance

Gazebo may show:

```text
RTF        4.60 %
Sim time   2.120 s
Real time  53.805 s
Iterations 106
```

RTF means:

```text
RTF = Real Time Factor
```

Approximately:

```text
RTF = Simulation Time / Real Time
```

Meaning:

```text
RTF = 100%
```

Real-world 1 second corresponds to approximately 1 simulation second.

```text
RTF = 5%
```

Real-world 100 seconds correspond to approximately 5 simulation seconds.

### Recommended troubleshooting order

```text
1. Check whether GPU rendering is hardware accelerated
2. Check whether scene models are too complex
3. Check collision mesh complexity
4. Check whether fixed objects are incorrectly dynamic
5. Check LiDAR and camera load
6. Check Gazebo plugin load
7. Check ros2_control update frequency
```

---

## 2.7 Check OpenGL Rendering

Run:

```bash
glxinfo | grep "OpenGL renderer"
```

Example output:

```text
OpenGL renderer string:
GFX1103_R1
(gfx1103_r1, LLVM 15.0.7, DRM 3.57, 6.8.0-124-generic)
```

This indicates an AMD GPU is being used.

If the output contains:

```text
llvmpipe
```

then the system is using CPU software rendering, which can severely reduce Gazebo performance.

---

## 2.8 Mark Fixed Scene Objects as Static

For fixed objects:

```xml
<include>
  <uri>model://Depot</uri>
  <name>Depot</name>
  <pose>...</pose>
  <static>true</static>
</include>
```

Recommended static objects:

```text
ground_plane
Depot
Apriltag models
cart_model_2
A_letter
B_letter
```

Only moving objects, such as the robot, should remain dynamic.

---

## 2.9 Check Collision Mesh Complexity

Search for collision definitions:

```bash
grep -R "<collision>" -n \
swervebot_description/models/Depot
```

Search for mesh usage:

```bash
grep -R "<mesh>" -n \
swervebot_description/models/Depot
```

Recommended design:

```text
Visual:
    High-resolution mesh

Collision:
    Simplified box
    Cylinder
    Simplified mesh
```

Avoid using an extremely high-resolution visual mesh directly as a collision mesh.

---

## 2.10 Disable Shadows for Performance Testing

Disable scene shadows:

```xml
<scene>
  <shadows>false</shadows>
</scene>
```

Disable light shadow casting:

```xml
<cast_shadows>false</cast_shadows>
```

This mainly affects rendering performance.

---

## 2.11 `gz_ros2_control` Plugin Not Found

Typical error:

```text
Failed to load system plugin [gz_ros2_control-system]:
couldn't find shared library.
```

Check:

```bash
ros2 pkg prefix gz_ros2_control
```

Install if missing:

```bash
sudo apt install ros-humble-gz-ros2-control
```

Search for the library:

```bash
find /opt/ros/humble -name "*gz_ros2_control*"
```

---

## 2.12 `ros2 control` Command Not Found

Typical error:

```text
ros2: error: invalid choice: 'control'
```

This means the `ros2controlcli` package is missing.

Install:

```bash
sudo apt install ros-humble-ros2controlcli
```

Then source the environment:

```bash
source /opt/ros/humble/setup.bash
```

Check:

```bash
ros2 control -h
```

---

## 2.13 What `ros2 control` Does

`ros2 control` is the command-line interface for `ros2_control`.

System architecture:

```text
Gazebo
   ↓
gz_ros2_control
   ↓
controller_manager
   ↓
controller plugins
```

Common commands:

```bash
ros2 control list_controllers
```

```bash
ros2 control list_controller_types
```

```bash
ros2 control list_hardware_interfaces
```

```bash
ros2 control load_controller <controller_name>
```

---

## 2.14 Swerve Drive Hardware Interfaces

Example:

```text
command interfaces

backleft_caster_wheel_joint/velocity
backright_caster_wheel_joint/velocity

base_backleft_caster_joint/position
base_backright_caster_joint/position
base_frontleft_caster_joint/position
base_frontright_caster_joint/position

frontleft_caster_wheel_joint/velocity
frontright_caster_wheel_joint/velocity
```

The drivetrain therefore has:

```text
4 steering position command interfaces
+
4 wheel velocity command interfaces
```

Meaning:

```text
Steering angle control:
position

Wheel drive control:
velocity
```

---

## 2.15 ros2_control Controller Configuration

Example YAML:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    caster_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    wheel_trajectory_controller:
      type: velocity_controllers/JointGroupVelocityController


caster_trajectory_controller:
  ros__parameters:
    joints:
      - base_frontleft_caster_joint
      - base_frontright_caster_joint
      - base_backleft_caster_joint
      - base_backright_caster_joint

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    allow_partial_joints_goal: true

    constraints:
      goal_time: 0.5
      stopped_velocity_tolerance: 0.2


wheel_trajectory_controller:
  ros__parameters:
    joints:
      - frontleft_caster_wheel_joint
      - frontright_caster_wheel_joint
      - backleft_caster_wheel_joint
      - backright_caster_wheel_joint
```

---

## 2.16 Controller Plugin Not Found

Typical error:

```text
Loader for controller 'joint_state_broadcaster'
(type 'joint_state_broadcaster/JointStateBroadcaster')
not found.
```

Check available controller types:

```bash
ros2 control list_controller_types
```

Expected types include:

```text
joint_state_broadcaster/JointStateBroadcaster
joint_trajectory_controller/JointTrajectoryController
velocity_controllers/JointGroupVelocityController
```

Install all standard controllers:

```bash
sudo apt install ros-humble-ros2-controllers
```

Or install individual packages:

```bash
sudo apt install \
  ros-humble-joint-state-broadcaster \
  ros-humble-velocity-controllers
```

After installation:

```bash
source /opt/ros/humble/setup.bash
```

Restart Gazebo completely because `/controller_manager` is running inside the Gazebo process.

---

## 2.17 Meaning of `[available] [unclaimed]`

Example:

```text
frontleft_caster_wheel_joint/velocity
[available] [unclaimed]
```

Meaning:

```text
available:
The hardware interface exists and can be used.

unclaimed:
No active controller currently owns this interface.
```

After successful controller activation:

```text
[available] [claimed]
```

---

## 2.18 Control Launch File Startup Order

The launch file starts controllers in the following order:

```text
joint_state_broadcaster
        ↓
caster_trajectory_controller
        ↓
wheel_trajectory_controller
```

At the same time:

```text
swerve_controller
scan_merger
```

are also started.

Important note:

```python
OnProcessExit
```

only means that the previous process exited.

It does not guarantee that the previous controller was loaded successfully.

Therefore:

```text
Spawner exits successfully
```

and:

```text
Spawner exits with failure
```

may both trigger the next event.

---

## 2.19 Complete Four-Wheel Swerve Control Chain

```text
Nav2 / Teleop
      ↓
    /cmd_vel
      ↓
swerve_controller
      ↓
Calculate:
4 steering angles
4 wheel speeds
      ↓
┌─────────────────────────────┐
│ caster_trajectory_controller│
│ wheel_trajectory_controller │
└─────────────────────────────┘
      ↓
controller_manager
      ↓
gz_ros2_control
      ↓
Gazebo
      ↓
4 steering joints
4 wheel joints
```

---

## 2.20 Dual-LiDAR Scan Merger

Input topics:

```text
/backright/scan
/topleft/scan
```

Output topic:

```text
/scan
```

Parameters:

```python
{'angle_min': -3.14159265},
{'angle_max': 3.14159265},
{'angle_increment': 0.004363323}
```

Meaning:

```text
Angular range:
-π to +π

Approximately 360°
```

Estimated number of scan points:

```text
2π / 0.004363323
≈ 1440 points
```

---

## 2.21 Recommended Debugging Workflow

When the simulation fails, use this order:

```text
1. Check whether Git submodules are complete
2. Use colcon list to verify packages
3. Check missing ROS 2 dependencies
4. Fix CMake imported target compatibility
5. Check ros_gz_sim installation
6. Check world file path
7. Check SDF version
8. Check IGN_GAZEBO_RESOURCE_PATH
9. Check model:// resources
10. Check gz_ros2_control
11. Check /controller_manager
12. Run ros2 control list_controller_types
13. Run ros2 control list_hardware_interfaces
14. Check controller YAML
15. Check claimed / unclaimed interfaces
16. Check RTF
17. Check static models
18. Check collision mesh complexity
19. Check LiDAR / camera load
20. Continue with Nav2 and kinematics debugging
```

---

# Chapter 3. Swerve Kinematics and Lawn Mower Robot Research

## 3.1 Four-Wheel Swerve Kinematics

Robot body velocity:

$$\mathbf{v}
\begin{bmatrix}
v_x\
v_y\
\omega
\end{bmatrix}
$$

For wheel (i), located at:

[
(x_i,y_i)
]

relative to the robot center, the local wheel velocity is:

[
v_{ix}=v_x-\omega y_i
]

$ v_{iy}=v_y+\omega x_i $

Wheel speed:

$
v_i=
\sqrt{
v_{ix}^2+v_{iy}^2
}
$

Steering angle:

$
\delta_i=
\operatorname{atan2}(v_{iy},v_{ix})
$

---

## 3.2 Standard Four-Wheel Layout

Assume:

```text
Front Left:  (+L/2, +W/2)
Front Right: (+L/2, -W/2)
Rear Left:   (-L/2, +W/2)
Rear Right:  (-L/2, -W/2)
```

Front-left wheel:

$
v_{fl,x}=v_x-\omega\frac{W}{2}
$

$
v_{fl,y}=v_y+\omega\frac{L}{2}
$

Front-right wheel:

$
v_{fr,x}=v_x+\omega\frac{W}{2}
$

$
v_{fr,y}=v_y+\omega\frac{L}{2}
$

Rear-left wheel:

$
v_{rl,x}=v_x-\omega\frac{W}{2}
$

$
v_{rl,y}=v_y-\omega\frac{L}{2}
$

Rear-right wheel:

$
v_{rr,x}=v_x+\omega\frac{W}{2}
$

$
v_{rr,y}=v_y-\omega\frac{L}{2}
$

For each wheel:

$
v_i=
\sqrt{
v_{ix}^2+v_{iy}^2
}
$

$
\delta_i=
\operatorname{atan2}(v_{iy},v_{ix})
$

---

## 3.3 Steering Angle Optimization

Suppose the current steering angle is:

```text
170°
```

and the target angle is:

```text
-170°
```

A direct rotation may require:

```text
340°
```

A better strategy is:

```text
Rotate the steering angle by approximately 180°
+
Reverse the wheel speed
```

Typical logic:

```cpp
if (std::abs(angle_error) > M_PI_2)
{
    target_angle += M_PI;
    target_speed = -target_speed;
}
```

Then normalize the angle to:

```text
[-π, π]
```

This is one of the most important practical optimizations in swerve drive control.

---

## 3.4 Dual-Swerve vs Four-Wheel Swerve Kinematics

A dual-swerve drivetrain is not simply:

```text
four-wheel swerve minus two wheels
```

Although each individual wheel still follows:

$
v_{ix}=v_x-\omega y_i
$

$
v_{iy}=v_y+\omega x_i
$

the complete drivetrain changes in terms of:

```text
Constraint conditions
Controllable degrees of freedom
Forward kinematics
Inverse kinematics
Redundancy
Singular configurations
Support wheel interaction
```

The exact kinematic model depends strongly on the mechanical layout.

---

## 3.5 Front-Rear Dual-Swerve Kinematics

Assume two active swerve modules:

```text
Front module: (L/2, 0)
Rear module:  (-L/2, 0)
```

Robot velocity:

```text
vx
vy
wz
```

Front module:

$
v_{fx}=v_x
$

$
v_{fy}=v_y+\omega\frac{L}{2}
$

Steering angle:

$
\delta_f=
\operatorname{atan2}
\left(
v_y+\omega\frac{L}{2},
v_x
\right)
$

Wheel speed:

$
s_f=
\sqrt{
v_x^2+
\left(
v_y+\omega\frac{L}{2}
\right)^2
}
$

Rear module:

$
v_{rx}=v_x
$

$
v_{ry}=v_y-\omega\frac{L}{2}
$

Steering angle:

$
\delta_r=
\operatorname{atan2}
\left(
v_y-\omega\frac{L}{2},
v_x
\right)
$

Wheel speed:

$
s_r=
\sqrt{
v_x^2+
\left(
v_y-\omega\frac{L}{2}
\right)^2
}
$

---

## 3.6 Differential Drive for Lawn Mowers

Advantages:

```text
Simple mechanical structure
Low cost
Simple control
Can rotate in place
Mature ROS 2 and Nav2 support
```

Disadvantages:

```text
In-place rotation may damage grass
Soft ground can cause wheel slip
Repeated skid steering may leave visible marks
```

Best suited for:

```text
Small residential lawn robots
Low-cost products
Rapid product development
```

Differential drive kinematics:

$
v=\frac{v_r+v_l}{2}
$

$
\omega=\frac{v_r-v_l}{L}
$

Inverse form:

$
v_l=v-\frac{\omega L}{2}
$

$
v_r=v+\frac{\omega L}{2}
$

---

## 3.7 Ackermann Steering for Lawn Mowers

Simplified bicycle model:

$
\omega=
\frac{v\tan\delta}{L}
$

Advantages:

```text
Smooth turning
Low turf damage
Good high-speed stability
Suitable for large-area mowing
```

Disadvantages:

```text
Cannot rotate in place
Large minimum turning radius
Less suitable for narrow areas
Coverage planning must respect curvature constraints
```

Best suited for:

```text
Large parks
Golf courses
Sports fields
Medium and large commercial mowing robots
```

---

## 3.8 Dual-Swerve Drive for Lawn Mowers

Advantages:

```text
High maneuverability
Possible lateral motion
Possible diagonal motion
Possible in-place or near-in-place rotation
Strong narrow-area coverage capability
```

Especially useful for:

```text
Boustrophedon coverage
Boundary following
Lateral lane shifting
Complex boundaries
Narrow spaces
```

A major potential benefit for coverage planning is:

```text
Finish one mowing lane
      ↓
Rotate the swerve modules by 90°
      ↓
Shift sideways by one cutting width
      ↓
Continue with the next lane
```

This can reduce or eliminate a traditional U-turn.

---

## 3.9 Comparison for Lawn Mower Applications

### Differential Drive

Best for:

```text
Lowest cost
Simplest control
Fast development
Small residential products
```

### Ackermann Steering

Best for:

```text
Large-area operation
Higher speed
Low turf damage
Commercial mowing platforms
```

### Dual-Swerve Drive

Best for:

```text
High maneuverability
Lateral motion
Coverage efficiency innovation
Narrow spaces
Complex environments
```

---

## 3.10 Recommended Selection

A practical summary:

```text
Lowest cost and fastest productization:
    Differential drive

Large-area, high-speed, turf-friendly operation:
    Ackermann steering

High maneuverability, lateral motion, and coverage efficiency:
    Dual-swerve drive
```

For different products:

```text
Small residential lawn mower
    → Differential drive

Medium or large commercial lawn mower
    → Ackermann steering

Differentiated high-mobility mowing robot
    → Dual-swerve drive
```

---

## 3.11 Why Dual-Swerve Is Interesting for Coverage Robots

Traditional boustrophedon coverage:

```text
────────────→
             U-turn
←────────────
```

A swerve-based platform may instead use:

```text
────────────→
             ↓ Side shift by one coverage width
←────────────
```

The robot may even keep:

```text
body yaw unchanged
```

while only changing:

```text
steering angle
+
wheel speed direction
```

This can potentially improve:

```text
Coverage efficiency
Turnaround time
Turf protection
Lane transition distance
```

---

## 3.12 Common Debugging Commands

Check ROS packages:

```bash
ros2 pkg list | grep swerve
```

Check nodes:

```bash
ros2 node list
```

Check controllers:

```bash
ros2 control list_controllers
```

Check controller plugins:

```bash
ros2 control list_controller_types
```

Check hardware interfaces:

```bash
ros2 control list_hardware_interfaces
```

Check Gazebo resource paths:

```bash
echo $IGN_GAZEBO_RESOURCE_PATH
```

Check GPU renderer:

```bash
glxinfo | grep "OpenGL renderer"
```

Find Gazebo models:

```bash
find . -name model.config
```

Find SDF files:

```bash
find . -name "*.sdf"
```

Check Git submodules:

```bash
git submodule status
```

Update Git submodules:

```bash
git submodule update --init --recursive
```

Monitor high CPU usage:

```bash
watch -n 1 'ps -eo pid,ppid,pcpu,pmem,rss,cmd --sort=-pcpu | head -20'
```

---