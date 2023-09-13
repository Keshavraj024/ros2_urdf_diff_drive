# ros2_urdf_diff_drive
# Controller Node

![ROS2](https://img.shields.io/badge/ROS2-%E2%9C%93-brightgreen)


# URDF Robot Description

drive_bot.urdf (Unified Robot Description Format) file that describes a simple robot with a base and wheels.

## Robot Description

The robot consists of the following components:

1. `base_footprint`: A virtual link representing the base footprint.
2. `base_link`: The main body of the robot, represented as a box with dimensions `base_length`, `base_width`, and `base_height`.
3. `right_wheel_link` and `left_wheel_link`: The right and left wheels of the robot, represented as cylinders with radius `wheel_radius` and length `wheel_length`.
4. `caster_wheel_link`: A caster wheel represented as a sphere with radius `wheel_radius / 2.0`

## Robot Laser Scanner Model

URDF (Unified Robot Description Format) representation for a robot's laser scanner model, commonly used in robotics for environment sensing. The laser scanner is represented as a cylindrical link with visual and collision properties.

**Key Features:**
- A Gazebo sensor plugin for simulating laser scan data.
- Configurable scan properties, such as scan angle, number of samples, range, and noise.

## Dependencies

- `rclcpp`

## Installation

1. Make sure you have ROS2 installed.
2. Clone this repository to your ROS2 workspace:

```bash
cd /path/to/your/ros2_workspace/src
git clone https://github.com/Keshavraj024/ros2_urdf_diff_drive.git
```

3. Build the package:

```bash
cd /path/to/your/ros2_urdf_diff_drive
colcon build
```

## Usage

To run, use the following command:

```bash
ros2 launch drive_bot_bringup drive_bot.launch.xml 
```

## Navigation Stack

**Description of the nav2 Launch File**

The `nav2` launch file is a critical component of the Navigation Stack 2.0 (nav2) in the Robot Operating System (ROS). This launch file is designed to facilitate autonomous navigation for robots by coordinating various navigation components, such as the robot's sensors, motion planner, and control system. It provides a comprehensive solution for enabling robots to navigate through complex environments, avoiding obstacles, and reaching designated goals.

**Key Features and Components:**

- **Robot Setup**: The `nav2` launch file configures and initializes the robot's hardware components, including sensors like LiDAR and establishes their connections with ROS2.

- **Navigation Stack**: It launches the core components of the Navigation Stack, including the `amcl` node for localization, the global and local planners for path planning, and the controller for motion control. These components work together to enable autonomous navigation.

- **Map Loading**: The launch file loads the map of the environment in which the robot will navigate.

- **Navigation Goals**: The `nav2` launch file can specify initial navigation goals, enabling the robot to start moving autonomously toward a predefined location upon startup.


**Usage**

```bash
ros2 launch drive_bot_bringup drive_bot_navigation.launch.xml 
```

### Create a map using the following commands
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
ros2 run turtlebot3_teleop teleop_keyboard 
ros2 run nav2_map_server map_saver_cli -f path/to/save
```
