# Robotic Arm Description Package

## Overview

This package contains the robot description of a 4-DOF robotic arm developed for learning ROS 2, URDF modeling, motion planning, and robot simulation.

The package provides:

* Robot model definition using URDF
* Meshes exported from SolidWorks
* RViz visualization support
* ros2_control configuration
* Robot State Publisher integration

---

## Package Structure

```text
robotic_arm_desc/
robotic_arm_desc/
├── urdf/
│   ├── robotic_arm.urdf.xacro
│   ├── ros2_control.xacro
│   ├── gazebo_ros2_control.xacro
│   └── gazebo.xacro
├── meshes/
│   ├── base_link.STL
│   ├── link1.STL
│   ├── link2.STL
│   ├── link3.STL
│   └── link4.STL
├── config/
│   └── controllers.yaml
├── launch/
│   └── display.launch.py
├── package.xml
└── setup.py
```

---

## Robot Specifications

| Property           | Value        |
| ------------------ | ------------ |
| Degrees of Freedom | 4            |
| CAD Software       | SolidWorks   |
| Middleware         | ROS 2 Jazzy  |
| Visualization      | RViz2        |
| Control Framework  | ros2_control |
| Motion Planning    | MoveIt 2     |

---

## Coordinate Frames

```text
base_link
 └── link_1
      └── link_2
           └── link_3
                └── link_4
                     ├── link_5
                     ├── link_6
                     └── tcp_link

---

## Launching the Robot

Build the workspace:

```bash
colcon build
source install/setup.bash
```

Launch the robot:

```bash
ros2 launch robotic_arm_desc display.launch.py
```

---

## ros2_control Architecture

The robot description package supports two different ros2_control backends depending on the execution environment.

### RViz / MoveIt Execution

For RViz visualization and MoveIt trajectory execution, the robot uses the GenericSystem mock hardware interface.

```xml
<plugin>mock_components/GenericSystem</plugin>
```

This backend provides:

* Motion planning with MoveIt 2
* Controller testing without simulation
* RViz trajectory execution
* Joint state publishing
* Development and debugging without physical hardware

The GenericSystem plugin mirrors commanded joint states directly to the state interfaces, making it ideal for testing motion planning pipelines.

---

### Gazebo Simulation

For physics-based simulation, the robot uses the GazeboSimSystem hardware interface.

```xml
<plugin>gz_ros2_control/GazeboSimSystem</plugin>
```

This backend provides:

* Gazebo Harmonic integration
* Physics-based joint simulation
* Controller execution through ros2_control
* Joint state feedback from simulation
* FollowJointTrajectory action support

The GazeboSimSystem plugin allows controllers to interact directly with simulated joints inside Gazebo.

---

### Backend Selection

The robot description dynamically selects the appropriate backend using a Xacro argument.

```xml
<xacro:arg name="use_gazebo" default="false"/>
```

When enabled:

```xml
use_gazebo:=true
```

The robot loads:

```text
gazebo_ros2_control.xacro
gazebo.xacro
```

When disabled:

```xml
use_gazebo:=false
```

The robot loads:

```text
ros2_control.xacro
```

This architecture allows a single robot model to be used across RViz, MoveIt 2, and Gazebo while maintaining separate controller implementations for each environment.


## Features

* Robot visualization in RViz2
* TF tree generation through robot_state_publisher
* Joint state publishing
* ros2_control integration
* STL mesh support
* Xacro-based robot description
* MoveIt 2 compatible robot model
* Gazebo Harmonic simulation support
* FollowJointTrajectory controller support
* Tool Center Point (TCP) frame for manipulation tasks
---

## Future Improvements

* MoveIt 2 integration
* Gazebo simulation
* Arduino servo control
* Camera-based object detection
* Autonomous pick-and-place operations

---

## Author

Rounak Senapati
Robotics and ROS 2 Learning Project
