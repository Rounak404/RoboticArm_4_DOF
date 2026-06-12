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
├── urdf/
│   └── robotic_arm.urdf
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
 └── link1
      └── link2
           └── link3
                └── link4
```

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

## Features

* Robot visualization in RViz
* TF tree generation through robot_state_publisher
* Joint state publishing
* ros2_control integration
* STL mesh support

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
