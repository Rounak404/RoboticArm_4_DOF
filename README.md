# Robotic Arm 4DOF Workspace

A custom robotic manipulation platform developed using **ROS 2 Jazzy** and **MoveIt 2** for learning robot modeling, kinematics, motion planning, trajectory execution, and robotic manipulation.

The project consists of a custom-designed robotic arm, a parallel gripper, MoveIt 2 integration, and future support for simulation, computer vision, and pick-and-place operations.

---

# Project Overview

This project was developed to gain hands-on experience with:

* Robot Modeling (URDF/Xacro)
* ROS 2
* MoveIt 2
* Kinematics
* Motion Planning
* Collision Detection
* ros2_control
* RViz Visualization
* Gazebo Simulation
* Robotic Manipulation

---

# Project Architecture

```text
Robotic Arm
     │
     ▼
URDF / Xacro
     │
     ▼
MoveIt 2
     │
     ├── Kinematics
     ├── Motion Planning
     ├── Collision Checking
     └── Trajectory Execution
     │
     ▼
Gripper Control
     │
     ▼
Pick and Place Pipeline
```

---

# Documentation

Detailed package documentation is available in the `docs` directory.

## Robot Description Package

Contains:

* URDF/Xacro Files
* Robot Links and Joints
* Meshes
* RViz Visualization
* Robot Launch Files

📖 Documentation:

➡️ [Robot Description Package](docs/roboticarm_desc.md)

---

## MoveIt Configuration Package

Contains:

* SRDF Configuration
* Planning Groups
* Kinematics Configuration
* Controller Configuration
* Motion Planning Setup
* Trajectory Execution

📖 Documentation:

➡️ [MoveIt Configuration Package](docs/moveit_setup.md)

---

# Screenshots

## Robot Overview

![Robot Overview](docs/images/robot_overview.png)

Robot model successfully loaded and visualized in RViz.

---

## Planning Groups

![Planning Groups](docs/images/planning_groups.png)

Arm and gripper planning groups configured using MoveIt Setup Assistant.

---

## Motion Planning

![Motion Planning](docs/images/motion_planning.png)

Collision-aware trajectory planning using MoveIt 2 and OMPL.

---

## Gripper Control

![Gripper Control](docs/images/gripper_control.png)

Independent gripper planning and execution.

---

# Workspace Structure

```text
robotic_arm_4dof_ws/

├── docs/
│   ├── roboticarm_desc.md
│   ├── moveit_setup.md
│   └── images/
│
├── src/
│   ├── robotic_arm_desc/
│   └── robotic_arm_moveit_config/
│
├── build/
├── install/
└── log/
```

---

# Packages

## robotic_arm_desc

Robot description package containing:

* URDF/Xacro
* Meshes
* Launch Files
* RViz Configuration

Documentation:

➡️ [Read More](docs/roboticarm_desc.md)

---

## robotic_arm_moveit_config

MoveIt 2 configuration package containing:

* Planning Groups
* Kinematics
* Controllers
* Motion Planning
* Trajectory Execution

Documentation:

➡️ [Read More](docs/moveit_setup.md)

---

# Building the Workspace

Clone the repository:

```bash
git clone <repository-url>
cd robotic_arm_4dof_ws
```

Build:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

---

# Running the Robot Description

Launch RViz visualization:

```bash
ros2 launch robotic_arm_desc display.launch.py
```

---

# Running MoveIt 2

Launch MoveIt environment:

```bash
ros2 launch robotic_arm_moveit_config demo.launch.py
```

This starts:

* Robot State Publisher
* Move Group
* RViz
* Motion Planning Pipeline
* ros2_control
* Controller Interfaces

---

# Features

### Robot Description

* Custom 4-DOF robotic arm
* Parallel gripper
* Modular URDF structure
* STL mesh integration

### MoveIt 2

* KDL inverse kinematics
* OMPL motion planning
* Collision checking
* Planning groups
* End effector configuration

### Motion Planning

* Pose goal planning
* Joint-space planning
* Interactive marker control
* Trajectory execution

### Controller Integration

* ros2_control support
* Joint state broadcasting
* MoveIt controller interface

---

# Future Extensions

The current architecture allows future integration of:

* Gazebo Harmonic
* Camera Sensors
* OpenCV
* Object Detection
* Pick and Place Pipeline
* Hardware Deployment
* Arduino Servo Control

---

# Technologies Used

* ROS 2 Jazzy
* MoveIt 2
* OMPL
* RViz
* ros2_control
* URDF
* Xacro
* Python
* C++

---

# Author

**Rounak Senapati**

Robotics | ROS 2 | MoveIt 2 | Computer Vision | Autonomous Systems

---

If you found this project useful, consider giving the repository a ⭐.
