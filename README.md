# 🤖 ROS 2 Custom Mobile Robot

A custom-designed differential drive mobile robot built from scratch using ROS 2 Humble, featuring URDF/Xacro modeling, LiDAR sensor integration, and Gazebo simulation.

![Mobile Robot in Gazebo](docs/images/gazebo_simulation.png)

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Robot Description](#robot-description)
- [Future Improvements](#future-improvements)
- [Author](#author)

---

## 🎯 Overview

This project demonstrates the complete workflow of creating a custom mobile robot in ROS 2 Humble:

1. **Robot Design** - Designing the robot structure with links and joints
2. **URDF/Xacro Modeling** - Creating the robot description with proper kinematics
3. **Physical Properties** - Defining mass, inertia, and collision geometries
4. **TF Publishing** - Setting up the transform tree via robot state publisher
5. **Sensor Integration** - Configuring LiDAR sensor plugin
6. **Motion Control** - Implementing differential drive controller
7. **Gazebo Simulation** - Spawning and simulating the robot in Gazebo

---

## ✨ Features

| Feature | Description |
|---------|-------------|
| **Custom URDF/Xacro** | Modular robot description using Xacro macros |
| **Differential Drive** | Two-wheel drive system with controller plugin |
| **LiDAR Sensor** | Laser scanner for environment perception |
| **Gazebo Simulation** | Full physics simulation with sensor plugins |
| **TF Tree** | Complete transform tree for all robot links |
| **RViz Visualization** | Real-time visualization of robot model, TFs, and sensor data |

---

## 📁 Project Structure

```
ros2_my_mobile_robot/
├── src/
│   └── my_robot_bringup/
│       ├── launch/
│       │   ├── display.launch.py      # RViz visualization
│       │   └── gazebo.launch.py       # Gazebo simulation
│       ├── urdf/
│       │   ├── my_robot.urdf.xacro    # Main robot description
│       │   ├── robot_core.xacro       # Core structure
│       │   └── gazebo_control.xacro   # Gazebo plugins
│       ├── rviz/
│       │   └── config.rviz            # RViz configuration
│       ├── meshes/                    # Visual meshes (if any)
│       ├── worlds/                    # Gazebo world files
│       ├── package.xml
│       └── CMakeLists.txt
└── README.md
```

---

## 🔧 Prerequisites

- **Ubuntu 22.04** (Jammy Jellyfish)
- **ROS 2 Humble Hawksbill**
- **Gazebo Classic** (Gazebo 11)

### Required ROS 2 Packages

```bash
sudo apt update && sudo apt install -y \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-rviz2
```

---

## 📥 Installation

### 1. Create a ROS 2 Workspace (if not exists)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone the Repository

```bash
git clone https://github.com/HerrTejas/ros2_my_mobile_robot.git
```

### 3. Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build the Workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Usage

### Launch RViz Visualization

Visualize the robot model and TF tree in RViz:

```bash
ros2 launch my_robot_bringup display.launch.py
```

### Launch Gazebo Simulation

Spawn the robot in Gazebo simulation:

```bash
ros2 launch my_robot_bringup gazebo.launch.py
```

### Control the Robot

Send velocity commands to move the robot:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

### View LiDAR Data

```bash
ros2 topic echo /scan
```

### View TF Tree

```bash
ros2 run tf2_tools view_frames
```

---

## 🤖 Robot Description

### Physical Specifications

| Component | Specification |
|-----------|--------------|
| **Base** | Rectangular chassis |
| **Drive Type** | Differential drive (2 wheels) |
| **Caster** | Rear caster wheel for stability |
| **Sensor** | 2D LiDAR scanner |
| **Frame** | `base_footprint` → `base_link` |

### Coordinate Frames

```
base_footprint
    └── base_link
        ├── left_wheel_link
        ├── right_wheel_link
        ├── caster_wheel_link
        └── lidar_link
```

### Sensor Configuration

| Sensor | Type | Topic |
|--------|------|-------|
| LiDAR | 2D Laser Scanner | `/scan` |

### Control Interface

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/odom` | `nav_msgs/Odometry` | Odometry data |

---

## 🔮 Future Improvements

- [ ] Add camera sensor for visual perception
- [ ] Implement SLAM for mapping
- [ ] Add Nav2 navigation stack
- [ ] Create autonomous navigation demo
- [ ] Add IMU sensor

---

## 👨‍💻 Author

**Tejas Murkute**

- 🎓 M.Sc. Industry 4.0: Automation, Robotics & 3D Manufacturing - SRH Berlin
- 📧 herrtejasmurkute@gmail.com
- 💼 [LinkedIn](https://linkedin.com/in/tejas-murkute-b1792a1b8)
- 🐙 [GitHub](https://github.com/HerrTejas)

---

## 📄 License

This project is open source and available under the [MIT License](LICENSE).

---

## 🙏 Acknowledgments

- ROS 2 Documentation and Community
- Gazebo Simulation Team
- Open Robotics

---

<p align="center">
  <i>Built with ❤️ and ROS 2 Humble</i>
</p>
