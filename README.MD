# Swerve Drive Rocker Robot


new:
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

# 🚗 Swerve Drive Rocker Robot - ROS 2 Simulation

This repository contains a ROS 2 package for simulating and controlling a **Swerve Drive Rocker Robot** in **Gazebo**. The robot combines the maneuverability of swerve drive with the terrain adaptability of a rocker suspension system. The simulation supports URDF-based modeling, ROS 2 control integration, and keyboard teleoperation.

## 📁 Repository Contents

- `rocker_description/` – URDF and Xacro files for robot description
- `rocker_controller/` – ROS 2 Control configs and launch files
- `launch/` – Gazebo, controllers, and teleop launch scripts
- `rviz/` – RViz configuration (if included)
- `worlds/` – Custom Gazebo world files (optional)

## 🛠️ Prerequisites

- **Ubuntu 22.04**
- **ROS 2 Humble Hawksbill**
- **Gazebo Fortress or Garden**

### Install Required Dependencies

```bash
sudo apt update
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-state-publisher-gui \
  python3-colcon-common-extensions
```bash
1️⃣ Clone the Repository
mkdir -p ~/rocker_ws/src
cd ~/rocker_ws/src
git clone https://github.com/your_username/rocker_ws.git

2️⃣ Build the Workspace
cd ~/rocker_ws
colcon build --symlink-install
source install/setup.bash

🧪 Running the Simulation
Open 3 separate terminals and source the workspace in each:
source ~/rocker_ws/install/setup.bash

🔷 Terminal 1: Launch Gazebo with the Robot
ros2 launch rocker_description gazebo.launch.py

🔶 Terminal 2: Load ROS 2 Controllers
ros2 launch rocker_controller controller.launch.py

⌨️ Terminal 3: Keyboard Teleoperation
ros2 launch rocker_controller keyboard_teleop.launch.py



