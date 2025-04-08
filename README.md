# Jetank Description

This repository contains the ROS 2 setup files for the **Jetank** robot, including:

- Launch files  
- URDF and Xacro files  
- STL files for visualization and simulation

These are required to properly configure the `robot_state_publisher` and simulate the Jetank in RViz and Gazebo.

![Jetank](https://www.waveshare.com/media/catalog/product/cache/1/image/800x800/9df78eab33525d08d6e5fb8d27136e95/j/e/jetank-ai-kit-1.jpg)

---

## Installation Instructions

### 1. Clone the repository

```bash
cd ~/ros2_ws/src/
git clone https://github.com/inbarajaldrin/jetank_description.git
```

### 2. Install dependencies

```bash
cd ~/ros2_ws/
rosdep install --from-paths src --ignore-src -r -y
```

Then update your packages and install the necessary ROS 2 tools:

```bash
sudo apt update
sudo apt install -y ros-humble-joint-state-publisher-gui
sudo apt install -y ros-humble-rviz2
```

---

## Launching the Robot

### 1. Visualize the robot in RViz

```bash
ros2 launch jetank_description jetank_rviz.launch.py
```

This will start a joint state broadcaster and visualize the robot in RViz.

### 2. Simulate in Ignition Gazebo

```bash
ros2 launch jetank_description jetank_gazebo.launch.py
```

This will spawn the robot in Ignition Gazebo.

Then, open RViz separately:

```bash
rviz2
```

In RViz, subscribe to the camera topic (e.g., `/camera/image_raw`) to see the camera feed from Gazebo.

---

## Teleoperation

Use the provided teleoperation script to control the robot:

```bash
cd ~/ros2_ws/src/jetank_description/scripts/
python3 teleop.py
```
