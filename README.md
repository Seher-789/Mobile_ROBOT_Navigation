
# Mobile Robot Navigation using GMapping

This project implements **2D SLAM (GMapping)** using a **differential drive mobile robot** equipped with a **Velodyne VLP-16 3D LiDAR**, simulated in **Gazebo 11** with **ROS Noetic** on **Ubuntu 20.04**.

---

## Overview

A custom-built mobile robot is simulated in Gazebo to perform SLAM using the GMapping package. The robot uses a Velodyne VLP-16 LiDAR sensor and differential drive control, visualized in RViz. This project showcases how to simulate realistic LiDAR-based SLAM in a ROS Noetic environment.

---

## Requirements
Ubuntu 20.04
ROS Noetic
Gazebo 11
Velodyne VLP-16
GMapping package: ros-noetic-slam-gmapping
---

## Launch Instructions

Run the following launch files in order:

## 1. **Start RViz:**
```bash
roslaunch robot_second robot_2_arviz.launch
```

## 2. Start the robot and world in Gazebo:
```bash
roslaunch robot_second robot_2_gazebo.launch
```

## 3. Start GMapping SLAM:
```bash
roslaunch robot_second robot_2_gmapping.launch
```

##  Features
**Two-wheel differential drive mobile robot** 

**Integrated Velodyne VLP-16 3D LiDAR sensor** 

**Realistic simulation in Gazebo 11** 

**2D SLAM using GMapping** 

**Live visualization in RViz** 

**Modular and clean launch structure** 

**ROS-compliant URDF/Xacro robot description** 

## Install required dependencies:
```bash
sudo apt update
sudo apt install ros-noetic-slam-gmapping ros-noetic-velodyne-gazebo-plugins
```

## Project Structure
```bash
robot_second/
├── launch/
│   ├── robot_2_arviz.launch
│   ├── robot_2_gazebo.launch
│   └── robot_2_gmapping.launch
├── urdf/
│   └── robot_model.urdf.xacro
├── worlds/
│   └── ISCAS_building.world
├── config/
│   └── rviz and SLAM configurations
```
<img width="1473" height="1218" alt="Screenshot from 2025-07-16 14-53-40" src="https://github.com/user-attachments/assets/f0cfa700-33a0-49c6-929d-f21af8671340" />
<img width="1049" height="1175" alt="Screenshot from 2025-07-16 14-53-26" src="https://github.com/user-attachments/assets/a77281ed-3c16-4911-8fd6-f12fb858b7bc" />

