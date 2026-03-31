# 🚀 Team Drishti – ROS 2 Workspace

International Rover Challenge (IRC) 2026

---

## 📌 Overview

This repository contains the ROS 2 workspace used by Team Drishti for the International Rover Challenge 2026.

It includes:

* Cone detection using OpenVINO
* ROS 2 communication (C++ and Python)
* Launch system for running different rover tasks

---

## 📁 Workspace Structure

```
src/
├── openvino_cone_detection/
├── cpp_pubsub/
├── py_pubsub/
├── my_launch/
```

---

## 🔹 Packages

### openvino_cone_detection

* Handles cone detection
* Uses OpenVINO for faster inference
* Publishes detection results

### cpp_pubsub

* C++ ROS 2 nodes

### py_pubsub

* Python ROS 2 nodes

### my_launch

* Contains launch files
* Used to run multiple nodes together

---

## ⚙️ Environment & Versions

* ROS 2 Version: Humble
* Operating System: Ubuntu 22.04

---

## 🚀 Launch Instructions

### 🔵 IDMO Task

```
ros2 launch my_launch joy2.launch.py
ros2 run cpp_pubsub 2ps4
ros2 launch my_launch idmo.launch.py
```

---

### 🟢 ABEX Task

```
ros2 launch my_launch joy2.launch.py
ros2 run cpp_pubsub ps4_abex
ros2 launch my_launch abex.launch.py
```

---

### 🔴 RADO Task

#### Recon

```
ros2 launch my_launch joy2.launch.py
ros2 run cpp_pubsub 2ps4
ros2 launch my_launch rado_sensors.launch.py
ros2 launch my_launch rado_recon.launch.py
```

#### Autonomous Delivery

```
ros2 launch my_launch rado_sensors.launch.py
ros2 launch my_launch rado_auto.launch.py
```

---

## 🤖 Arduino (micro-ROS)

* Microcontroller code for ESP32 Micontroller
* Use micro_ros_arduino library built for ROS2 Humble only

**Repository Link:**
Coming Soon

---

## 🦾 Robotic Arm (URDF + MoveIt)

* Contains robot description (URDF)
* Includes MoveIt configuration for motion planning

**Repository Link:**
Coming Soon

---

## 👥 Contributors

* [Jival Dhingra](https://github.com/jivalz) 
* [Prakhar Dwivedi](https://github.com/prakhar-29)
* [Pranav Singh](https://github.com/Pranav-710)

---

## 👥 Team Drishti

Built for the International Rover Challenge (IRC) 2026
Focused on building simple and reliable rover systems.

