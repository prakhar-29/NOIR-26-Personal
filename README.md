# 🚀 Team Drishti – ROS 2 Workspace  
International Rover Challenge (IRC) 2026

---

## 📌 Overview
This repository contains the ROS 2 workspace used by Team Drishti for the International Rover Challenge 2026.

It includes:
- Cone detection using OpenVINO  
- ROS 2 communication (C++ and Python)  
- Launch system for running different rover tasks  

---

## 📁 Workspace Structure

src/
├── openvino_cone_detection/
├── cpp_pubsub/
├── py_pubsub/
├── my_launch/

---

## 🔹 Packages

### openvino_cone_detection
- Handles cone detection  
- Uses OpenVINO for faster inference  
- Publishes detection results  

### cpp_pubsub
- C++ ROS 2 nodes  
- Used for faster communication  

### py_pubsub
- Python ROS 2 nodes  
- Used for simple logic and testing  

### my_launch
- Contains launch files  
- Used to run multiple nodes together  

---

## 🔄 System Flow (Simple)

- Camera input → openvino_cone_detection  
- Detection results → ROS topics  
- Other nodes (cpp_pubsub, py_pubsub) use this data  
- Tasks are executed based on this flow  

---

## ⚙️ Environment & Versions

- ROS 2 Version: (e.g., Humble / Jazzy)  
- Operating System: (e.g., Ubuntu 22.04)  
- OpenVINO Version:  
- Python Version:  

---

## 🚀 Launch Instructions (Fill This Section)

### IDMO Task
# Add command here
Description:
- ros2 launch my_launch joy2.launch.py 
- ros2 run cpp_pubsub 2ps4
- ros2 launch my_launch idmo.launch.py

---

### ABEX Task
# Add command here
Description:
- ros2 launch my_launch joy2.launch.py 
- ros2 run cpp_pubsub ps4_abex
- ros2 launch my_launch abex.launch.py

---

### RADO Task
# Add command here
Description:
Recon:
- ros2 launch my_launch joy2.launch.py 
- ros2 run cpp_pubsub 2ps4
- ros2 launch my_launch rado_sensors.launch.py
- ros2 launch my_launch rado_recon.launch.py
---
Autonomous Delivery:
- ros2 launch my_launch rado_sensors.launch.py
- ros2 launch my_launch rado_auto.launch.py

## 🤖 Arduino (micro-ROS)

- Microcontroller code for rover hardware communication  
- Uses micro-ROS for interfacing with ROS 2  

Repository Link:
Coming Soon:

---

## 🦾 Robotic Arm (URDF + MoveIt)

- Contains robot description (URDF)  
- Includes MoveIt configuration for motion planning  

Repository Link:
Coming Soon:

---

## 👥 Contributors

- @jivalz  
- @prakhar-29

---

## 👥 Team Drishti

Built for the International Rover Challenge (IRC) 2026  
Focused on building simple and reliable rover systems.
