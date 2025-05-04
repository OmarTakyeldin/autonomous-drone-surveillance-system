# 🛰️ Autonomous Drone Surveillance System

**An intelligent, battery-aware, multi-drone surveillance simulation**  
*Developed using Python and Webots for persistent, edge-focused aerial monitoring*

---

## 📖 Overview

This project presents a fully autonomous drone-based surveillance framework capable of delivering uninterrupted 24/7 monitoring using two coordinated UAVs. Drones patrol the perimeters of static obstacles using edge-based path planning, dynamically switching roles when battery levels drop below a critical threshold. The system is implemented and tested in the Webots simulation environment using realistic physics and virtual sensors.

---

## 🎯 Key Features

- **Autonomous Multi-Drone Coordination**  
  Two drones operate in a coordinated manner, handing over surveillance duties seamlessly when battery constraints arise.

- **Edge-Optimized Path Planning**  
  Obstacle perimeters are extracted using computer vision. Drones follow optimized paths using the A* algorithm with line-of-sight smoothing.

- **Battery-Aware Operation**  
  Drones monitor their battery in real time. When below 25%, they return to a charging station while the second drone resumes the mission.

- **Finite-State Machine (FSM) Control**  
  Drone behavior — from takeoff and navigation to handover and landing — is governed by a robust FSM.

- **Lightweight Communication Protocol**  
  Drones use Webots’ emitter/receiver devices for simple but effective peer-to-peer signaling during handover.

---

## ⚙️ Requirements

- **Python**: 3.8 or later
- **Simulation Environment**: [Webots R2023b+](https://cyberbotics.com/)
- **Python Dependencies within the code files**: pip install numpy opencv-python matplotlib scikit-image

---

## 🚀 How to Run

### 1. Open project files downloaded

### 2. Set Up Your Environment
Ensure Python 3.8+ is installed 

### 3. Go to the "worlds" file and pick one

### 4. Launch Webots and run the simulation automatically


