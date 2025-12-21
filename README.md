# Kavach

Kavach is an experimental autonomous UAV system focused on real-time perception,
decision-making, and control using computer vision and PX4-based flight systems.

---

## Objective
To design and test autonomous drone behaviors that integrate perception,
navigation, and control logic in realistic simulation environments.

---

## Key Capabilities
- Real-time object detection and tracking
- Autonomous navigation logic
- PX4-based flight control integration
- Simulation-first development approach


## Tech Stack

### Perception & AI
- YOLO (Object Detection)
- OpenCV
- Computer Vision

### UAV & Autonomy
- PX4 Autopilot
- MAVSDK
- SITL / Gazebo Simulation

### Programming
- Python

## System Overview

1. Camera feed captured from simulation or onboard source
2. YOLO-based model performs real-time object detection
3. Detection results processed using OpenCV
4. Navigation and control commands generated
5. Commands sent to PX4 via MAVSDK
6. UAV behavior tested in SITL / Gazebo


---

## Project Status
🚧 Experimental & research-focused  
Used for testing autonomy concepts and control strategies.

---

## Future Work
- Improved tracking and prediction
- Advanced control strategies
- Transition from simulation to real hardware
