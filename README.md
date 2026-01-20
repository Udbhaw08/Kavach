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


## 📂 System Components

### 1. [Autonomous Dive Missions](./Diving_Scripts)
Advanced autonomous maneuver simulations for tactical scenarios.

| Vehicle Type | Model | Key Behaviors |
|--------------|-------|---------------|
| **Multicopter** | `typhoon_h480` | High-speed ascent (6m/s), Hover stability, 35° Dive |
| **Fixed-Wing** | `plane_cam` | Runway takeoff, Loiter orbit, High-speed Dive |

**Key Features:**
- Modular mission architecture
- Automated safety checks (Altitude floor, Tilt limits)
- Comprehensive telemetry logging (`logs/` directory)

[👉 View Full Documentation](./Diving_Scripts/README.md)

---

## 🏗️ Project Structure

```text
Kavach/
├── Diving_Scripts/        # Autonomous maneuver logic
│   ├── multicopter/       # Quadcopter-specific missions
│   ├── fixedwing/         # Plane-specific missions
│   ├── logs/              # Mission telemetry logs
│   └── legacy/            # Archived tests
├── models/                # Custom Gazebo models
└── README.md              # Project documentation
```

---

## Future Work
- Improved tracking and prediction
- Advanced control strategies
- Transition from simulation to real hardware
