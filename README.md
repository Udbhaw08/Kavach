# Kavach Project

Kavach is an experimental autonomous UAV system focused on real-time perception, decision-making, and advanced control maneuvers using PX4-based flight systems and MAVSDK. The project aims to develop autonomous drone behaviors, including high-speed dives, VTOL transitions, and vision-based targeting.

---

## 🚀 Key Capabilities

*   **Autonomous Dive Missions**: High-speed ballistic dives for fixed-wing and VTOL aircraft.
*   **VTOL Operations**: Analysis and implementation of transition logic (Multicopter $\leftrightarrow$ Fixed-Wing) and hover stability.
*   **Computer Vision Integration**: 
    *   Real-time object detection (Color-based, YOLO).
    *   Integration with Gazebo camera sensors (Depth, RGB).
    *   Custom Python scripts for image processing (`gazebo_camera_poc.py`).
*   **Custom Simulation Models**: 
    *   `vtol_downward_depth_camera`: A standard VTOL with a 70° downward-pitched depth camera.
    *   `depth_camera`: Standalone sensor model.

## 🛠️ Tech Stack

*   **Flight Control**: PX4 Autopilot (v1.14+)
*   **Simulation**: Gazebo Classic (Simulated SITL)
*   **SDK**: MAVSDK-Python
*   **Language**: Python 3.10+, Bash
*   **Vision**: OpenCV, NumPy

---

## 📂 Project Structure

```text
Kavach/
├── Diving_Scripts/                  # Core autonomous mission scripts
│   ├── fixedwing/                   # Plans & Dive logic for fixed-wing/VTOL
│   │   ├── mission_aggressive_dive_optimized.py  # Optimized dive logic
│   │   ├── mission_dive_vtol_camera.py           # VTOL-specific dive with camera
│   │   ├── mission_dive_pinpoint.py              # Precision dive targeting
│   │   └── ...
│   ├── multicopter/                 # Quadcopter-specific missions
│   │   └── mission_dive.py
│   └── logs/                        # Telemetry and flight logs
├── models/                          # Custom Gazebo/SDF models
├── launch_vtol_large_world.sh       # Main simulation launcher
├── gazebo_camera_poc.py             # Camera Proof-of-Concept script
└── ...
```

---

## ⚡ Quick Start

### 1. Launch Simulation
The primary launch script initializes the PX4 SITL environment with the custom VTOL model in Gazebo.

```bash
cd /home/udbhaw/Kavach
./launch_vtol_large_world.sh
```

*This script will:*
*   Kill existing PX4/Gazebo instances.
*   Build and launch `px4_sitl` with `gazebo-classic_vtol_downward_depth_camera`.

### 2. Run Mission Scripts
Open a new terminal to run the mission scripts.

**VTOL Camera Dive:**
```bash
python3 Diving_Scripts/fixedwing/mission_dive_vtol_camera.py
```

**Fixed-Wing Aggressive Dive:**
```bash
python3 Diving_Scripts/fixedwing/mission_aggressive_dive_optimized.py
```

### 3. Run Camera PoC
To verify the camera feed and object detection:
```bash
python3 gazebo_camera_poc.py
```

---

## 📝 Recent Developments & Workflow

### 1. VTOL Camera Configuration
*   **Model**: Modified `vtol_downward_depth_camera` to include a depth camera pitched at 70° (1.22 rad) for better ground visibility during forward flight.
*   **Status**: Successfully integrated and visible in Gazebo.

### 2. Dive Mission Logic
*   **Fixed-Wing**: Debugged altitude hold issues and optimized dive parameters (`FW_T_CLMB_MAX`, `FW_T_SINK_MAX`) for steeper, faster descents.
*   **VTOL**: Implemented transition logic to handle `COMMAND_DENIED` errors by ensuring proper mode switching (Hold -> Stabilized -> Mission).

### 3. Vision System
*   **PoC**: Established a pipeline to read raw image data from Gazebo topics via `pygazebo` and process it with OpenCV.
*   **Detection**: Basic color tracking implemented as a proof of concept.

---

## ⚠️ Common Issues & Fixes

*   **`COMMAND_DENIED` during Transition**: Ensure the vehicle is armed and in a stable mode (e.g., Hold/Position) before commanding a transition. The scripts include retry logic for this.
*   **Gazebo Path Issues**: If models are missing, run `source Tools/simulation/gazebo-classic/setup_gazebo.bash` in the PX4 directory or check `GAZEBO_MODEL_PATH`.

---

## 🗓️ Future Works
*   Integrate YOLOv8 for advanced object detection.
*   Refine terminal phase guidance for dive missions.
*   Hardware-in-the-Loop (HITL) testing.
