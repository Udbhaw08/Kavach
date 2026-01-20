# PX4 Autonomous Dive Mission Scripts 🚁✈️

Autonomous dive attack simulations for PX4-based drones types (Multicopter & Fixed-Wing) using MAVSDK-Python.

## 📂 Directory Structure

The repository is organized by vehicle type to ensure clean separation of concerns:

```text
Diving_Scripts/
├── multicopter/           # Quadcopter (typhoon_h480) specific code
│   ├── mission_dive.py    # Main mission script for quadcopters
│   ├── analyze_log.py     # Log analysis tool
│   └── README.md          # Multi-copter specific guide
│
├── fixedwing/             # Fixed-Wing (plane_cam) specific code
│   ├── mission_dive.py    # Main mission script for planes
│   └── README.md          # Fixed-wing specific guide
│
├── logs/                  # 📊 Centralized Mission Logs
│   └── *.csv              # All mission telemetry logs are saved here
│
└── legacy/                # Old/Deprecated scripts
```

---

## 🚀 Getting Started

### Prerequisites

- **PX4-Autopilot** (Source code installed)
- **Gazebo Classic** (Simulation environment)
- **MAVSDK-Python** (`pip install mavsdk`)
- **Pandas** (For log analysis: `pip install pandas`)

---

## 🕹️ Usage Guide

### Option 1: Multicopter Mission (Quadcopter)

A 4-phase mission: **Vertical Ascent (6 m/s) → Hover (5s) → Dive (35°) → Recovery**.

1. **Start Simulator:**
   ```bash
   cd ~/PX4-Autopilot
   make px4_sitl gazebo-classic_typhoon_h480
   ```

2. **Run Mission:**
   ```bash
   cd multicopter
   python3 mission_dive.py
   ```

### Option 2: Fixed-Wing Mission (Plane)

A 4-phase mission: **Runway Takeoff → Loiter Orbit → Dive (35°) → Pull-up**.

1. **Start Simulator:**
   ```bash
   cd ~/PX4-Autopilot
   make px4_sitl gazebo-classic_plane_cam
   ```

2. **Run Mission:**
   ```bash
   cd fixedwing
   python3 mission_dive.py
   ```

---

## 📊 Telemetry & Logging

All missions automatically generate detailed CSV logs in the `logs/` directory.

### Log Analysis
We provide a tool to analyze mission performance (altitude, speed, dive angles):

```bash
cd multicopter
# Analyze a specific log file
python3 analyze_log.py ../logs/mission_dive_YYYYMMDD_HHMMSS.csv
```

---

## 🛡️ Safety Features
Both mission types include robust safety monitoring:
- **Altitude Floor:** Aborts mission if below 5m.
- **Tilt Limits:** Monitors pitch/roll to prevent loss of control.
- **Fail-safe:** Automatic recovery to Hover (Multicopter) or Loiter (Fixed-wing) on error.
- **Parameter Management:** Aggressive flight parameters are applied only during the mission and restored afterwards.
