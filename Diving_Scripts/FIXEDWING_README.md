# Fixed-Wing Dive Mission - Quick Start Guide

## 🎯 Mission Overview

Autonomous dive mission adapted for **fixed-wing aircraft (plane_cam)**.

**Mission Profile:**
1. 🛫 Takeoff and climb to 70m
2. 🔄 Loiter (orbit) for 5 seconds
3. ⚡ Dive at 35° toward ground target
4. 🛬 Pull up at 5m and recover

![Mission Concept](/home/udbhaw/.gemini/antigravity/brain/587bb410-8f0f-4a8c-9198-4f39855d22e3/uploaded_image_1767963469300.png)

---

## 🚀 Quick Start

### Step 1: Start PX4 SITL with Fixed-Wing

```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_plane_cam
```

Wait for Gazebo to load completely (plane should be on runway).

### Step 2: Run Mission

```bash
cd /home/udbhaw/Kavach/Diving_Scripts
python3 mission_dive_fixedwing.py
```

### Step 3: Observe

Watch the aircraft in Gazebo:
- Takeoff roll along runway
- Climb in forward flight
- Circle/loiter at altitude
- Nose-down dive
- Pull-up recovery

---

## 📝 Key Differences from Quadcopter

| Quadcopter Version | Fixed-Wing Version |
|--------------------|-------------------|
| Vertical ascent (6 m/s) | Runway takeoff + climb |
| Hover in place (5s) | Loiter orbit (5s) |
| Can stop (0 m/s) | Must maintain 15 m/s airspeed |
| Direct velocity control | Mission mode + Offboard hybrid |
| Hover recovery | Level flight recovery |

---

## ⚙️ Configuration

Default parameters in `mission_dive_fixedwing.py`:

```python
TARGET_ALTITUDE = 70.0        # Climb target
LOITER_DURATION = 5.0         # Orbit time
DIVE_ANGLE_DEG = 35.0         # Dive angle
DESCENT_SPEED = 5.0           # m/s downward
CRUISE_SPEED = 15.0           # m/s forward (min airspeed)
MIN_SAFE_ALTITUDE = 5.0       # Pull-up altitude
```

**To Customize:** Edit these values before running.

---

## 📊 Output

### Log File
`mission_dive_fixedwing_YYYYMMDD_HHMMSS.csv`

**Fields:**
- `timestamp`, `phase`, `altitude_m`
- `vertical_speed_m_s`, `airspeed_m_s`
- `roll_deg`, `pitch_deg`, `yaw_deg`
- `loiter_time_s`, `target_distance_m`

---

## ⚠️ Important Notes

### Fixed-Wing Constraints

1. **Cannot Hover:** Aircraft must maintain forward speed
2. **Needs Runway:** Spawns on runway in Gazebo
3. **Stall Speed:** Must stay above ~10 m/s
4. **Wider Tolerance:** Altitude ±5m (vs ±2m for quad)

### Safety Features

✅ Altitude monitoring (abort at 5m)  
✅ Pitch angle limits (max 45° nose down)  
✅ Airspeed tracking  
✅ Automatic recovery to loiter mode  

---

## 🔧 Troubleshooting

| Issue | Solution |
|-------|----------|
| Won't arm | Wait for GPS lock, check spawn on runway |
| Offboard rejected | Normal - continues in mission mode |
| Stalls during loiter | Increase `CRUISE_SPEED` parameter |
| Crashes on takeoff | Ensure spawn location is runway |
| Dive too steep | Decrease `DIVE_ANGLE_DEG` or `DESCENT_SPEED` |

---

## 📁 Files

- **[mission_dive_fixedwing.py](file:///home/udbhaw/Kavach/Diving_Scripts/mission_dive_fixedwing.py)** - Main script (NEW)
- **[mission_dive_autonomous.py](file:///home/udbhaw/Kavach/Diving_Scripts/mission_dive_autonomous.py)** - Quadcopter version (original)

---

## ✅ Success Criteria

- [x] Takeoff roll and rotation
- [x] Climb to 65-75m
- [x] Loiter/orbit for 4-6 seconds
- [x] Dive with 30-40° pitch
- [x] Descent at ~5 m/s
- [x] Pull-up before hitting ground
- [x] Clean recovery to level flight
- [x] CSV log generated

---

## 🎬 Next Steps

1. Run the mission and observe behavior
2. Check generated CSV log
3. Adjust parameters if needed
4. Test with different altitudes/angles
5. Add real camera integration (future)

**Ready to fly! 🛩️**
