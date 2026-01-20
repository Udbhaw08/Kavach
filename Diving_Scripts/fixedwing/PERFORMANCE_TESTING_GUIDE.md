# Fixed-Wing Performance Testing Guide 🛩️

## Overview

This document provides the **maximum performance thresholds** for the PX4 fixed-wing aircraft (plane_cam model) to help you manually test and optimize your dive mission parameters.

---

## 📊 Key Performance Parameters

### Airspeed Limits

| Parameter | PX4 Name | Default | Safe Range | Max Tested | Notes |
|-----------|----------|---------|------------|------------|-------|
| **Minimum Airspeed** | `FW_AIRSPD_MIN` | 10 m/s | 8-12 m/s | - | Below this → STALL! |
| **Trim Airspeed** | `FW_AIRSPD_TRIM` | 15 m/s | 12-18 m/s | - | Cruise speed for loiter |
| **Maximum Airspeed** | `FW_AIRSPD_MAX` | 20 m/s | 20-30 m/s | 35 m/s* | Structural limit |

> [!WARNING]
> **Stall Prevention:** Never let airspeed drop below `FW_AIRSPD_MIN`. Fixed-wings lose lift and crash!

---

### Pitch Angle Limits (Dive Angle)

| Parameter | PX4 Name | Default | Max Aggressive | Notes |
|-----------|----------|---------|----------------|-------|
| **Max Pitch Up** | `FW_P_LIM_MAX` | +45° | +60° | Climb limitation |
| **Max Pitch Down** | `FW_P_LIM_MIN` | -45° | -70°* | **Dive angle limit** |

> [!TIP]
> For aggressive dives, you can push `FW_P_LIM_MIN` to -60° or even -70° in simulation. However, in the real world this depends on the airframe structural limits.

**Testing Matrix for Dive Angles:**

| Dive Angle | Risk Level | Expected Behavior |
|------------|------------|-------------------|
| 35° | ✅ Safe | Smooth, controlled dive |
| 45° | ✅ Moderate | Steeper, airspeed increases faster |
| 55° | ⚠️ Aggressive | High speed buildup, needs careful pull-up |
| 70° | 🔴 Extreme | Near-vertical dive, max stress |

---

### Climb & Descent Rates

| Parameter | PX4 Name | Default | Max Tested | Notes |
|-----------|----------|---------|------------|-------|
| **Max Climb Rate** | `FW_T_CLMB_MAX` | 3 m/s | 5-8 m/s | Depends on thrust/weight |
| **Min Sink Rate** | `FW_T_SINK_MIN` | 2 m/s | - | Glide performance |
| **Max Sink Rate** | `FW_T_SINK_MAX` | 5 m/s | 15 m/s* | During dive |

**Testing Matrix for Descent Speed:**

| Descent Speed | Dive Angle | Risk Level |
|---------------|------------|------------|
| 5 m/s | 35° | ✅ Safe (Current) |
| 8 m/s | 45° | ⚠️ Moderate |
| 12 m/s | 55° | ⚠️ Aggressive |
| 15 m/s | 70° | 🔴 Extreme |

---

### Loiter & Orbit Parameters

| Parameter | PX4 Name | Default | Range | Notes |
|-----------|----------|---------|-------|-------|
| **Loiter Radius** | `NAV_LOITER_RAD` | 50 m | 30-100 m | Smaller = tighter turns |
| **Loiter Speed** | `FW_LTR_AIRSPD` | `FW_AIRSPD_TRIM` | - | Loiter airspeed |

---

### Takeoff Parameters

| Parameter | PX4 Name | Default | Notes |
|-----------|----------|---------|-------|
| **Takeoff Airspeed** | `FW_TKO_AIRSPD` | `FW_AIRSPD_MIN` × 1.3 | Apply throttle until this speed |
| **Takeoff Pitch** | `FW_TKO_PITCH_MIN` | 10° | Initial climb pitch after rotation |
| **Takeoff Throttle** | `FW_THR_MAX` | 1.0 (100%) | Full throttle during takeoff |
| **Runway Length** | (Simulation) | ~200m | Catapult or runway required |

---

## 🎯 Recommended Test Matrix

Use this matrix to systematically test different configurations:

### Test 1: Conservative Baseline

```python
TARGET_ALTITUDE = 70.0     # m
DIVE_ANGLE_DEG = 35.0      # degrees
DESCENT_SPEED = 5.0        # m/s
MIN_SAFE_ALTITUDE = 5.0    # m
CRUISE_SPEED = 15.0        # m/s
```

**Expected Result:** Safe, controlled dive with smooth recovery.

---

### Test 2: Moderate Aggression

```python
TARGET_ALTITUDE = 100.0    # m (higher = more dive distance)
DIVE_ANGLE_DEG = 45.0      # degrees (steeper)
DESCENT_SPEED = 8.0        # m/s (faster)
MIN_SAFE_ALTITUDE = 10.0   # m (higher safety margin)
CRUISE_SPEED = 18.0        # m/s (faster cruise)
```

**Expected Result:** Faster dive, higher terminal velocity, needs earlier pull-up.

---

### Test 3: Aggressive Performance

```python
TARGET_ALTITUDE = 120.0    # m
DIVE_ANGLE_DEG = 55.0      # degrees
DESCENT_SPEED = 12.0       # m/s
MIN_SAFE_ALTITUDE = 15.0   # m
CRUISE_SPEED = 20.0        # m/s
```

**Expected Result:** High-speed dive, significant airspeed buildup, aggressive pull-up required.

---

### Test 4: Maximum Performance (Extreme)

```python
TARGET_ALTITUDE = 150.0    # m
DIVE_ANGLE_DEG = 70.0      # degrees
DESCENT_SPEED = 15.0       # m/s
MIN_SAFE_ALTITUDE = 20.0   # m
CRUISE_SPEED = 25.0        # m/s
```

> [!CAUTION]
> This is experimental! May exceed structural limits in real aircraft. Use only in simulation.

---

## 🔧 How to Modify PX4 Parameters

> [!WARNING]
> **CRITICAL**: The default `mission_dive.py` script NOW AUTOMATICALLY sets aggressive parameters!
> The script backs up, modifies, and restores parameters for you. Manual parameter modification is only needed for custom configurations.

### Automatic Parameter Configuration (Recommended)

The fixed-wing dive script automatically sets these aggressive parameters:

```python
# These are set automatically by mission_dive.py
aggressive_params = {
    "FW_T_SINK_MAX": 15.0,      # Max sink rate (default 2.7 m/s)
    "FW_P_LIM_MIN": -60.0,       # Max pitch down (default -15.0°)
    "FW_AIRSPD_MAX": 30.0,       # Max airspeed (default 20.0 m/s)
}
```

### Default PX4 Parameters for plane_cam

These are the **actual default values** for the plane_cam model:

| Parameter | Default Value | What It Limits |
|-----------|---------------|----------------|
| `FW_T_SINK_MAX` | **2.7 m/s** | Maximum descent rate |
| `FW_P_LIM_MIN` | **-15.0°** | Maximum pitch down (dive angle) |
| `FW_P_LIM_MAX` | **+45.0°** | Maximum pitch up |
| `FW_AIRSPD_MAX` | **20.0 m/s** | Maximum airspeed |
| `FW_AIRSPD_MIN` | **10.0 m/s** | Minimum airspeed (stall) |
| `FW_T_CLMB_MAX` | **5.0 m/s** | Maximum climb rate |

> [!CAUTION]
> **Without modifying these parameters**, your commanded 8 m/s descent will be LIMITED to ~2.5 m/s!

### Manual Parameter Modification (Optional)

If you want to set custom parameters beyond the script defaults:

**Via PX4 Shell (SITL):**
```bash
# View current value
param show FW_T_SINK_MAX

# Set new value
param set FW_T_SINK_MAX 15.0
param set FW_P_LIM_MIN -60.0

# Save parameters (persists across restarts)
param save
```

**Via MAVSDK (in Python):**
```python
# Backup original value
original = await drone.param.get_param_float("FW_T_SINK_MAX")

# Set aggressive value
await drone.param.set_param_float("FW_T_SINK_MAX", 15.0)
await drone.param.set_param_float("FW_P_LIM_MIN", -60.0)

# ... run mission ...

# Restore original
await drone.param.set_param_float("FW_T_SINK_MAX", original)
```

### Why Automatic Configuration is Better

✅ **Safety**: Always restores original parameters  
✅ **Convenience**: No manual PX4 shell commands needed  
✅ **Consistency**: Same configuration every test run  
✅ **Documentation**: Parameters logged in console output

---

## 📈 Performance Comparison: Quadcopter vs Fixed-Wing

| Metric | Quadcopter | Fixed-Wing | Advantage |
|--------|------------|------------|-----------|
| **Max Dive Speed** | ~8 m/s | ~15+ m/s | ✈️ Fixed-Wing |
| **Max Dive Angle** | ~55° (limited by tilt) | ~70° (structural) | ✈️ Fixed-Wing |
| **Terminal Velocity** | Limited by motors | Gravity + aerodynamics | ✈️ Fixed-Wing |
| **Precision Targeting** | ✅ Can hover | ❌ Must loiter | 🚁 Quadcopter |
| **Recovery Time** | Instant (hover) | Slow (pull-up) | 🚁 Quadcopter |

---

## 🧪 Testing Checklist

Use this checklist for each test run:

- [ ] Start PX4 SITL with `make px4_sitl gazebo-classic_plane_cam`
- [ ] Modify script parameters
- [ ] Run mission: `python3 mission_dive.py`
- [ ] Observe in Gazebo
- [ ] Check log file: `../logs/mission_dive_fixedwing_*.csv`
- [ ] Record results in table below

### Test Log Template

| Test # | Altitude | Dive Angle | Descent Speed | Peak Airspeed | Recovery Alt | Result |
|--------|----------|------------|---------------|---------------|--------------|--------|
| 1 | 70m | 35° | 5 m/s | ? m/s | ? m | ✅/❌ |
| 2 | | | | | | |
| 3 | | | | | | |

---

## ⚠️ Safety Limits Summary

| Limit | Value | What Happens If Exceeded |
|-------|-------|--------------------------|
| Min Airspeed | 10 m/s | Stall → Crash |
| Max Pitch Down | -45° (default) | PX4 limits pitch automatically |
| Min Altitude | 5 m (script) | Script triggers abort |
| Max Descent | 15 m/s | Structural stress, hard recovery |

---

## 🎬 Next Steps

1. **Run baseline test** with default parameters
2. **Increment one parameter at a time** (e.g., increase dive angle by 10°)
3. **Log results** in the test log template
4. **Find optimal balance** between speed and safety
5. **Document best configuration** for your use case

**Happy Testing! 🚀**
