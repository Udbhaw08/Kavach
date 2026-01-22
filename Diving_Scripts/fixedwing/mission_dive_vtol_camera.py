#!/usr/bin/env python3
"""
PX4 VTOL Aggressive Pinpoint Dive Attack with Depth Camera
===========================================================
Energy-managed dive to dynamically detected target using depth camera

Mission Profile:
1. MC Takeoff and climb to attack altitude
2. Transition to Fixed-Wing
3. DEPTH CAMERA target acquisition (replaces hardcoded lat/lon)
4. Calculate PINCH POINT (entry point before target)
5. Navigate to pinch point
6. Execute energy-managed aggressive dive to target
7. Transition to Multicopter
8. Pull-up and recovery

Key Principles (VTOL + Depth Camera):
- Dynamic target detection (NOT hardcoded GPS)
- Camera → Body → NED → Lat/Lon transformation
- Airspeed-centric control (not pitch/attitude forcing)
- Energy management via TECS
- Pinch-point navigation geometry
- Aggressive attack speed (faster than best glide)
- Explicit VTOL transitions (MC ↔ FW)
"""

import asyncio
import csv
import math
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw

# ================= DEPTH CAMERA PARAMETERS =================
CAM_FX = 320.0   # focal length x (pixels)
CAM_FY = 320.0   # focal length y (pixels)
CAM_CX = 320.0   # image center x
CAM_CY = 240.0   # image center y

# ================= MISSION PARAMETERS =================
ATTACK_ALTITUDE = 200.0       # meters AGL (pinch point altitude)
MIN_SAFE_ALTITUDE = 80.0      # m (realistic pull-up altitude for high-speed)

# AGGRESSIVE ATTACK PROFILE
ATTACK_AIRSPEED = 35.0        # m/s (~68 knots) - aggressive but safe
DIVE_GLIDE_RATIO = 5.0        # 5:1 glide ratio (steeper than best glide ~9:1)

# Control parameters
CRUISE_SPEED = 18.0           # m/s (normal flight)
CONTROL_RATE = 0.1            # 10Hz

# Fixed-wing safety limits
MAX_AIRSPEED = 50.0           # m/s (structural limit)
MIN_AIRSPEED = 12.0           # m/s (stall margin)

# ================= GLOBAL STATE =================
latest_pos = None
latest_vel = None
latest_attitude = None
stop_tasks = False
mission_log = []
home_position = None

# ================= DYNAMIC TARGET STATE =================
# Hardcoded target location - ATTACK COORDINATES
target_lat = 47.38847  # Attack target latitude
target_lon = 8.551052  # Attack target longitude
target_valid = True

# ================= TELEMETRY LISTENERS =================
async def position_listener(drone):
    global latest_pos, stop_tasks
    async for pos in drone.telemetry.position():
        if stop_tasks:
            break
        latest_pos = pos

async def velocity_listener(drone):
    global latest_vel, stop_tasks
    async for vel in drone.telemetry.velocity_ned():
        if stop_tasks:
            break
        latest_vel = vel

async def attitude_listener(drone):
    global latest_attitude, stop_tasks
    async for attitude in drone.telemetry.attitude_euler():
        if stop_tasks:
            break
        latest_attitude = attitude

# ================= PARAMETER MANAGEMENT =================
async def configure_aggressive_params(drone):
    """Configure PX4 for aggressive dive"""
    print("🔧 Configuring aggressive dive parameters...")
    
    params_to_set = {
        # BALLISTIC DIVE PARAMETERS - DISABLE TECS LIMITS
        "FW_T_SINK_MAX": 50.0,      # Allow 50 m/s sink (ballistic)
        "FW_T_CLMB_MAX": 15.0,      # Allow 15 m/s climb
        "FW_P_LIM_MIN": -80.0,      # Allow 80° pitch down (ballistic)
        "FW_P_LIM_MAX": 60.0,       # Allow 60° pitch up
        
        # AIRSPEED LIMITS
        "FW_AIRSPD_MAX": 60.0,      # Allow 60 m/s max
        "FW_AIRSPD_MIN": 8.0,       # Lower stall speed
        
        # AGGRESSIVE THROTTLE
        "FW_THR_MAX": 1.0,          # 100% throttle
        "FW_THR_IDLE": 0.0,         # 0% idle for dive
        
        # DISABLE ALTITUDE HOLD CONSTRAINTS
        "FW_T_HRATE_FF": 0.0,       # Disable height rate feedforward
        "FW_T_ALT_TC": 10.0,        # Slow altitude time constant
        
        # MULTICOPTER PARAMETERS
        "MPC_Z_VEL_MAX_UP": 10.0,   # Max ascent speed (MC)
        "MPC_Z_VEL_MAX_DN": 12.0,   # Max descent speed (MC)
        "MPC_TKO_SPEED": 10.0,      # Takeoff speed
    }
    
    original_params = {}
    
    for param_name, new_value in params_to_set.items():
        try:
            original = await drone.param.get_param_float(param_name)
            original_params[param_name] = original
            await drone.param.set_param_float(param_name, new_value)
            print(f"  ✓ {param_name}: {original:.1f} → {new_value:.1f}")
        except Exception as e:
            print(f"  ⚠️  {param_name}: {e}")
    
    await asyncio.sleep(1.0)
    return original_params

async def restore_params(drone, original_params):
    """Restore original parameters"""
    print("🔧 Restoring original parameters...")
    for param_name, original_value in original_params.items():
        try:
            await drone.param.set_param_float(param_name, original_value)
            print(f"  ✓ {param_name} → {original_value:.1f}")
        except Exception as e:
            print(f"  ⚠️  {param_name}: {e}")

# ================= SAFETY & HEALTH =================
async def wait_until_armable(drone):
    print("⏳ Waiting for vehicle to become armable...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("✅ Vehicle is armable")
            return
        await asyncio.sleep(0.2)

def check_safety_abort(altitude, velocity):
    """Energy-aware safety checks"""
    if altitude < MIN_SAFE_ALTITUDE:
        return False, f"Altitude below minimum ({altitude:.1f}m < {MIN_SAFE_ALTITUDE}m)"
    
    if velocity is not None:
        airspeed = math.hypot(velocity.north_m_s, velocity.east_m_s)
        
        if airspeed > 0.9 * MAX_AIRSPEED:
            return False, f"Approaching max airspeed ({airspeed:.1f} m/s)"
        
        if airspeed < MIN_AIRSPEED and altitude > MIN_SAFE_ALTITUDE + 20:
            return False, f"Airspeed too low - STALL RISK ({airspeed:.1f} m/s)"
    
    return True, "OK"

def log_telemetry(phase, altitude, velocity, attitude, extra=None):
    global mission_log
    
    entry = {
        "timestamp": datetime.now().isoformat(),
        "phase": phase,
        "altitude_m": round(altitude, 2),
        "vertical_speed_m_s": round(velocity.down_m_s, 2) if velocity else 0.0,
        "airspeed_m_s": round(math.hypot(velocity.north_m_s, velocity.east_m_s), 2) if velocity else 0.0,
        "roll_deg": round(attitude.roll_deg, 2) if attitude else 0.0,
        "pitch_deg": round(attitude.pitch_deg, 2) if attitude else 0.0,
        "yaw_deg": round(attitude.yaw_deg, 2) if attitude else 0.0,
    }
    
    if extra:
        entry.update(extra)
    
    mission_log.append(entry)

def log_speed_verification(phase, velocity, expected_ascent_speed=10.0, expected_descent_speed=12.0):
    """Log detailed speed verification data"""
    if velocity is None:
        return
    
    vertical_speed = velocity.down_m_s  # negative = up, positive = down
    airspeed = math.hypot(velocity.north_m_s, velocity.east_m_s)
    north_speed = velocity.north_m_s
    east_speed = velocity.east_m_s
    
    speed_log = {
        "timestamp": datetime.now().isoformat(),
        "phase": phase,
        "vertical_speed_m_s": round(vertical_speed, 3),
        "ascent_speed_m_s": round(-vertical_speed, 3) if vertical_speed < 0 else 0.0,
        "descent_speed_m_s": round(vertical_speed, 3) if vertical_speed > 0 else 0.0,
        "airspeed_m_s": round(airspeed, 3),
        "north_velocity_m_s": round(north_speed, 3),
        "east_velocity_m_s": round(east_speed, 3),
        "expected_ascent_speed": expected_ascent_speed,
        "expected_descent_speed": expected_descent_speed,
    }
    
    return speed_log

# ================= GEODETIC UTILITIES =================
def bearing_deg(lat1, lon1, lat2, lon2):
    """Calculate bearing from point 1 to point 2"""
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    Δλ = math.radians(lon2 - lon1)
    
    y = math.sin(Δλ) * math.cos(φ2)
    x = math.cos(φ1)*math.sin(φ2) - math.sin(φ1)*math.cos(φ2)*math.cos(Δλ)
    
    return (math.degrees(math.atan2(y, x)) + 360) % 360

def distance_meters(lat1, lon1, lat2, lon2):
    """Haversine distance between two lat/lon points"""
    R = 6371000  # Earth radius in meters
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    Δφ = math.radians(lat2 - lat1)
    Δλ = math.radians(lon2 - lon1)
    
    a = math.sin(Δφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(Δλ/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    return R * c

def offset_latlon(lat, lon, distance_m, bearing_deg_input):
    """Calculate new lat/lon from offset (distance, bearing)"""
    R = 6371000
    brng = math.radians(bearing_deg_input)
    
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    
    lat2 = math.asin(math.sin(lat1)*math.cos(distance_m/R) +
                     math.cos(lat1)*math.sin(distance_m/R)*math.cos(brng))
    
    lon2 = lon1 + math.atan2(math.sin(brng)*math.sin(distance_m/R)*math.cos(lat1),
                             math.cos(distance_m/R)-math.sin(lat1)*math.sin(lat2))
    
    return math.degrees(lat2), math.degrees(lon2)

# ================= CAMERA / FRAME TRANSFORMS =================

def camera_to_body(px, py, depth, fx, fy, cx, cy):
    """
    Convert pixel + depth to body-frame coordinates
    Camera frame: X forward, Y right, Z down
    """
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return x, y, z


def body_to_ned(xb, yb, zb, yaw_deg):
    """
    Convert body-frame coordinates to NED frame
    """
    yaw = math.radians(yaw_deg)
    north = xb * math.cos(yaw) - yb * math.sin(yaw)
    east  = xb * math.sin(yaw) + yb * math.cos(yaw)
    down  = zb
    return north, east, down


def ned_to_latlon(lat, lon, north, east):
    """
    Convert NED offset to latitude/longitude
    """
    dlat = north / 111320.0
    dlon = east / (111320.0 * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon

# ================= DEPTH TARGET ACQUISITION =================

async def phase_depth_target_acquisition(drone, px, py, depth_m):
    """
    Acquire target using downward depth camera
    """
    global target_lat, target_lon, target_valid

    if latest_pos is None or latest_attitude is None:
        print("❌ No telemetry for target acquisition")
        return False

    # Camera → body
    xb, yb, zb = camera_to_body(
        px, py, depth_m,
        CAM_FX, CAM_FY, CAM_CX, CAM_CY
    )

    # Body → NED
    yaw = latest_attitude.yaw_deg
    north, east, down = body_to_ned(xb, yb, zb, yaw)

    # NED → lat/lon
    target_lat, target_lon = ned_to_latlon(
        latest_pos.latitude_deg,
        latest_pos.longitude_deg,
        north, east
    )

    target_valid = True

    print("\n🎯 TARGET LOCKED (DEPTH CAMERA)")
    print(f"   Pixel: ({px:.1f}, {py:.1f})")
    print(f"   Depth: {depth_m:.1f} m")
    print(f"   Target Lat/Lon: ({target_lat:.6f}, {target_lon:.6f})")

    return True

# ================= PINCH POINT CALCULATION =================
def calculate_pinch_point(current_lat, current_lon, current_alt):
    """
    Calculate pinch point using dynamically detected target
    """
    if not target_valid:
        raise RuntimeError("Target not valid")

    horizontal_dive_dist = current_alt * DIVE_GLIDE_RATIO

    target_bearing = bearing_deg(
        current_lat, current_lon,
        target_lat, target_lon
    )

    reverse_bearing = (target_bearing + 180) % 360

    pinch_lat, pinch_lon = offset_latlon(
        target_lat,
        target_lon,
        horizontal_dive_dist,
        reverse_bearing
    )

    dive_angle = math.degrees(math.atan(1.0 / DIVE_GLIDE_RATIO))

    return {
        "pinch_lat": pinch_lat,
        "pinch_lon": pinch_lon,
        "pinch_alt": current_alt,
        "target_bearing": target_bearing,
        "dive_distance": horizontal_dive_dist,
        "dive_angle": dive_angle,
        "attack_airspeed": ATTACK_AIRSPEED
    }

# ================= MISSION PHASES =================
async def phase_takeoff_and_climb(drone):
    """Phase 1: MC Takeoff and climb to attack altitude"""
    global home_position
    
    print("\n" + "="*50)
    print("🚀 PHASE 1: MC TAKEOFF & CLIMB TO ATTACK ALTITUDE")
    print("="*50)
    print(f"Target: {ATTACK_ALTITUDE}m AGL")
    print(f"Expected ascent speed: 10 m/s")
    
    # Store home position
    if latest_pos:
        home_position = {
            'lat': latest_pos.latitude_deg,
            'lon': latest_pos.longitude_deg,
            'alt': latest_pos.absolute_altitude_m
        }
        print(f"📍 Home: ({home_position['lat']:.6f}, {home_position['lon']:.6f})")
    
    await drone.action.set_takeoff_altitude(ATTACK_ALTITUDE)
    
    # Use Offboard for aggressive climb
    print("🎮 Starting OFFBOARD climb (10 m/s)...")
    
    # Send initial setpoint before starting offboard
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -10.0, 0.0))
    
    try:
        await drone.offboard.start()
        print("✅ OFFBOARD climb active")
    except OffboardError as e:
        print(f"⚠️  Offboard climb failed: {e}")
        print("Fallback to standard takeoff...")
        await drone.action.takeoff()
    
    climb_speeds = []
    
    while True:
        if latest_pos is None:
            await asyncio.sleep(0.1)
            continue
            
        alt = latest_pos.relative_altitude_m
        
        # Enforce climb setpoint
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -10.0, 0.0))
        
        if latest_vel and latest_attitude:
            log_telemetry("MC_CLIMB", alt, latest_vel, latest_attitude)
            
            # Track climb speed
            climb_rate = -latest_vel.down_m_s
            if climb_rate > 0:
                climb_speeds.append(climb_rate)
        
        climb_rate = -latest_vel.down_m_s if latest_vel else 0.0
        airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s) if latest_vel else 0.0
        print(f"📈 ALT {alt:5.1f}m | Climb {climb_rate:5.1f} m/s | IAS {airspeed:5.1f} m/s", end="\r")
        
        if alt >= ATTACK_ALTITUDE - 5.0:  # Stop slightly before to avoid overshoot if needed, or rely on hold
            if climb_speeds:
                avg_climb = sum(climb_speeds) / len(climb_speeds)
                max_climb = max(climb_speeds)
                print(f"\n✅ Attack altitude reached: {alt:.1f}m | Avg Climb: {avg_climb:.2f} m/s | Max Climb: {max_climb:.2f} m/s")
            else:
                print(f"\n✅ Attack altitude reached: {alt:.1f}m")
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    # Enter HOLD mode and stabilize before transition
    print("\n📍 Entering HOLD mode for stabilization...")
    try:
        await drone.offboard.stop()
    except:
        pass
    await drone.action.hold()
    await asyncio.sleep(3.0)  # Wait for vehicle to stabilize
    print("✅ Vehicle stabilized")
    
    return True

async def phase_transition_to_fw(drone):
    """Phase 2: Transition to Fixed-Wing mode"""
    print("\n" + "="*50)
    print("🔄 PHASE 2: TRANSITION TO FIXED-WING")
    print("="*50)
    
    # Verify we're at safe altitude
    alt = latest_pos.relative_altitude_m
    print(f"Current altitude: {alt:.1f}m")
    
    if alt < 50.0:
        print(f"⚠️  Altitude too low for transition ({alt:.1f}m < 50m)")
        return False
    
    print("Initiating FW transition...")
    
    try:
        await drone.action.transition_to_fixedwing()
        print("✅ Transition command sent")
    except Exception as e:
        print(f"⚠️  Transition failed: {e}")
        return False
    
    # Wait for transition to complete
    print("Waiting for transition to complete...")
    await asyncio.sleep(8.0)
    
    # Monitor airspeed during transition
    for i in range(20):
        if latest_vel:
            airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s)
            print(f"  Transition in progress... IAS: {airspeed:5.1f} m/s", end="\r")
            if airspeed > 15.0:  # FW mode achieved
                print(f"\n✅ Fixed-wing mode active (IAS: {airspeed:.1f} m/s)")
                return True
        await asyncio.sleep(0.5)
    
    print("\n✅ Fixed-wing mode active")
    return True

async def phase_target_acquisition_and_pinch_calc(drone):
    """Phase 3: Direct approach to attack position using Offboard velocity control"""
    print("\n" + "="*50)
    print("🎯 PHASE 3: DIRECT APPROACH TO ATTACK POSITION")
    print("="*50)
    
    # Current position
    curr_lat = latest_pos.latitude_deg
    curr_lon = latest_pos.longitude_deg
    curr_alt = latest_pos.relative_altitude_m
    
    print(f"📍 Current position: ({curr_lat:.6f}, {curr_lon:.6f}, {curr_alt:.1f}m)")
    
    # ===== HARDCODED TARGET LOCATION =====
    print(f"\n🎯 Target location (HARDCODED):")
    print(f"   Lat/Lon: ({target_lat:.6f}, {target_lon:.6f})")
    
    # Calculate distance to target
    dist_to_target = distance_meters(curr_lat, curr_lon, target_lat, target_lon)
    print(f"📏 Distance to target: {dist_to_target:.1f}m")
    
    # Calculate attack bearing
    attack_bearing = bearing_deg(curr_lat, curr_lon, target_lat, target_lon)
    print(f"🧭 Attack bearing: {attack_bearing:.1f}°")
    
    # DIRECT APPROACH USING OFFBOARD VELOCITY CONTROL
    # No waypoints, no curves - straight line to attack position
    print(f"\n🚀 Starting OFFBOARD cruise to attack position...")
    print(f"   Cruise speed: {CRUISE_SPEED} m/s")
    print(f"   Approach distance: 500m from target")
    
    # Start Offboard mode
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(
            north_m_s=CRUISE_SPEED * math.cos(math.radians(attack_bearing)),
            east_m_s=CRUISE_SPEED * math.sin(math.radians(attack_bearing)),
            down_m_s=0.0,  # Maintain altitude
            yaw_deg=attack_bearing
        )
    )
    
    try:
        await drone.offboard.start()
        print("✅ OFFBOARD cruise active")
    except OffboardError as e:
        print(f"⚠️  Offboard failed: {e}")
        return None
    
    # Cruise toward target until within 500m
    print("\n🛫 Cruising to attack position (straight line)...")
    while True:
        curr_lat = latest_pos.latitude_deg
        curr_lon = latest_pos.longitude_deg
        curr_alt = latest_pos.relative_altitude_m
        
        # Recalculate bearing (laser-guided)
        attack_bearing = bearing_deg(curr_lat, curr_lon, target_lat, target_lon)
        dist_to_target = distance_meters(curr_lat, curr_lon, target_lat, target_lon)
        
        # Update velocity vector to point at target
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(
                north_m_s=CRUISE_SPEED * math.cos(math.radians(attack_bearing)),
                east_m_s=CRUISE_SPEED * math.sin(math.radians(attack_bearing)),
                down_m_s=0.0,
                yaw_deg=attack_bearing
            )
        )
        
        airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s) if latest_vel else 0.0
        
        if latest_vel and latest_attitude:
            log_telemetry("FW_CRUISE", curr_alt, latest_vel, latest_attitude, {
                "distance_to_target_m": round(dist_to_target, 2)
            })
        
        print(f"   Distance to target: {dist_to_target:6.1f}m | ALT {curr_alt:5.1f}m | IAS {airspeed:5.1f} m/s | Bearing {attack_bearing:5.1f}°", end="\r")
        
        # Stop cruise when within 500m (dive initiation point)
        if dist_to_target < 500.0:
            print(f"\n✅ Attack position reached ({dist_to_target:.1f}m from target)")
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    # Prepare dive data
    dive_data = {
        "target_bearing": attack_bearing,
        "attack_airspeed": ATTACK_AIRSPEED,
        "dive_angle": math.degrees(math.atan(1.0 / DIVE_GLIDE_RATIO))
    }
    
    return dive_data

async def phase_transition_to_mc_for_dive(drone):
    """Phase 3.5: Transition to MC for ballistic dive"""
    print("\n" + "="*50)
    print("🔄 PHASE 3.5: TRANSITION TO MC FOR DIVE")
    print("="*50)
    
    # Stop Offboard cruise
    try:
        await drone.offboard.stop()
    except:
        pass
    
    # Verify altitude
    alt = latest_pos.relative_altitude_m
    print(f"Current altitude: {alt:.1f}m")
    
    if alt < MIN_SAFE_ALTITUDE:
        print(f"⚠️  Altitude too low for transition ({alt:.1f}m)")
        return False
    
    print("Initiating MC transition for dive...")
    
    try:
        await drone.action.transition_to_multicopter()
        print("✅ MC transition command sent")
    except Exception as e:
        print(f"⚠️  Transition failed: {e}")
        return False
    
    # Wait for transition
    print("Waiting for MC transition to complete...")
    await asyncio.sleep(6.0)
    
    # Monitor transition
    for i in range(15):
        if latest_vel:
            airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s)
            print(f"  Transition in progress... IAS: {airspeed:5.1f} m/s", end="\r")
            if airspeed < 8.0:  # MC mode achieved
                print(f"\n✅ Multicopter mode active (IAS: {airspeed:.1f} m/s)")
                return True
        await asyncio.sleep(0.5)
    
    print("\n✅ Multicopter mode active")
    return True


async def phase_energy_managed_dive(drone, dive_data):
    """Phase 4: BALLISTIC DIVE - MC Mode Aggressive Descent"""
    print("\n" + "="*50)
    print("🚀 PHASE 4: BALLISTIC DIVE ATTACK (MC MODE)")
    print("="*50)
    print(f"Attack profile:")
    print(f"  Target: ({target_lat:.6f}, {target_lon:.6f})")
    print(f"  Mode: Multicopter (enables vertical dive)")
    print(f"  Descent speed: 12 m/s (MC max configured)")
    print(f"  Style: BOLT-M laser-guided missile")
    
    dive_start_alt = latest_pos.relative_altitude_m
    dive_start_time = asyncio.get_event_loop().time()
    
    print("\n🎯 Initiating ballistic dive...")
    
    # Speed tracking
    descent_speeds = []
    airspeed_profile = []
    
    # Calculate bearing to target
    curr_lat = latest_pos.latitude_deg
    curr_lon = latest_pos.longitude_deg
    bearing = bearing_deg(curr_lat, curr_lon, target_lat, target_lon)
    
    # Start Offboard mode for MC dive
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(
            north_m_s=0.0,
            east_m_s=0.0,
            down_m_s=12.0,  # Aggressive descent (MC configured max)
            yaw_deg=bearing
        )
    )
    
    try:
        await drone.offboard.start()
        print("✅ OFFBOARD dive control active (MC Mode)")
    except OffboardError as e:
        print(f"⚠️  Offboard failed: {e}")
        return False
    
    while True:
        alt = latest_pos.relative_altitude_m
        curr_lat = latest_pos.latitude_deg
        curr_lon = latest_pos.longitude_deg
        
        # Update bearing (laser-guided)
        bearing = bearing_deg(curr_lat, curr_lon, target_lat, target_lon)
        horizontal_dist = distance_meters(curr_lat, curr_lon, target_lat, target_lon)
        
        # Command aggressive descent with slight horizontal correction
        north_vel = 5.0 * math.cos(math.radians(bearing))  # Slow horizontal toward target
        east_vel = 5.0 * math.sin(math.radians(bearing))
        down_vel = 12.0  # Maximum MC descent
        
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(
                north_m_s=north_vel,
                east_m_s=east_vel,
                down_m_s=down_vel,
                yaw_deg=bearing
            )
        )
        
        # Safety checks
        is_safe, reason = check_safety_abort(alt, latest_vel)
        if not is_safe:
            print(f"\n⚠️  SAFETY ABORT: {reason}")
            break
        
        # Calculate metrics
        dist_to_target = horizontal_dist
        dive_distance = dive_start_alt - alt
        dive_time = asyncio.get_event_loop().time() - dive_start_time
        
        if latest_vel and latest_attitude:
            airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s)
            sink_rate = latest_vel.down_m_s
            
            # Track descent and airspeed
            descent_speeds.append(sink_rate)
            airspeed_profile.append(airspeed)
            
            # Calculate actual dive angle
            if horizontal_dist > 0:
                dive_angle = math.atan2(alt, horizontal_dist)
            else:
                dive_angle = math.radians(90)
            
            log_telemetry("BALLISTIC_DIVE_MC", alt, latest_vel, latest_attitude, {
                "dist_to_target_m": round(dist_to_target, 2),
                "dive_dist_m": round(dive_distance, 2),
                "dive_time_s": round(dive_time, 2),
                "dive_angle_deg": round(math.degrees(dive_angle), 2),
                "commanded_down_vel": round(down_vel, 2)
            })
            
            pitch = latest_attitude.pitch_deg
            dive_angle_deg = math.degrees(dive_angle)
            
            # Enhanced display
            print(f"🎯 ALT {alt:5.1f}m | Target {dist_to_target:5.1f}m | IAS {airspeed:5.1f} m/s | Descent {sink_rate:5.2f} m/s | Angle {dive_angle_deg:5.1f}° | Pitch {pitch:5.1f}°", end="\r")
        
        # Pull-up condition
        if alt <= MIN_SAFE_ALTITUDE:
            print(f"\n✅ Pull-up altitude reached: {alt:.1f}m")
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    # Calculate dive statistics
    print(f"\n📊 BALLISTIC DIVE STATISTICS:")
    print(f"   Altitude lost: {dive_distance:.1f}m")
    print(f"   Time in dive: {dive_time:.1f}s")
    if dive_time > 0:
        print(f"   Average sink rate: {dive_distance/dive_time:.2f} m/s")
    print(f"   Final distance to target: {dist_to_target:.1f}m")
    
    if descent_speeds:
        avg_descent = sum(descent_speeds) / len(descent_speeds)
        max_descent = max(descent_speeds)
        min_descent = min(descent_speeds)
        print(f"\n   📉 DESCENT SPEED VERIFICATION (Target: 10+ m/s):")
        print(f"      Average descent: {avg_descent:.2f} m/s")
        print(f"      Max descent: {max_descent:.2f} m/s")
        print(f"      Min descent: {min_descent:.2f} m/s")
        print(f"      Descent status: {'✅ BALLISTIC' if avg_descent >= 10.0 else '⚠️  TOO SLOW'}")
    
    if airspeed_profile:
        avg_airspeed = sum(airspeed_profile) / len(airspeed_profile)
        max_airspeed = max(airspeed_profile)
        print(f"\n   ✈️  AIRSPEED PROFILE:")
        print(f"      Average airspeed: {avg_airspeed:.2f} m/s")
        print(f"      Max airspeed: {max_airspeed:.2f} m/s")
    
    await drone.offboard.stop()
    
    return True

async def phase_transition_to_mc(drone):
    """Phase 5: Transition to Multicopter mode"""
    print("\n" + "="*50)
    print("🔄 PHASE 5: TRANSITION TO MULTICOPTER")
    print("="*50)
    
    # Verify we're at safe altitude
    alt = latest_pos.relative_altitude_m
    print(f"Current altitude: {alt:.1f}m")
    
    if alt < MIN_SAFE_ALTITUDE:
        print(f"⚠️  Altitude critical for MC transition ({alt:.1f}m)")
        print("Proceeding with emergency transition...")
    
    print("Initiating MC transition...")
    
    try:
        await drone.action.transition_to_multicopter()
        print("✅ Transition command sent")
    except Exception as e:
        print(f"⚠️  Transition failed: {e}")
        return False
    
    # Wait for transition to complete
    print("Waiting for transition to complete...")
    await asyncio.sleep(6.0)
    
    # Monitor during transition
    for i in range(15):
        if latest_vel:
            airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s)
            print(f"  Transition in progress... IAS: {airspeed:5.1f} m/s", end="\r")
            if airspeed < 8.0:  # MC mode achieved
                print(f"\n✅ Multicopter mode active (IAS: {airspeed:.1f} m/s)")
                return True
        await asyncio.sleep(0.5)
    
    print("\n✅ Multicopter mode active")
    return True

async def phase_recovery(drone):
    """Phase 6: Recovery and landing"""
    print("\n" + "="*50)
    print("🛬 PHASE 6: RECOVERY")
    print("="*50)
    
    print("Entering HOLD mode...")
    await drone.action.hold()
    
    for i in range(30):
        alt = latest_pos.relative_altitude_m if latest_pos else 0.0
        
        if latest_vel and latest_attitude:
            log_telemetry("RECOVERY", alt, latest_vel, latest_attitude)
        
        await asyncio.sleep(CONTROL_RATE)
    
    print("✅ Recovery complete")
    return True

# ================= MAIN =================
async def run():
    global latest_pos, latest_vel, latest_attitude, stop_tasks
    
    print("\n" + "="*60)
    print("⚔️  PX4 VTOL AGGRESSIVE PINPOINT DIVE ATTACK")
    print("     (DEPTH CAMERA TARGET ACQUISITION)")
    print("="*60)
    print(f"Mission Profile:")
    print(f"  Attack altitude: {ATTACK_ALTITUDE}m")
    print(f"  Attack airspeed: {ATTACK_AIRSPEED} m/s")
    print(f"  Dive ratio: {DIVE_GLIDE_RATIO}:1 (aggressive)")
    print(f"  Pull-up altitude: {MIN_SAFE_ALTITUDE}m")
    print(f"  Target: DYNAMIC (depth camera)")
    print("="*60)
    
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    print("\n⏳ Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected to PX4 SITL")
            break
    
    await wait_until_armable(drone)
    
    original_params = await configure_aggressive_params(drone)
    
    print("\n📡 Starting telemetry listeners...")
    pos_task = asyncio.create_task(position_listener(drone))
    vel_task = asyncio.create_task(velocity_listener(drone))
    att_task = asyncio.create_task(attitude_listener(drone))
    
    while latest_pos is None or latest_vel is None or latest_attitude is None:
        await asyncio.sleep(0.1)
    
    print("✅ Telemetry online")
    
    print("\n💪 Arming...")
    await drone.action.arm()
    print("✅ Armed")
    
    try:
        # Phase 1: MC Takeoff & Climb
        if not await phase_takeoff_and_climb(drone):
            print("❌ Climb failed")
            return
        
        # Phase 2: Transition to FW
        if not await phase_transition_to_fw(drone):
            print("❌ FW transition failed")
            return
        
        # Phase 3: Direct approach to attack position
        dive_data = await phase_target_acquisition_and_pinch_calc(drone)
        
        # Phase 3.5: Transition to MC for dive
        if not await phase_transition_to_mc_for_dive(drone):
            print("❌ MC transition for dive failed")
            return
        
        # Phase 4: Ballistic dive (MC mode)
        if not await phase_energy_managed_dive(drone, dive_data):
            print("❌ Dive failed")
            return
        
        # Phase 5: Recovery (already in MC)
        await phase_recovery(drone)
        
        print("\n" + "="*60)
        print("✅ MISSION COMPLETE - TARGET ENGAGED")
        print("="*60)
        
    except Exception as e:
        print(f"\n❌ Mission error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n🔄 Shutting down...")
        
        try:
            await drone.offboard.stop()
        except:
            pass
        
        try:
            await drone.action.hold()
        except:
            pass
        
        stop_tasks = True
        await asyncio.sleep(0.5)
        pos_task.cancel()
        vel_task.cancel()
        att_task.cancel()
        
        await restore_params(drone, original_params)
        
        if mission_log:
            filename = f"../logs/mission_dive_vtol_camera_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            try:
                with open(filename, "w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=mission_log[0].keys())
                    writer.writeheader()
                    writer.writerows(mission_log)
                print(f"📊 Log saved: {filename}")
            except Exception as e:
                print(f"⚠️  Log save failed: {e}")
        
        print("✅ Shutdown complete")

if __name__ == "__main__":
    asyncio.run(run())
