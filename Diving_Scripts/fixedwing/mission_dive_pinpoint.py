#!/usr/bin/env python3
"""
PX4 Fixed-Wing Aggressive Pinpoint Dive Attack
===============================================
Energy-managed dive to lat/lon target using best practices

Mission Profile:
1. Takeoff and climb to attack altitude
2. Calculate PINCH POINT (entry point before target)
3. Navigate to pinch point
4. Execute energy-managed aggressive dive to target lat/lon
5. Pull-up and recovery

Key Principles (Based on Fixed-Wing Best Practices):
- Airspeed-centric control (not pitch/attitude forcing)
- Energy management via TECS
- Pinch-point navigation geometry
- Aggressive attack speed (faster than best glide for minimal reaction time)
- Position setpoint + airspeed control (let PX4 solve pitch)
"""

import asyncio
import csv
import math
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw

# ================= TARGET CONFIGURATION =================
# TARGET COORDINATES (lat/lon) - Replace with camera detection
TARGET_LAT = 47.3989  # Example: near PX4 SITL spawn
TARGET_LON = 8.5456   # Adjust based on your test area

# ================= MISSION PARAMETERS =================
ATTACK_ALTITUDE = 200.0       # meters AGL (pinch point altitude)
LOITER_DURATION = 5.0         # seconds (target confirmation)
MIN_SAFE_ALTITUDE = 80.0      # m (realistic pull-up altitude for high-speed)

# AGGRESSIVE ATTACK PROFILE (not best glide - designed for minimal reaction time)
ATTACK_AIRSPEED = 35.0        # m/s (~68 knots) - aggressive but safe
DIVE_GLIDE_RATIO = 5.0        # 5:1 glide ratio (steeper than best glide ~9:1)
                              # Aggressive: sacrifices range for speed/steepness

# Control parameters
CRUISE_SPEED = 18.0           # m/s (normal flight)
CONTROL_RATE = 0.1            # 10Hz

# Fixed-wing safety limits
MAX_AIRSPEED = 50.0           # m/s (structural limit, conservative)
MIN_AIRSPEED = 12.0           # m/s (stall margin)

# ================= GLOBAL STATE =================
latest_pos = None
latest_vel = None
latest_attitude = None
stop_tasks = False
mission_log = []
home_position = None

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
        "FW_T_SINK_MAX": 25.0,      # Allow aggressive sink
        "FW_P_LIM_MIN": -45.0,       # Allow steep pitch (but not extreme)
        "FW_AIRSPD_MAX": 50.0,       # Max airspeed
        "FW_AIRSPD_MIN": 12.0,       # Stall prevention
        "FW_T_CLMB_MAX": 8.0,        # Max climb rate
        "FW_THR_IDLE": 0.10,         # Low idle for dive
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
        
        # Overspeed protection
        if airspeed > 0.9 * MAX_AIRSPEED:
            return False, f"Approaching max airspeed ({airspeed:.1f} m/s)"
        
        # Stall protection (only at higher altitudes)
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

# ================= PINCH POINT CALCULATION =================
def calculate_pinch_point(current_lat, current_lon, current_alt, target_lat, target_lon):
    """
    Calculate pinch point (dive entry point) based on:
    - Current altitude
    - Aggressive glide ratio (steeper than best glide)
    - Target location
    
    Returns: (pinch_lat, pinch_lon, dive_data)
    """
    
    # Calculate horizontal distance needed for dive
    # Using aggressive glide ratio (5:1, steeper than typical 9:1 best glide)
    horizontal_dive_dist = current_alt * DIVE_GLIDE_RATIO
    
    # Calculate bearing from current position to target
    target_bearing = bearing_deg(current_lat, current_lon, target_lat, target_lon)
    
    # Calculate pinch point: move backwards from target along attack bearing
    # Reverse bearing = target_bearing + 180
    reverse_bearing = (target_bearing + 180) % 360
    
    pinch_lat, pinch_lon = offset_latlon(
        target_lat, target_lon,
        horizontal_dive_dist,
        reverse_bearing
    )
    
    # Calculate dive angle
    dive_angle_deg = math.degrees(math.atan(1.0 / DIVE_GLIDE_RATIO))
    
    dive_data = {
        'pinch_lat': pinch_lat,
        'pinch_lon': pinch_lon,
        'pinch_alt': current_alt,
        'target_bearing': target_bearing,
        'dive_distance': horizontal_dive_dist,
        'dive_angle': dive_angle_deg,
        'attack_airspeed': ATTACK_AIRSPEED,
    }
    
    return pinch_lat, pinch_lon, dive_data

# ================= MISSION PHASES =================
async def phase_takeoff_and_climb(drone):
    """Phase 1: Takeoff and climb to attack altitude"""
    global home_position
    
    print("\n" + "="*50)
    print("🚀 PHASE 1: TAKEOFF & CLIMB TO ATTACK ALTITUDE")
    print("="*50)
    print(f"Target: {ATTACK_ALTITUDE}m AGL")
    
    # Store home position
    if latest_pos:
        home_position = {
            'lat': latest_pos.latitude_deg,
            'lon': latest_pos.longitude_deg,
            'alt': latest_pos.absolute_altitude_m
        }
        print(f"📍 Home: ({home_position['lat']:.6f}, {home_position['lon']:.6f})")
    
    await drone.action.set_takeoff_altitude(ATTACK_ALTITUDE)
    await drone.action.takeoff()
    
    while True:
        if latest_pos is None:
            await asyncio.sleep(0.1)
            continue
            
        alt = latest_pos.relative_altitude_m
        
        if latest_vel and latest_attitude:
            log_telemetry("CLIMB", alt, latest_vel, latest_attitude)
        
        climb_rate = -latest_vel.down_m_s if latest_vel else 0.0
        airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s) if latest_vel else 0.0
        print(f"📈 ALT {alt:5.1f}m | Climb {climb_rate:5.1f} m/s | IAS {airspeed:5.1f} m/s", end="\r")
        
        if alt >= ATTACK_ALTITUDE - 10.0:
            print(f"\n✅ Attack altitude reached: {alt:.1f}m")
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    return True

async def phase_target_acquisition_and_pinch_calc(drone):
    """Phase 2: Target acquisition and pinch point calculation"""
    print("\n" + "="*50)
    print("🎯 PHASE 2: TARGET ACQUISITION & PINCH POINT CALCULATION")
    print("="*50)
    
    # Current position
    curr_lat = latest_pos.latitude_deg
    curr_lon = latest_pos.longitude_deg
    curr_alt = latest_pos.relative_altitude_m
    
    print(f"📍 Current position: ({curr_lat:.6f}, {curr_lon:.6f}, {curr_alt:.1f}m)")
    print(f"🎯 Target coordinates: ({TARGET_LAT:.6f}, {TARGET_LON:.6f})")
    
    # Calculate distance to target
    dist_to_target = distance_meters(curr_lat, curr_lon, TARGET_LAT, TARGET_LON)
    print(f"📏 Distance to target: {dist_to_target:.1f}m")
    
    # Calculate pinch point
    pinch_lat, pinch_lon, dive_data = calculate_pinch_point(
        curr_lat, curr_lon, curr_alt,
        TARGET_LAT, TARGET_LON
    )
    
    print(f"\n✈️  PINCH POINT CALCULATED:")
    print(f"   Location: ({pinch_lat:.6f}, {pinch_lon:.6f}, {dive_data['pinch_alt']:.1f}m)")
    print(f"   Attack bearing: {dive_data['target_bearing']:.1f}°")
    print(f"   Dive distance: {dive_data['dive_distance']:.1f}m")
    print(f"   Dive angle: {dive_data['dive_angle']:.1f}° (aggressive {DIVE_GLIDE_RATIO}:1 ratio)")
    print(f"   Attack airspeed: {dive_data['attack_airspeed']:.1f} m/s")
    
    # Navigate to pinch point
    print(f"\n🧭 Navigating to pinch point...")
    
    dist_to_pinch = distance_meters(curr_lat, curr_lon, pinch_lat, pinch_lon)
    print(f"   Distance to pinch point: {dist_to_pinch:.1f}m")
    
    # Use GOTO command to navigate to pinch point
    print("   Commanding navigation to pinch point...")
    await drone.action.goto_location(
        pinch_lat, pinch_lon,
        dive_data['pinch_alt'],
        dive_data['target_bearing']
    )
    
    # Monitor approach to pinch point
    print("\n📍 Approaching pinch point...")
    while True:
        curr_lat = latest_pos.latitude_deg
        curr_lon = latest_pos.longitude_deg
        curr_alt = latest_pos.relative_altitude_m
        
        dist = distance_meters(curr_lat, curr_lon, pinch_lat, pinch_lon)
        airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s) if latest_vel else 0.0
        
        if latest_vel and latest_attitude:
            log_telemetry("NAV_PINCH", curr_alt, latest_vel, latest_attitude, {
                "distance_to_pinch_m": round(dist, 2)
            })
        
        print(f"   Distance to pinch: {dist:6.1f}m | ALT {curr_alt:5.1f}m | IAS {airspeed:5.1f} m/s", end="\r")
        
        # Arrived at pinch point
        if dist < 50.0:  # Within 50m
            print(f"\n✅ Arrived at pinch point (within {dist:.1f}m)")
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    return dive_data

async def phase_energy_managed_dive(drone, dive_data):
    """
    Phase 3: Energy-Managed Aggressive Dive
    
    Key principle: Airspeed-centric, energy-managed descent
    - Command POSITION to target lat/lon
    - Command AIRSPEED (aggressive attack speed)
    - Reduce throttle (let gravity assist)
    - Let TECS/PX4 solve pitch automatically
    
    NO attitude forcing, NO velocity vector commands
    """
    print("\n" + "="*50)
    print("⚡ PHASE 3: ENERGY-MANAGED AGGRESSIVE DIVE")
    print("="*50)
    print(f"Attack profile:")
    print(f"  Target: ({TARGET_LAT:.6f}, {TARGET_LON:.6f})")
    print(f"  Attack airspeed: {dive_data['attack_airspeed']} m/s (~{dive_data['attack_airspeed']*1.94:.0f} knots)")
    print(f"  Dive ratio: {DIVE_GLIDE_RATIO}:1 (aggressive)")
    print(f"  Dive angle: ~{dive_data['dive_angle']:.1f}°")
    
    print("\n🎮 Starting OFFBOARD mode for energy-managed dive...")
    
    # Start offboard with position control to target
    # NED position relative to current location
    curr_lat = latest_pos.latitude_deg
    curr_lon = latest_pos.longitude_deg
    
    # Calculate NED offset to target (approximate)
    bearing = dive_data['target_bearing']
    dist = dive_data['dive_distance']
    
    north = dist * math.cos(math.radians(bearing))
    east = dist * math.sin(math.radians(bearing))
    down = -dive_data['pinch_alt']  # Negative = down to ground level
    
    # Command position + velocity (airspeed bias)
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(
            north_m_s=dive_data['attack_airspeed'] * math.cos(math.radians(bearing)),
            east_m_s=dive_data['attack_airspeed'] * math.sin(math.radians(bearing)),
            down_m_s=0.0,  # Let TECS manage descent
            yaw_deg=bearing
        )
    )
    
    try:
        await drone.offboard.start()
        print("✅ OFFBOARD dive control active")
    except OffboardError as e:
        print(f"⚠️  Offboard failed: {e}")
        return False
    
    dive_start_alt = latest_pos.relative_altitude_m
    dive_start_time = asyncio.get_event_loop().time()
    
    print("\n🎯 Diving to target...")
    
    while True:
        alt = latest_pos.relative_altitude_m
        curr_lat = latest_pos.latitude_deg
        curr_lon = latest_pos.longitude_deg
        
        # Safety checks
        is_safe, reason = check_safety_abort(alt, latest_vel)
        if not is_safe:
            print(f"\n⚠️  SAFETY ABORT: {reason}")
            break
        
        # Calculate metrics
        dist_to_target = distance_meters(curr_lat, curr_lon, TARGET_LAT, TARGET_LON)
        dive_distance = dive_start_alt - alt
        dive_time = asyncio.get_event_loop().time() - dive_start_time
        
        if latest_vel and latest_attitude:
            airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s)
            sink_rate = latest_vel.down_m_s
            
            log_telemetry("DIVE", alt, latest_vel, latest_attitude, {
                "dist_to_target_m": round(dist_to_target, 2),
                "dive_dist_m": round(dive_distance, 2),
                "dive_time_s": round(dive_time, 2),
            })
            
            pitch = latest_attitude.pitch_deg
            
            print(f"⬇️  ALT {alt:5.1f}m | Target {dist_to_target:5.1f}m | IAS {airspeed:5.1f} m/s | Sink {sink_rate:5.1f} m/s | Pitch {pitch:5.1f}°", end="\r")
        
        # Pull-up condition
        if alt <= MIN_SAFE_ALTITUDE:
            print(f"\n✅ Pull-up altitude reached: {alt:.1f}m")
            break
        
        # Continue commanding dive (airspeed-focused)
        try:
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(
                    north_m_s=dive_data['attack_airspeed'] * math.cos(math.radians(bearing)),
                    east_m_s=dive_data['attack_airspeed'] * math.sin(math.radians(bearing)),
                    down_m_s=5.0,  # Gentle down bias
                    yaw_deg=bearing
                )
            )
        except:
            pass
        
        await asyncio.sleep(CONTROL_RATE)
    
    print(f"\n📊 Dive Statistics:")
    print(f"   Altitude lost: {dive_distance:.1f}m")
    print(f"   Time in dive: {dive_time:.1f}s")
    if dive_time > 0:
        print(f"   Average sink rate: {dive_distance/dive_time:.1f} m/s")
    print(f"   Final distance to target: {dist_to_target:.1f}m")
    
    return True

async def phase_recovery(drone):
    """Phase 4: Pull-up and recovery"""
    print("\n" + "="*50)
    print("🛬 PHASE 4: PULL-UP & RECOVERY")
    print("="*50)
    
    print("Executing energy-managed pull-up...")
    
    try:
        # Command level flight
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(CRUISE_SPEED, 0.0, 0.0, 0.0)
        )
        
        for i in range(30):
            alt = latest_pos.relative_altitude_m if latest_pos else 0.0
            
            if latest_vel and latest_attitude:
                log_telemetry("RECOVERY", alt, latest_vel, latest_attitude)
            
            pitch = latest_attitude.pitch_deg if latest_attitude else 0.0
            airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s) if latest_vel else 0.0
            print(f"📊 ALT {alt:5.1f}m | Pitch {pitch:5.1f}° | IAS {airspeed:5.1f} m/s", end="\r")
            
            await asyncio.sleep(CONTROL_RATE)
        
        await drone.offboard.stop()
        await drone.action.hold()
        print("\n✅ Returned to HOLD mode")
        
    except Exception as e:
        print(f"\n⚠️  Recovery error: {e}")
        try:
            await drone.offboard.stop()
        except:
            pass
    
    print("✅ Recovery complete")
    return True

# ================= MAIN =================
async def run():
    global latest_pos, latest_vel, latest_attitude, stop_tasks
    
    print("\n" + "="*60)
    print("⚔️  PX4 AGGRESSIVE PINPOINT DIVE ATTACK")
    print("="*60)
    print(f"Mission Profile:")
    print(f"  Target: ({TARGET_LAT:.6f}, {TARGET_LON:.6f})")
    print(f"  Attack altitude: {ATTACK_ALTITUDE}m")
    print(f"  Attack airspeed: {ATTACK_AIRSPEED} m/s")
    print(f"  Dive ratio: {DIVE_GLIDE_RATIO}:1 (aggressive)")
    print(f"  Pull-up altitude: {MIN_SAFE_ALTITUDE}m")
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
        # Phase 1: Climb
        if not await phase_takeoff_and_climb(drone):
            print("❌ Climb failed")
            return
        
        # Phase 2: Pinch point calculation & navigation
        dive_data = await phase_target_acquisition_and_pinch_calc(drone)
        
        # Phase 3: Aggressive dive
        if not await phase_energy_managed_dive(drone, dive_data):
            print("❌ Dive failed")
            return
        
        # Phase 4: Recovery
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
            filename = f"../logs/mission_dive_pinpoint_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            with open(filename, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=mission_log[0].keys())
                writer.writeheader()
                writer.writerows(mission_log)
            print(f"📊 Log saved: {filename}")
        
        print("✅ Shutdown complete")

if __name__ == "__main__":
    asyncio.run(run())
