#!/usr/bin/env python3
"""
PX4 Fixed-Wing Targeted Location Dive Mission
==============================================
Dive to specific GPS coordinates (x, y) instead of ground-based angle

Mission Profile:
1. Takeoff and climb to 200m altitude
2. Loiter/orbit for target acquisition (5 seconds)
3. IDENTIFY SPECIFIC TARGET LOCATION (x, y) - simulating camera detection
4. Calculate dive vector pointing DIRECTLY to that location
5. Execute high-speed dive toward specific coordinates
6. Pull up at safe altitude and recover

Ultimate Goal: Camera-based tank detection and targeted dive attack
Currently: Simulates target identification with configurable (x, y) coordinates
"""

import asyncio
import csv
import math
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, Attitude

# ================= TARGET CONFIGURATION =================
# SIMULATED TARGET LOCATION (will be replaced by camera detection)
# These are NED coordinates relative to takeoff position
TARGET_LOCATION_NORTH = 300.0  # meters north of takeoff
TARGET_LOCATION_EAST = -200.0   # meters east of takeoff (negative = west)

# Mission parameters
TARGET_ALTITUDE = 200.0       # meters AGL
LOITER_DURATION = 5.0         # seconds (target identification time)
MIN_SAFE_ALTITUDE = 15.0      # pull-up altitude
CRUISE_SPEED = 20.0           # m/s (normal flight speed)
CONTROL_RATE = 0.1            # 10Hz control loop

# Dive speed parameters
MAX_DIVE_SPEED = 45.0         # m/s (maximum descent speed)
MAX_GLIDE_SPEED = 90.0        # m/s (maximum horizontal speed)

# Fixed-wing safety limits
MAX_PITCH_DOWN = 70.0         # degrees (aggressive dive limit)
MAX_AIRSPEED = 100.0          # m/s (structural limit)
MIN_AIRSPEED = 8.0            # m/s (stall prevention)

# ================= GLOBAL STATE =================
latest_pos = None
latest_vel = None
latest_attitude = None
stop_tasks = False
mission_log = []
home_position = None  # Store takeoff position for NED calculations

# ================= TELEMETRY LISTENERS =================
async def position_listener(drone):
    """Continuously update global position"""
    global latest_pos, stop_tasks
    async for pos in drone.telemetry.position():
        if stop_tasks:
            break
        latest_pos = pos


async def velocity_listener(drone):
    """Continuously update global velocity"""
    global latest_vel, stop_tasks
    async for vel in drone.telemetry.velocity_ned():
        if stop_tasks:
            break
        latest_vel = vel


async def attitude_listener(drone):
    """Continuously update global attitude"""
    global latest_attitude, stop_tasks
    async for attitude in drone.telemetry.attitude_euler():
        if stop_tasks:
            break
        latest_attitude = attitude


# ================= PARAMETER MANAGEMENT =================
async def configure_aggressive_params(drone):
    """Set parameters for high-speed dive with backup"""
    print("🔧 Configuring aggressive dive parameters...")
    
    params_to_set = {
        "FW_T_SINK_MAX": 50.0,      # Max sink rate
        "FW_P_LIM_MIN": -70.0,       # Max pitch down
        "FW_AIRSPD_MAX": 100.0,      # Max airspeed
        "FW_AIRSPD_MIN": 8.0,        # Min airspeed (stall prevention)
        "FW_T_CLMB_MAX": 8.0,        # Max climb rate
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
    """Restore parameters to original values"""
    print("🔧 Restoring original parameters...")
    for param_name, original_value in original_params.items():
        try:
            await drone.param.set_param_float(param_name, original_value)
            print(f"  ✓ {param_name} → {original_value:.1f}")
        except Exception as e:
            print(f"  ⚠️  {param_name}: {e}")


# ================= SAFETY & HEALTH =================
async def wait_until_armable(drone):
    """Wait for vehicle to be ready for arming"""
    print("⏳ Waiting for vehicle to become armable...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("✅ Vehicle is armable")
            return
        await asyncio.sleep(0.2)


def check_safety_abort(altitude, attitude, velocity):
    """
    Check safety conditions for high-speed dive
    Returns: (is_safe: bool, reason: str)
    """
    if altitude < MIN_SAFE_ALTITUDE:
        return False, f"Altitude below minimum ({altitude:.1f}m < {MIN_SAFE_ALTITUDE}m)"
    
    if attitude is not None:
        pitch_down = -attitude.pitch_deg
        if pitch_down > MAX_PITCH_DOWN:
            return False, f"Dive too steep ({pitch_down:.1f}° > {MAX_PITCH_DOWN}°)"
    
    if velocity is not None:
        airspeed = math.hypot(velocity.north_m_s, velocity.east_m_s)
        if airspeed > MAX_AIRSPEED:
            return False, f"Airspeed too high ({airspeed:.1f} m/s > {MAX_AIRSPEED} m/s)"
        if airspeed < MIN_AIRSPEED and altitude > MIN_SAFE_ALTITUDE + 10:
            return False, f"Airspeed too low - STALL RISK ({airspeed:.1f} m/s < {MIN_AIRSPEED} m/s)"
    
    return True, "OK"


def log_telemetry(phase, altitude, velocity, attitude, extra=None):
    """Add telemetry data to mission log"""
    global mission_log
    
    entry = {
        "timestamp": datetime.now().isoformat(),
        "phase": phase,
        "altitude_m": round(altitude, 2),
        "vertical_speed_m_s": round(velocity.down_m_s, 2) if velocity else 0.0,
        "airspeed_m_s": round(math.hypot(velocity.north_m_s, velocity.east_m_s), 2) if velocity else 0.0,
        "total_speed_m_s": round(math.sqrt(velocity.north_m_s**2 + velocity.east_m_s**2 + velocity.down_m_s**2), 2) if velocity else 0.0,
        "roll_deg": round(attitude.roll_deg, 2) if attitude else 0.0,
        "pitch_deg": round(attitude.pitch_deg, 2) if attitude else 0.0,
        "yaw_deg": round(attitude.yaw_deg, 2) if attitude else 0.0,
        "loiter_time_s": 0.0,
        "target_distance_m": 0.0,
        "dive_distance_m": 0.0,
        "dive_time_s": 0.0,
    }
    
    if extra:
        entry.update(extra)
    
    mission_log.append(entry)


# ================= COORDINATE CONVERSION =================
def get_current_ned_position():
    """
    Get current position in NED coordinates relative to home
    For simplicity, we use basic approximation
    """
    if latest_pos is None or home_position is None:
        return 0.0, 0.0
    
    # Simple lat/lon to meters conversion (approximate)
    # 1 degree latitude ≈ 111,320 meters
    # 1 degree longitude ≈ 111,320 * cos(latitude) meters
    
    lat_diff = latest_pos.latitude_deg - home_position['lat']
    lon_diff = latest_pos.longitude_deg - home_position['lon']
    
    north = lat_diff * 111320.0
    east = lon_diff * 111320.0 * math.cos(math.radians(home_position['lat']))
    
    return north, east


# ================= MISSION PHASES =================
async def phase_takeoff_and_climb(drone):
    """
    Phase 1: Takeoff and High-Altitude Climb
    Climb to 200m altitude
    """
    global home_position
    
    print("\n" + "="*50)
    print("🚀 PHASE 1: TAKEOFF & HIGH-ALTITUDE CLIMB")
    print("="*50)
    print(f"Target: {TARGET_ALTITUDE}m AGL")
    
    # Store home position for NED calculations
    if latest_pos:
        home_position = {
            'lat': latest_pos.latitude_deg,
            'lon': latest_pos.longitude_deg,
            'alt': latest_pos.absolute_altitude_m
        }
        print(f"📍 Home position: ({home_position['lat']:.6f}, {home_position['lon']:.6f})")
    
    # Use PX4 built-in takeoff for fixed-wing
    print("Executing fixed-wing takeoff...")
    await drone.action.set_takeoff_altitude(TARGET_ALTITUDE)
    await drone.action.takeoff()
    
    # Monitor climb
    while True:
        if latest_pos is None:
            await asyncio.sleep(0.1)
            continue
            
        alt = latest_pos.relative_altitude_m
        
        # Log telemetry
        if latest_vel and latest_attitude:
            log_telemetry("CLIMB", alt, latest_vel, latest_attitude)
        
        # Display progress
        climb_rate = -latest_vel.down_m_s if latest_vel else 0.0
        airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s) if latest_vel else 0.0
        print(f"📈 ALT {alt:5.1f} m | Climb {climb_rate:5.1f} m/s | Airspeed {airspeed:5.1f} m/s", end="\r")
        
        # Exit condition: reached target altitude
        if alt >= TARGET_ALTITUDE - 10.0:
            print(f"\n✅ Climb complete: {alt:.1f}m")
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    return True


async def phase_target_identification(drone):
    """
    Phase 2: Target Identification (Simulated)
    In real implementation: camera-based target detection
    Current: uses predefined coordinates
    """
    print("\n" + "="*50)
    print("🎯 PHASE 2: TARGET IDENTIFICATION")
    print("="*50)
    print(f"Maintaining altitude for {LOITER_DURATION} seconds at {TARGET_ALTITUDE}m")
    print("🔍 Simulating camera-based target detection...")
    
    # Start offboard mode to maintain altitude precisely
    print("\n🎮 Starting OFFBOARD mode for altitude hold...")
    
    # Command level flight to maintain altitude
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(CRUISE_SPEED, 0.0, 0.0, 0.0)
    )
    
    try:
        await drone.offboard.start()
        print("✅ OFFBOARD altitude hold active")
    except OffboardError as e:
        print(f"⚠️  Offboard failed: {e}, using HOLD instead")
        await drone.action.hold()
    
    start_time = asyncio.get_event_loop().time()
    
    # Simulate target detection during loiter
    while True:
        elapsed = asyncio.get_event_loop().time() - start_time
        alt = latest_pos.relative_altitude_m if latest_pos else 0.0
        
        # Get current position
        curr_north, curr_east = get_current_ned_position()
        
        # Log telemetry
        if latest_vel and latest_attitude:
            log_telemetry("TARGET_ID", alt, latest_vel, latest_attitude, 
                         {"loiter_time_s": round(elapsed, 2)})
        
        # Display progress
        remaining = LOITER_DURATION - elapsed
        airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s) if latest_vel else 0.0
        print(f"🔍 Scanning... {remaining:.1f}s | ALT {alt:.1f}m | Pos ({curr_north:.1f}m N, {curr_east:.1f}m E)", end="\r")
        
        # Continuously command altitude hold
        try:
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(CRUISE_SPEED, 0.0, 0.0, 0.0)
            )
        except:
            pass
        
        # Exit condition
        if elapsed >= LOITER_DURATION:
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    # "Detected" target at configured location
    print(f"\n\n🎯 TARGET DETECTED!")
    print(f"   Location: ({TARGET_LOCATION_NORTH:.1f}m N, {TARGET_LOCATION_EAST:.1f}m E)")
    print(f"   Type: SIMULATED (will be camera detection in production)")
    
    # Get current position
    curr_north, curr_east = get_current_ned_position()
    current_alt = latest_pos.relative_altitude_m
    
    # Calculate vector from current position to target
    delta_north = TARGET_LOCATION_NORTH - curr_north
    delta_east = TARGET_LOCATION_EAST - curr_east
    horizontal_dist = math.hypot(delta_north, delta_east)
    
    # Calculate heading to target
    target_heading = math.degrees(math.atan2(delta_east, delta_north))
    
    # Calculate dive angle based on current altitude and horizontal distance
    dive_angle = math.degrees(math.atan(current_alt / horizontal_dist))
    
    # Calculate required speeds to hit target
    # We want to maintain aggressive dive parameters
    glide_speed = min(MAX_GLIDE_SPEED, horizontal_dist / (current_alt / MAX_DIVE_SPEED))
    descent_speed = min(MAX_DIVE_SPEED, current_alt / (horizontal_dist / glide_speed))
    
    print(f"\n📊 Dive Vector Calculated:")
    print(f"   Current Position: ({curr_north:.1f}m N, {curr_east:.1f}m E, {current_alt:.1f}m AGL)")
    print(f"   Target Position: ({TARGET_LOCATION_NORTH:.1f}m N, {TARGET_LOCATION_EAST:.1f}m E, 0m)")
    print(f"   Horizontal Distance: {horizontal_dist:.1f}m")
    print(f"   Target Heading: {target_heading:.1f}°")
    print(f"   Dive Angle: {dive_angle:.1f}°")
    print(f"   Glide Speed: {glide_speed:.1f} m/s")
    print(f"   Descent Speed: {descent_speed:.1f} m/s")
    
    return {
        'target_north': TARGET_LOCATION_NORTH,
        'target_east': TARGET_LOCATION_EAST,
        'heading': target_heading,
        'dive_angle': dive_angle,
        'glide_speed': glide_speed,
        'descent_speed': descent_speed,
        'horizontal_dist': horizontal_dist
    }


async def phase_targeted_dive(drone, dive_params):
    """
    Phase 3: Targeted Dive Attack
    Dive directly toward specific target coordinates
    """
    print("\n" + "="*50)
    print("⚡ PHASE 3: TARGETED DIVE ATTACK")
    print("="*50)
    print(f"Executing precision dive to target:")
    print(f"  Target: ({dive_params['target_north']:.1f}m N, {dive_params['target_east']:.1f}m E)")
    print(f"  Heading: {dive_params['heading']:.1f}°")
    print(f"  Dive Angle: {dive_params['dive_angle']:.1f}°")
    print(f"  Glide Speed: {dive_params['glide_speed']:.1f} m/s")
    print(f"  Descent Speed: {dive_params['descent_speed']:.1f} m/s")
    
    # Stop previous offboard mode
    try:
        await drone.offboard.stop()
        await asyncio.sleep(0.5)
    except:
        pass
    
    # Start fresh offboard mode with attitude control
    print("\n🎮 Starting OFFBOARD mode for targeted dive...")
    
    await drone.offboard.set_attitude(
        Attitude(
            roll_deg=0.0,
            pitch_deg=-dive_params['dive_angle'],
            yaw_deg=dive_params['heading'],
            thrust_value=0.7
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
    
    while True:
        alt = latest_pos.relative_altitude_m
        
        # Safety check
        is_safe, reason = check_safety_abort(alt, latest_attitude, latest_vel)
        if not is_safe:
            print(f"\n⚠️  SAFETY ABORT: {reason}")
            break
        
        # Calculate current position and distance to target
        curr_north, curr_east = get_current_ned_position()
        dist_to_target = math.hypot(
            dive_params['target_north'] - curr_north,
            dive_params['target_east'] - curr_east
        )
        
        # Calculate dive metrics
        dive_distance = dive_start_alt - alt
        dive_time = asyncio.get_event_loop().time() - dive_start_time
        
        # Log telemetry
        if latest_vel and latest_attitude:
            airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s)
            total_speed = math.sqrt(latest_vel.north_m_s**2 + latest_vel.east_m_s**2 + latest_vel.down_m_s**2)
            
            log_telemetry("DIVE", alt, latest_vel, latest_attitude, {
                "target_distance_m": round(dist_to_target, 2),
                "dive_distance_m": round(dive_distance, 2),
                "dive_time_s": round(dive_time, 2),
            })
            
            # Display progress
            pitch = latest_attitude.pitch_deg
            v_down = latest_vel.down_m_s
            
            print(f"⬇️  ALT {alt:5.1f}m | Dist {dist_to_target:5.1f}m | Vz {v_down:5.1f} m/s | Airspeed {airspeed:5.1f} m/s | Pitch {pitch:5.1f}°", end="\r")
        
        # Exit condition: reached pull-up altitude
        if alt <= MIN_SAFE_ALTITUDE:
            print(f"\n✅ Pull-up altitude reached: {alt:.1f}m")
            break
        
        # Dynamically adjust dive attitude to track target
        # Recalculate heading to target
        delta_north = dive_params['target_north'] - curr_north
        delta_east = dive_params['target_east'] - curr_east
        current_heading = math.degrees(math.atan2(delta_east, delta_north))
        
        # Command dive attitude with updated heading
        try:
            await drone.offboard.set_attitude(
                Attitude(
                    roll_deg=0.0,
                    pitch_deg=-dive_params['dive_angle'],
                    yaw_deg=current_heading,
                    thrust_value=0.7
                )
            )
        except:
            pass
        
        await asyncio.sleep(CONTROL_RATE)
    
    print(f"\n📊 Dive Statistics:")
    print(f"   Distance descended: {dive_distance:.1f}m")
    print(f"   Time in dive: {dive_time:.1f}s")
    if dive_time > 0:
        print(f"   Average descent rate: {dive_distance/dive_time:.1f} m/s")
    print(f"   Final distance to target: {dist_to_target:.1f}m")
    
    return True


async def phase_recovery(drone):
    """
    Phase 4: Pull-up and Recovery
    Transition to level flight
    """
    print("\n" + "="*50)
    print("🛬 PHASE 4: PULL-UP & RECOVERY")
    print("="*50)
    
    print("Executing pull-up maneuver...")
    
    try:
        # Level flight at cruise speed
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(CRUISE_SPEED, 0.0, 0.0, 0.0)
        )
        
        # Maintain level flight for a few seconds
        for i in range(30):  # 3 seconds
            alt = latest_pos.relative_altitude_m if latest_pos else 0.0
            
            if latest_vel and latest_attitude:
                log_telemetry("RECOVERY", alt, latest_vel, latest_attitude)
            
            pitch = latest_attitude.pitch_deg if latest_attitude else 0.0
            airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s) if latest_vel else 0.0
            print(f"📊 ALT {alt:5.1f}m | Pitch {pitch:5.1f}° | Airspeed {airspeed:5.1f} m/s", end="\r")
            
            await asyncio.sleep(CONTROL_RATE)
        
        # Stop offboard and return to hold
        await drone.offboard.stop()
        await drone.action.hold()
        print("\n✅ Returned to HOLD mode")
        
    except Exception as e:
        print(f"\n⚠️  Recovery error: {e}")
        try:
            await drone.offboard.stop()
        except:
            pass
    
    print(f"✅ Recovery complete")
    return True


# ================= MAIN =================
async def run():
    """Main mission execution"""
    global latest_pos, latest_vel, latest_attitude, stop_tasks
    
    print("\n" + "="*60)
    print("🎯 PX4 TARGETED LOCATION DIVE MISSION")
    print("="*60)
    print(f"Configuration:")
    print(f"  Target Altitude: {TARGET_ALTITUDE}m")
    print(f"  Target Location: ({TARGET_LOCATION_NORTH}m N, {TARGET_LOCATION_EAST}m E)")
    print(f"  Min Safe Altitude: {MIN_SAFE_ALTITUDE}m")
    print(f"  Max Dive Speed: {MAX_DIVE_SPEED} m/s")
    print(f"  Max Glide Speed: {MAX_GLIDE_SPEED} m/s")
    print("="*60)
    
    # Connect to PX4
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    print("\n⏳ Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected to PX4 SITL")
            break
    
    # Wait for armable
    await wait_until_armable(drone)
    
    # Configure parameters
    original_params = await configure_aggressive_params(drone)
    
    # Start telemetry listeners
    print("\n📡 Starting telemetry listeners...")
    pos_task = asyncio.create_task(position_listener(drone))
    vel_task = asyncio.create_task(velocity_listener(drone))
    att_task = asyncio.create_task(attitude_listener(drone))
    
    # Wait for telemetry to be available
    while latest_pos is None or latest_vel is None or latest_attitude is None:
        await asyncio.sleep(0.1)
    
    print("✅ Telemetry online")
    
    # Arm
    print("\n💪 Arming...")
    await drone.action.arm()
    print("✅ Armed")
    
    # Execute mission phases
    try:
        # Phase 1: Takeoff and Climb
        if not await phase_takeoff_and_climb(drone):
            print("❌ Takeoff/Climb phase failed")
            return
        
        # Phase 2: Target Identification
        dive_params = await phase_target_identification(drone)
        
        # Phase 3: Targeted Dive
        if not await phase_targeted_dive(drone, dive_params):
            print("❌ Dive phase failed")
            return
        
        # Phase 4: Recovery
        await phase_recovery(drone)
        
        print("\n" + "="*60)
        print("✅ MISSION COMPLETE")
        print("="*60)
        
    except Exception as e:
        print(f"\n❌ Mission error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Clean shutdown
        print("\n🔄 Shutting down...")
        
        # Stop offboard if active
        try:
            await drone.offboard.stop()
        except:
            pass
        
        # Return to hold
        try:
            await drone.action.hold()
        except:
            pass
        
        # Stop telemetry tasks
        stop_tasks = True
        await asyncio.sleep(0.5)
        pos_task.cancel()
        vel_task.cancel()
        att_task.cancel()
        
        # Restore parameters
        await restore_params(drone, original_params)
        
        # Save log
        if mission_log:
            filename = f"../logs/mission_dive_targeted_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            with open(filename, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=mission_log[0].keys())
                writer.writeheader()
                writer.writerows(mission_log)
            print(f"📊 Log saved: {filename}")
        
        print("✅ Shutdown complete")


if __name__ == "__main__":
    asyncio.run(run())
