#!/usr/bin/env python3
"""
PX4 Fixed-Wing Advanced Dive Mission
=====================================
High-altitude dive attack with optimized gliding descent

Mission Profile (Based on Trajectory Image):
1. Takeoff and climb to 200m altitude
2. Loiter/orbit for target acquisition (5 seconds)
3. Calculate ground target for optimized glide path
4. Execute high-speed dive:
   - Glide speed: 90 m/s (horizontal component)
   - Descent speed: 45 m/s (vertical component)
   - Optimized gliding angle calculated from speeds
5. Pull up at safe altitude and recover

Safety: Parameter backup/restore, continuous health monitoring
"""

import asyncio
import csv
import math
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, Attitude

# ================= CONFIGURATION (Based on Image) =================
TARGET_ALTITUDE = 200.0       # meters AGL (from image)
LOITER_DURATION = 5.0         # seconds (target acquisition)
GLIDE_SPEED = 90.0            # m/s (horizontal speed during dive)
DESCENT_SPEED = 45.0          # m/s (vertical speed during dive)
MIN_SAFE_ALTITUDE = 15.0      # pull-up altitude (higher for high-speed dive)
CRUISE_SPEED = 20.0           # m/s (normal flight speed)
DIVE_HEADING_DEG = 270.0      # degrees (heading for dive)
CONTROL_RATE = 0.1            # 10Hz control loop

# Calculate dive angle from speeds (arctan(descent/glide))
DIVE_ANGLE_DEG = math.degrees(math.atan(DESCENT_SPEED / GLIDE_SPEED))

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
        "FW_T_SINK_MAX": 50.0,      # Max sink rate (allow 45 m/s descent)
        "FW_P_LIM_MIN": -70.0,       # Max pitch down (allow steep dive)
        "FW_AIRSPD_MAX": 100.0,      # Max airspeed (allow 90+ m/s glide)
        "FW_AIRSPD_MIN": 8.0,        # Min airspeed (stall prevention)
        "FW_T_CLMB_MAX": 8.0,        # Max climb rate for initial ascent
    }
    
    original_params = {}
    
    for param_name, new_value in params_to_set.items():
        try:
            # Get original value
            original = await drone.param.get_param_float(param_name)
            original_params[param_name] = original
            
            # Set new value
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
        pitch_down = -attitude.pitch_deg  # Negative pitch = nose down
        
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


# ================= MISSION PHASES =================
async def phase_takeoff_and_climb(drone):
    """
    Phase 1: Takeoff and High-Altitude Climb
    Climb to 200m altitude
    """
    print("\n" + "="*50)
    print("🚀 PHASE 1: TAKEOFF & HIGH-ALTITUDE CLIMB")
    print("="*50)
    print(f"Target: {TARGET_ALTITUDE}m AGL")
    
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
        if alt >= TARGET_ALTITUDE - 10.0:  # 10m tolerance for fixed-wing
            print(f"\n✅ Climb complete: {alt:.1f}m")
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    return True


async def phase_loiter_and_target(drone):
    """
    Phase 2: Loiter & Target Acquisition
    Orbit at target altitude and calculate dive parameters
    """
    print("\n" + "="*50)
    print("🎯 PHASE 2: LOITER & TARGET ACQUISITION")
    print("="*50)
    print(f"Maintaining altitude for {LOITER_DURATION} seconds at {TARGET_ALTITUDE}m")
    
    # Get current position for altitude hold
    current_alt = latest_pos.relative_altitude_m if latest_pos else TARGET_ALTITUDE
    
    # Start offboard mode to maintain altitude precisely
    print("🎮 Starting OFFBOARD mode for altitude hold...")
    
    # Command level flight at current altitude (maintain altitude during loiter)
    heading_rad = math.radians(DIVE_HEADING_DEG)
    level_north = CRUISE_SPEED * math.cos(heading_rad)
    level_east = CRUISE_SPEED * math.sin(heading_rad)
    
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(level_north, level_east, 0.0, DIVE_HEADING_DEG)
    )
    
    try:
        await drone.offboard.start()
        print("✅ OFFBOARD altitude hold active")
    except OffboardError as e:
        print(f"⚠️  Offboard failed: {e}, using HOLD instead")
        await drone.action.hold()
    
    start_time = asyncio.get_event_loop().time()
    
    while True:
        elapsed = asyncio.get_event_loop().time() - start_time
        alt = latest_pos.relative_altitude_m if latest_pos else 0.0
        
        # Log telemetry
        if latest_vel and latest_attitude:
            log_telemetry("LOITER", alt, latest_vel, latest_attitude, 
                         {"loiter_time_s": round(elapsed, 2)})
        
        # Display progress
        remaining = LOITER_DURATION - elapsed
        airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s) if latest_vel else 0.0
        print(f"⏱️  Altitude Hold... {remaining:.1f}s | ALT {alt:.1f}m | Airspeed {airspeed:.1f} m/s", end="\r")
        
        # Continuously command altitude hold (0 vertical speed)
        try:
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(level_north, level_east, 0.0, DIVE_HEADING_DEG)
            )
        except:
            pass
        
        # Exit condition
        if elapsed >= LOITER_DURATION:
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    # Calculate target position based on optimized glide path
    current_alt = latest_pos.relative_altitude_m
    angle_rad = math.radians(DIVE_ANGLE_DEG)
    horizontal_dist = current_alt / math.tan(angle_rad)
    
    # Use configured dive heading
    heading_rad = math.radians(DIVE_HEADING_DEG)
    
    # Calculate target coordinates
    target_north = horizontal_dist * math.cos(heading_rad)
    target_east = horizontal_dist * math.sin(heading_rad)
    
    print(f"\n🎯 Dive Parameters Calculated:")
    print(f"   Dive Angle: {DIVE_ANGLE_DEG:.1f}° (optimized gliding angle)")
    print(f"   Horizontal Distance: {horizontal_dist:.1f}m")
    print(f"   Glide Speed: {GLIDE_SPEED:.1f} m/s")
    print(f"   Descent Speed: {DESCENT_SPEED:.1f} m/s")
    print(f"   Total Speed: {math.sqrt(GLIDE_SPEED**2 + DESCENT_SPEED**2):.1f} m/s")
    print(f"   Dive Heading: {DIVE_HEADING_DEG:.1f}°")
    
    return target_north, target_east


async def phase_high_speed_dive(drone, target_north, target_east):
    """
    Phase 3: High-Speed Optimized Dive
    Execute dive at 90 m/s glide speed with 45 m/s descent
    """
    print("\n" + "="*50)
    print("⚡ PHASE 3: HIGH-SPEED DIVE ATTACK")
    print("="*50)
    print(f"Executing optimized dive:")
    print(f"  Glide Speed: {GLIDE_SPEED} m/s (horizontal)")
    print(f"  Descent Speed: {DESCENT_SPEED} m/s (vertical)")
    print(f"  Dive Angle: {DIVE_ANGLE_DEG:.1f}°")
    
    # Stop previous offboard mode from loiter
    try:
        await drone.offboard.stop()
        await asyncio.sleep(0.5)
    except:
        pass
    
    # Start fresh offboard mode with attitude control for dive
    print("\n🎮 Starting OFFBOARD mode for dive (attitude control)...")
    
    # Use attitude control - more reliable for fixed-wing steep dives
    await drone.offboard.set_attitude(
        Attitude(
            roll_deg=0.0,
            pitch_deg=-DIVE_ANGLE_DEG,
            yaw_deg=DIVE_HEADING_DEG,
            thrust_value=0.7  # High thrust for dive
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
        
        # Calculate dive metrics
        dive_distance = dive_start_alt - alt
        dive_time = asyncio.get_event_loop().time() - dive_start_time
        
        # Calculate distance to target
        dist_to_target = math.hypot(target_north, target_east) if target_north else 0.0
        
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
            yaw = latest_attitude.yaw_deg
            v_down = latest_vel.down_m_s
            
            print(f"⬇️  ALT {alt:5.1f}m | Vz {v_down:5.1f} m/s | Airspeed {airspeed:5.1f} m/s | Total {total_speed:5.1f} m/s | Pitch {pitch:5.1f}°", end="\r")
        
        # Exit condition: reached pull-up altitude
        if alt <= MIN_SAFE_ALTITUDE:
            print(f"\n✅ Pull-up altitude reached: {alt:.1f}m")
            break
        
        # Command dive attitude (continuous update for fixed-wing stability)
        try:
            await drone.offboard.set_attitude(
                Attitude(
                    roll_deg=0.0,
                    pitch_deg=-DIVE_ANGLE_DEG,
                    yaw_deg=DIVE_HEADING_DEG,
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
    
    return True



async def phase_recovery(drone):
    """
    Phase 4: Pull-up and Recovery
    Transition to level flight after high-speed dive
    """
    print("\n" + "="*50)
    print("🛬 PHASE 4: PULL-UP & RECOVERY")
    print("="*50)
    
    # Command level flight velocity first (gradual pull-up)
    print("Executing pull-up maneuver...")
    
    try:
        # Level flight at cruise speed
        heading_rad = math.radians(DIVE_HEADING_DEG)
        level_north = CRUISE_SPEED * math.cos(heading_rad)
        level_east = CRUISE_SPEED * math.sin(heading_rad)
        
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(level_north, level_east, 0.0, DIVE_HEADING_DEG)
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
        # Emergency: just stop offboard
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
    print("✈️  PX4 ADVANCED FIXED-WING DIVE MISSION")
    print("="*60)
    print(f"Configuration:")
    print(f"  Target Altitude: {TARGET_ALTITUDE}m")
    print(f"  Glide Speed: {GLIDE_SPEED} m/s")
    print(f"  Descent Speed: {DESCENT_SPEED} m/s")
    print(f"  Dive Angle: {DIVE_ANGLE_DEG:.1f}° (optimized)")
    print(f"  Min Safe Altitude: {MIN_SAFE_ALTITUDE}m")
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
        
        # Phase 2: Loiter & Target
        target_north, target_east = await phase_loiter_and_target(drone)
        
        # Phase 3: High-Speed Dive
        if not await phase_high_speed_dive(drone, target_north, target_east):
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
            filename = f"../logs/mission_dive_advanced_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            with open(filename, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=mission_log[0].keys())
                writer.writeheader()
                writer.writerows(mission_log)
            print(f"📊 Log saved: {filename}")
        
        print("✅ Shutdown complete")


if __name__ == "__main__":
    asyncio.run(run())
