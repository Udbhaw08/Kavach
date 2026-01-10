#!/usr/bin/env python3
"""
PX4 Autonomous Dive Mission
============================
High-speed ascent → Hover → Target acquisition → Controlled dive

Mission Profile:
1. Ascent at 6 m/s to 70m
2. Hover for 5 seconds
3. Calculate ground target at 35° angle
4. Dive at 5 m/s descent toward target
5. Abort at 5m and recover

Safety: Parameter backup/restore, continuous health monitoring
"""

import asyncio
import csv
import math
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

# ================= CONFIGURATION =================
TAKEOFF_ALTITUDE = 70.0       # meters
ASCENT_SPEED = 6.0            # m/s (upward, negative in NED)
HOVER_DURATION = 5.0          # seconds
DIVE_ANGLE_DEG = 35.0         # degrees from horizontal
DESCENT_SPEED = 5.0           # m/s (downward, positive in NED)
MIN_SAFE_ALTITUDE = 5.0       # abort floor
MAX_TILT_ANGLE = 55.0         # degrees (safety limit, allows 35° dive)
CONTROL_RATE = 0.05           # 20Hz control loop (50ms)

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
    """Continuously update global attitude (for safety checks)"""
    global latest_attitude, stop_tasks
    async for attitude in drone.telemetry.attitude_euler():
        if stop_tasks:
            break
        latest_attitude = attitude


# ================= SAFETY & HEALTH =================
async def wait_until_armable(drone):
    """Wait for vehicle to be ready for arming"""
    print("⏳ Waiting for vehicle to become armable...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("✅ Vehicle is armable")
            return
        await asyncio.sleep(0.2)


async def configure_aggressive_params(drone):
    """Set parameters for high-speed flight"""
    print("🔧 Configuring aggressive flight parameters...")
    
    # Backup original values for restoration later
    params_to_set = [
        ("MPC_Z_VEL_MAX_UP", 8.0),    # Allow 6+ m/s ascent
        ("MPC_Z_VEL_MAX_DN", 6.0),    # Allow 5 m/s descent
        ("MPC_TILTMAX_AIR", 50.0),    # Allow dive angle
        ("MPC_XY_VEL_MAX", 10.0),     # Forward speed during dive
        ("MPC_ACC_DOWN_MAX", 6.0),    # Aggressive descent accel
    ]
    
    original_params = {}
    
    for param_name, new_value in params_to_set:
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


def check_safety_abort(altitude, attitude):
    """
    Check safety conditions
    Returns: (is_safe: bool, reason: str)
    """
    if altitude < MIN_SAFE_ALTITUDE:
        return False, f"Altitude below minimum ({altitude:.1f}m < {MIN_SAFE_ALTITUDE}m)"
    
    if attitude is not None:
        pitch = abs(attitude.pitch_deg)
        roll = abs(attitude.roll_deg)
        max_tilt = max(pitch, roll)
        
        if max_tilt > MAX_TILT_ANGLE:
            return False, f"Tilt angle too high ({max_tilt:.1f}° > {MAX_TILT_ANGLE}°)"
    
    return True, "OK"


def log_telemetry(phase, altitude, velocity, attitude, extra=None):
    """Add telemetry data to mission log"""
    global mission_log
    
    entry = {
        "timestamp": datetime.now().isoformat(),
        "phase": phase,
        "altitude_m": round(altitude, 2),
        "vertical_speed_m_s": round(velocity.down_m_s, 2) if velocity else 0.0,
        "horizontal_speed_m_s": round(math.hypot(velocity.north_m_s, velocity.east_m_s), 2) if velocity else 0.0,
        "roll_deg": round(attitude.roll_deg, 2) if attitude else 0.0,
        "pitch_deg": round(attitude.pitch_deg, 2) if attitude else 0.0,
        "yaw_deg": round(attitude.yaw_deg, 2) if attitude else 0.0,
        "hover_time_s": 0.0,        # Default values for optional fields
        "target_distance_m": 0.0,   # Will be overwritten if provided in extra
    }
    
    if extra:
        entry.update(extra)
    
    mission_log.append(entry)


# ================= MISSION PHASES =================
async def phase_ascent(drone):
    """
    Phase 1: High-Speed Ascent
    Climb at 6 m/s to 70m altitude
    """
    print("\n" + "="*50)
    print("🚀 PHASE 1: HIGH-SPEED ASCENT")
    print("="*50)
    print(f"Target: {TAKEOFF_ALTITUDE}m at {ASCENT_SPEED} m/s")
    
    # Command high-speed ascent (negative = up in NED)
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, -ASCENT_SPEED, 0.0)
    )
    
    while True:
        alt = latest_pos.relative_altitude_m
        vel = latest_vel
        
        # Log telemetry
        log_telemetry("ASCENT", alt, vel, latest_attitude)
        
        # Display progress
        print(f"📈 ALT {alt:5.1f} m | Vz {-vel.down_m_s:5.1f} m/s", end="\r")
        
        # Exit condition
        if alt >= TAKEOFF_ALTITUDE:
            print(f"\n✅ Target altitude reached: {alt:.1f}m")
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    # Stop ascent
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    return True


async def phase_hover_and_target(drone):
    """
    Phase 2: Hover & Target Acquisition
    Stabilize for 5 seconds and calculate dive target
    Returns: (target_north, target_east) in meters from current position
    """
    print("\n" + "="*50)
    print("🛰️  PHASE 2: HOVER & TARGET ACQUISITION")
    print("="*50)
    print(f"Stabilizing for {HOVER_DURATION} seconds...")
    
    # Command hover
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    
    start_time = asyncio.get_event_loop().time()
    
    while True:
        elapsed = asyncio.get_event_loop().time() - start_time
        alt = latest_pos.relative_altitude_m
        vel = latest_vel
        
        # Log telemetry
        log_telemetry("HOVER", alt, vel, latest_attitude, {"hover_time_s": round(elapsed, 2)})
        
        # Display progress
        remaining = HOVER_DURATION - elapsed
        print(f"⏱️  Hovering... {remaining:.1f}s remaining | ALT {alt:.1f}m", end="\r")
        
        # Exit condition
        if elapsed >= HOVER_DURATION:
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    # Calculate target position at 35° angle
    # From geometry: horizontal_distance = altitude / tan(angle)
    current_alt = latest_pos.relative_altitude_m
    angle_rad = math.radians(DIVE_ANGLE_DEG)
    horizontal_dist = current_alt / math.tan(angle_rad)
    
    # Place target directly north (yaw = 0°)
    target_north = horizontal_dist
    target_east = 0.0
    
    print(f"\n🎯 Target calculated:")
    print(f"   Angle: {DIVE_ANGLE_DEG}° from horizontal")
    print(f"   Distance: {horizontal_dist:.1f}m north")
    print(f"   Current altitude: {current_alt:.1f}m")
    
    return target_north, target_east


async def phase_dive(drone, target_north, target_east):
    """
    Phase 3: Controlled Dive
    Dive toward target at 5 m/s descent speed
    """
    print("\n" + "="*50)
    print("⚡ PHASE 3: CONTROLLED DIVE")
    print("="*50)
    print(f"Diving at {DESCENT_SPEED} m/s descent, {DIVE_ANGLE_DEG}° angle")
    
    # Calculate velocity components
    # For 35° dive: vertical = 5 m/s, horizontal = 5 / tan(35°) ≈ 7.14 m/s
    angle_rad = math.radians(DIVE_ANGLE_DEG)
    forward_speed = DESCENT_SPEED / math.tan(angle_rad)
    
    print(f"   Vertical: {DESCENT_SPEED:.1f} m/s")
    print(f"   Horizontal: {forward_speed:.1f} m/s")
    
    # Initial distance to target
    start_pos = latest_pos
    
    while True:
        alt = latest_pos.relative_altitude_m
        vel = latest_vel
        
        # Calculate remaining distance to target
        current_north = 0.0  # We don't have absolute position, use relative
        current_east = 0.0
        dist_to_target = math.hypot(target_north - current_north, target_east - current_east)
        
        # Safety check
        is_safe, reason = check_safety_abort(alt, latest_attitude)
        if not is_safe:
            print(f"\n⚠️  SAFETY ABORT: {reason}")
            break
        
        # Log telemetry
        log_telemetry("DIVE", alt, vel, latest_attitude, {
            "target_distance_m": round(dist_to_target, 2)
        })
        
        # Display progress
        print(f"⬇️  ALT {alt:5.1f} m | Vz {vel.down_m_s:5.1f} m/s | Pitch {latest_attitude.pitch_deg:5.1f}°", end="\r")
        
        # Exit condition: reached abort altitude
        if alt <= MIN_SAFE_ALTITUDE:
            print(f"\n✅ Abort altitude reached: {alt:.1f}m")
            break
        
        # Command dive velocity
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(
                north_m_s=forward_speed,
                east_m_s=0.0,
                down_m_s=DESCENT_SPEED,
                yaw_deg=0.0
            )
        )
        
        await asyncio.sleep(CONTROL_RATE)
    
    # Stop dive
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    return True


async def phase_recovery(drone):
    """
    Phase 4: Recovery
    Stabilize and prepare for landing
    """
    print("\n" + "="*50)
    print("🛬 PHASE 4: RECOVERY")
    print("="*50)
    
    # Command hover
    print("Stabilizing...")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(2.0)
    
    # Log final state
    alt = latest_pos.relative_altitude_m
    log_telemetry("RECOVERY", alt, latest_vel, latest_attitude)
    
    print(f"✅ Recovery complete at {alt:.1f}m")
    return True


# ================= MAIN =================
async def run():
    """Main mission execution"""
    global latest_pos, latest_vel, latest_attitude, stop_tasks
    
    print("\n" + "="*60)
    print("🚁 PX4 AUTONOMOUS DIVE MISSION")
    print("="*60)
    
    # Connect to PX4
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    print("⏳ Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected to PX4 SITL")
            break
    
    # Wait for armable
    await wait_until_armable(drone)
    
    # Configure parameters
    original_params = await configure_aggressive_params(drone)
    
    # Arm
    print("\n💪 Arming...")
    await drone.action.arm()
    print("✅ Armed")
    
    # Initial takeoff to establish offboard control
    print(f"\n🛫 Taking off to {10.0}m (initial)...")
    await drone.action.set_takeoff_altitude(10.0)
    await drone.action.takeoff()
    
    # Wait for initial altitude
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= 9.0:
            print(f"✅ Initial takeoff complete: {pos.relative_altitude_m:.1f}m")
            break
    
    # Start telemetry listeners
    print("\n📡 Starting telemetry listeners...")
    pos_task = asyncio.create_task(position_listener(drone))
    vel_task = asyncio.create_task(velocity_listener(drone))
    att_task = asyncio.create_task(attitude_listener(drone))
    
    # Wait for telemetry to be available
    while latest_pos is None or latest_vel is None or latest_attitude is None:
        await asyncio.sleep(0.1)
    
    print("✅ Telemetry online")
    
    # Start offboard mode
    print("\n🎮 Starting OFFBOARD mode...")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    
    try:
        await drone.offboard.start()
        print("✅ OFFBOARD mode active")
    except OffboardError as e:
        print(f"❌ Offboard failed: {e}")
        stop_tasks = True
        return
    
    # Execute mission phases
    try:
        # Phase 1: Ascent
        if not await phase_ascent(drone):
            print("❌ Ascent phase failed")
            return
        
        # Phase 2: Hover & Target
        target_north, target_east = await phase_hover_and_target(drone)
        
        # Phase 3: Dive
        if not await phase_dive(drone, target_north, target_east):
            print("❌ Dive phase failed")
            return
        
        # Phase 4: Recovery
        await phase_recovery(drone)
        
        print("\n" + "="*60)
        print("✅ MISSION COMPLETE")
        print("="*60)
        
    except Exception as e:
        print(f"\n❌ Mission error: {e}")
    
    finally:
        # Clean shutdown
        print("\n🔄 Shutting down...")
        
        # Stop offboard
        try:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
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
            filename = f"mission_dive_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            with open(filename, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=mission_log[0].keys())
                writer.writeheader()
                writer.writerows(mission_log)
            print(f"📊 Log saved: {filename}")
        
        print("✅ Shutdown complete")


if __name__ == "__main__":
    asyncio.run(run())
