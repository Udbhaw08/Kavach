#!/usr/bin/env python3
"""
PX4 Fixed-Wing Autonomous Dive Mission
========================================
Adapted for plane_cam model - Fixed-wing dive attack simulation

Mission Profile (Fixed-Wing Adaptation):
1. Takeoff and climb to 70m altitude
2. Loiter/orbit for 5 seconds at 70m
3. Calculate ground target at 35° angle
4. Execute dive toward target at 5 m/s descent
5. Pull up at 5m and level flight recovery

Note: Fixed-wing aircraft cannot hover - they maintain forward airspeed
"""

import asyncio
import csv
import math
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw

# ================= FIXED-WING CONFIGURATION =================
TARGET_ALTITUDE = 70.0        # meters AGL
LOITER_RADIUS = 50.0          # meters (orbit radius)
LOITER_DURATION = 5.0         # seconds (cannot truly "hover")
DIVE_ANGLE_DEG = 35.0         # degrees from horizontal
DESCENT_SPEED = 5.0           # m/s (downward velocity)
CRUISE_SPEED = 15.0           # m/s (typical fixed-wing cruise)
MIN_SAFE_ALTITUDE = 5.0       # pull-up altitude
MAX_PITCH_DOWN = 45.0         # degrees (dive pitch limit)
CONTROL_RATE = 0.1            # 10Hz control loop (fixed-wing slower)

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


# ================= SAFETY & HEALTH =================
async def wait_until_armable(drone):
    """Wait for vehicle to be ready for arming"""
    print("⏳ Waiting for vehicle to become armable...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("✅ Vehicle is armable")
            return
        await asyncio.sleep(0.2)


def check_safety_abort(altitude, attitude):
    """
    Check safety conditions for fixed-wing
    Returns: (is_safe: bool, reason: str)
    """
    if altitude < MIN_SAFE_ALTITUDE:
        return False, f"Altitude below minimum ({altitude:.1f}m < {MIN_SAFE_ALTITUDE}m)"
    
    if attitude is not None:
        pitch_down = -attitude.pitch_deg  # Negative pitch = nose down
        
        if pitch_down > MAX_PITCH_DOWN:
            return False, f"Dive too steep ({pitch_down:.1f}° > {MAX_PITCH_DOWN}°)"
    
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
        "roll_deg": round(attitude.roll_deg, 2) if attitude else 0.0,
        "pitch_deg": round(attitude.pitch_deg, 2) if attitude else 0.0,
        "yaw_deg": round(attitude.yaw_deg, 2) if attitude else 0.0,
        "loiter_time_s": 0.0,
        "target_distance_m": 0.0,
    }
    
    if extra:
        entry.update(extra)
    
    mission_log.append(entry)


# ================= MISSION PHASES (FIXED-WING) =================
async def phase_takeoff_and_climb(drone):
    """
    Phase 1: Takeoff and Climb
    Fixed-wing takeoff to 70m altitude
    """
    print("\n" + "="*50)
    print("🛫 PHASE 1: TAKEOFF & CLIMB")
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
        print(f"📈 ALT {alt:5.1f} m | Climb Rate {climb_rate:5.1f} m/s", end="\r")
        
        # Exit condition: reached target altitude
        if alt >= TARGET_ALTITUDE - 5.0:  # 5m tolerance for fixed-wing
            print(f"\n✅ Climb complete: {alt:.1f}m")
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    return True


async def phase_loiter(drone):
    """
    Phase 2: Loiter (Fixed-wing "hover")
    Orbit at target altitude for specified duration
    """
    print("\n" + "="*50)
    print("🔄 PHASE 2: LOITER (ORBIT)")
    print("="*50)
    print(f"Orbiting for {LOITER_DURATION} seconds at {TARGET_ALTITUDE}m")
    
    # Get current position for loiter center
    if latest_pos is None:
        print("⚠️  No position data, skipping loiter")
        return None, None
    
    center_lat = latest_pos.latitude_deg
    center_lon = latest_pos.longitude_deg
    
    # Command loiter/hold
    print(f"Loitering at ({center_lat:.6f}, {center_lon:.6f})")
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
        print(f"⏱️  Loitering... {remaining:.1f}s | ALT {alt:.1f}m | Airspeed {airspeed:.1f} m/s", end="\r")
        
        # Exit condition
        if elapsed >= LOITER_DURATION:
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    # Calculate target position at 35° angle
    current_alt = latest_pos.relative_altitude_m
    angle_rad = math.radians(DIVE_ANGLE_DEG)
    horizontal_dist = current_alt / math.tan(angle_rad)
    
    # Get current heading to place target ahead
    heading_rad = math.radians(latest_attitude.yaw_deg) if latest_attitude else 0.0
    
    # Calculate target coordinates (simplified - assumes flat earth)
    # In real implementation, use proper geodetic calculations
    target_north = horizontal_dist * math.cos(heading_rad)
    target_east = horizontal_dist * math.sin(heading_rad)
    
    print(f"\n🎯 Target calculated:")
    print(f"   Angle: {DIVE_ANGLE_DEG}° from horizontal")
    print(f"   Distance: {horizontal_dist:.1f}m")
    print(f"   Heading: {latest_attitude.yaw_deg:.1f}°")
    
    return target_north, target_east


async def phase_dive(drone, target_north, target_east):
    """
    Phase 3: Dive Attack
    Execute controlled dive toward target at 35° angle
    """
    print("\n" + "="*50)
    print("⚡ PHASE 3: DIVE ATTACK")
    print("="*50)
    print(f"Diving at {DESCENT_SPEED} m/s descent, {DIVE_ANGLE_DEG}° angle")
    
    # Start offboard mode for dive control
    print("🎮 Starting OFFBOARD mode for dive...")
    
    # Calculate dive velocity components
    angle_rad = math.radians(DIVE_ANGLE_DEG)
    forward_speed = DESCENT_SPEED / math.tan(angle_rad)
    
    print(f"   Descent: {DESCENT_SPEED:.1f} m/s")
    print(f"   Forward: {forward_speed:.1f} m/s")
    
    # Initialize offboard with current velocity
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(forward_speed, 0.0, DESCENT_SPEED, 0.0)
    )
    
    try:
        await drone.offboard.start()
        print("✅ OFFBOARD mode active")
    except OffboardError as e:
        print(f"⚠️  Offboard failed: {e}")
        print("   Continuing with mission mode...")
        # Fixed-wing can continue in mission mode if offboard fails
    
    while True:
        alt = latest_pos.relative_altitude_m
        
        # Safety check
        is_safe, reason = check_safety_abort(alt, latest_attitude)
        if not is_safe:
            print(f"\n⚠️  SAFETY ABORT: {reason}")
            break
        
        # Calculate distance to target (simplified)
        dist_to_target = math.hypot(target_north, target_east) if target_north else 0.0
        
        # Log telemetry
        if latest_vel and latest_attitude:
            log_telemetry("DIVE", alt, latest_vel, latest_attitude,
                         {"target_distance_m": round(dist_to_target, 2)})
        
        # Display progress
        pitch = latest_attitude.pitch_deg if latest_attitude else 0.0
        v_down = latest_vel.down_m_s if latest_vel else 0.0
        print(f"⬇️  ALT {alt:5.1f} m | Vz {v_down:5.1f} m/s | Pitch {pitch:5.1f}°", end="\r")
        
        # Exit condition: reached abort altitude
        if alt <= MIN_SAFE_ALTITUDE:
            print(f"\n✅ Pull-up altitude reached: {alt:.1f}m")
            break
        
        # Command dive velocity (fixed-wing will pitch down naturally)
        try:
            heading_deg = latest_attitude.yaw_deg if latest_attitude else 0.0
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(
                    north_m_s=forward_speed,
                    east_m_s=0.0,
                    down_m_s=DESCENT_SPEED,
                    yaw_deg=heading_deg
                )
            )
        except:
            # If offboard fails, continue monitoring
            pass
        
        await asyncio.sleep(CONTROL_RATE)
    
    # Stop dive - command level flight velocity
    try:
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(CRUISE_SPEED, 0.0, 0.0, 0.0)
        )
    except:
        pass
    
    return True


async def phase_recovery(drone):
    """
    Phase 4: Pull-up and Recovery
    Transition to level flight
    """
    print("\n" + "="*50)
    print("🛬 PHASE 4: PULL-UP & RECOVERY")
    print("="*50)
    
    # Command level flight
    print("Executing pull-up maneuver...")
    
    try:
        # Stop offboard and return to hold/loiter
        await drone.action.hold()
        print("✅ Returned to HOLD mode")
    except Exception as e:
        print(f"⚠️  Hold failed: {e}")
    
    # Monitor recovery for a few seconds
    for i in range(20):  # 2 seconds
        alt = latest_pos.relative_altitude_m if latest_pos else 0.0
        
        if latest_vel and latest_attitude:
            log_telemetry("RECOVERY", alt, latest_vel, latest_attitude)
        
        pitch = latest_attitude.pitch_deg if latest_attitude else 0.0
        print(f"📊 ALT {alt:5.1f} m | Pitch {pitch:5.1f}°", end="\r")
        
        await asyncio.sleep(CONTROL_RATE)
    
    print(f"\n✅ Recovery complete")
    return True


# ================= MAIN =================
async def run():
    """Main mission execution"""
    global latest_pos, latest_vel, latest_attitude, stop_tasks
    
    print("\n" + "="*60)
    print("✈️  PX4 FIXED-WING AUTONOMOUS DIVE MISSION")
    print("="*60)
    print("Model: plane_cam (Fixed-Wing)")
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
        
        # Phase 2: Loiter
        target_north, target_east = await phase_loiter(drone)
        
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
        
        # Save log
        if mission_log:
            filename = f"../logs/mission_dive_fixedwing_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            with open(filename, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=mission_log[0].keys())
                writer.writeheader()
                writer.writerows(mission_log)
            print(f"📊 Log saved: {filename}")
        
        print("✅ Shutdown complete")


if __name__ == "__main__":
    asyncio.run(run())
