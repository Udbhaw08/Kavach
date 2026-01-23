#!/usr/bin/env python3
"""
Simple VTOL Aggressive Dive - NO TARGET TRACKING
================================================
Just climb, fly straight, then dive aggressively

Profile:
1. MC Takeoff → 200m
2. Transition to FW
3. Fly straight for 10 seconds
4. DIVE straight down at 40 m/s
5. Pull up at 50m
"""

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, PositionNedYaw, VelocityBodyYawspeed

# ================= PARAMETERS =================
ATTACK_ALTITUDE = 250.0      # Increased to 250m for more dive time
MIN_SAFE_ALTITUDE = 30.0     # Lower pull-up for more dive distance
ASCENT_SPEED = 15.0          # 15 m/s climb
DESCENT_SPEED = 50.0         # Increased to 50 m/s (will try to achieve 40+)
CRUISE_SPEED = 20.0          # Forward speed
CONTROL_RATE = 0.05          # 20 Hz for faster response

# ================= GLOBAL STATE =================
latest_pos = None
latest_vel = None
latest_attitude = None
stop_tasks = False

# ================= TELEMETRY =================
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

# ================= PARAMETERS =================
async def configure_params(drone):
    print("🔧 Configuring AGGRESSIVE parameters...")
    params = {
        # MC aggressive descent
        "MPC_Z_VEL_MAX_UP": 15.0,       # 15 m/s ascent
        "MPC_Z_VEL_MAX_DN": 50.0,       # 50 m/s descent (aim high)
        "MPC_TKO_SPEED": 15.0,          # Fast takeoff
        
        # MC acceleration limits (increase for faster response)
        "MPC_ACC_DOWN_MAX": 15.0,       # Max downward acceleration
        "MPC_ACC_UP_MAX": 10.0,         # Max upward acceleration
        "MPC_JERK_MAX": 50.0,           # Max jerk (smoothness)
        
        # Disable some safety limits
        "MPC_LAND_SPEED": 50.0,         # Don't limit descent as "landing"
        
        # FW parameters
        "FW_T_SINK_MAX": 50.0,          # Allow 50 m/s sink
        "FW_P_LIM_MIN": -70.0,          # Allow 70° pitch down
        "FW_THR_IDLE": 0.0,             # 0% idle
    }
    original = {}
    for param, value in params.items():
        try:
            orig = await drone.param.get_param_float(param)
            original[param] = orig
            await drone.param.set_param_float(param, value)
            print(f"  ✓ {param}: {orig:.1f} → {value:.1f}")
        except Exception as e:
            print(f"  ⚠️  {param}: {e}")
    await asyncio.sleep(1.0)
    return original

# ================= PHASES =================
async def phase_climb(drone):
    """Climb to 200m"""
    print("\n" + "="*50)
    print("🚀 PHASE 1: CLIMB TO 200m")
    print("="*50)
    
    # Offboard climb
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -ASCENT_SPEED, 0.0))
    
    try:
        await drone.offboard.start()
        print("✅ Climbing...")
    except:
        await drone.action.takeoff()
    
    while True:
        if latest_pos is None or latest_vel is None:
            await asyncio.sleep(0.1)
            continue
        
        alt = latest_pos.relative_altitude_m
        climb_rate = -latest_vel.down_m_s
        
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -ASCENT_SPEED, 0.0))
        
        print(f"📈 ALT: {alt:5.1f}m | Climb: {climb_rate:5.1f} m/s", end="\r")
        
        if alt >= ATTACK_ALTITUDE - 5.0:
            print(f"\n✅ Reached {alt:.1f}m")
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    try:
        await drone.offboard.stop()
    except:
        pass
    await drone.action.hold()
    await asyncio.sleep(2.0)
    return True

async def phase_transition_fw(drone):
    """Transition to Fixed-Wing"""
    print("\n" + "="*50)
    print("🔄 PHASE 2: TRANSITION TO FW")
    print("="*50)
    
    try:
        await drone.action.transition_to_fixedwing()
        print("✅ Transitioning...")
    except Exception as e:
        print(f"⚠️  {e}")
        return False
    
    await asyncio.sleep(8.0)
    
    for i in range(20):
        if latest_vel:
            airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s)
            if airspeed > 15.0:
                print(f"✅ FW active (IAS: {airspeed:.1f} m/s)")
                return True
        await asyncio.sleep(0.5)
    
    print("✅ FW active")
    return True

async def phase_cruise_straight(drone):
    """Fly straight for 10 seconds"""
    print("\n" + "="*50)
    print("✈️  PHASE 3: CRUISING STRAIGHT")
    print("="*50)
    
    # Get initial heading
    heading = latest_attitude.yaw_deg if latest_attitude else 0.0
    
    # Fly straight
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(
            north_m_s=CRUISE_SPEED * math.cos(math.radians(heading)),
            east_m_s=CRUISE_SPEED * math.sin(math.radians(heading)),
            down_m_s=0.0,
            yaw_deg=heading
        )
    )
    
    try:
        await drone.offboard.start()
    except:
        pass
    
    print(f"Flying straight at heading {heading:.1f}°...")
    
    for i in range(100):  # 10 seconds
        if latest_vel:
            airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s)
            alt = latest_pos.relative_altitude_m if latest_pos else 0
            print(f"   ALT: {alt:5.1f}m | IAS: {airspeed:5.1f} m/s | Heading: {heading:5.1f}°", end="\r")
        
        await asyncio.sleep(CONTROL_RATE)
    
    print(f"\n✅ Cruise complete")
    return heading

async def phase_transition_mc(drone):
    """Transition BACK to MC for dive"""
    print("\n" + "="*50)
    print("🔄 PHASE 4: TRANSITION TO MC FOR DIVE")
    print("="*50)
    
    try:
        await drone.offboard.stop()
    except:
        pass
    
    try:
        await drone.action.transition_to_multicopter()
        print("✅ Transitioning to MC...")
    except Exception as e:
        print(f"⚠️  {e}")
        return False
    
    await asyncio.sleep(6.0)
    
    for i in range(15):
        if latest_vel:
            airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s)
            if airspeed < 8.0:
                print(f"✅ MC active (IAS: {airspeed:.1f} m/s)")
                return True
        await asyncio.sleep(0.5)
    
    print("✅ MC active")
    return True

async def phase_dive_fw(drone):
    """AGGRESSIVE DIVE in FW mode - 35° DIVE ANGLE!"""
    print("\n" + "="*50)
    print("⚡ PHASE 4: AGGRESSIVE FW DIVE (35° ANGLE)")
    print("="*50)
    
    # Geometric dive parameters
    DIVE_ANGLE_DEG = 35.0       # Aggressive but achievable
    TARGET_AIRSPEED = 40.0      # m/s total airspeed
    
    # Calculate velocity components from dive angle
    angle_rad = math.radians(DIVE_ANGLE_DEG)
    forward_speed = TARGET_AIRSPEED * math.cos(angle_rad)  # ~32.7 m/s
    down_speed = TARGET_AIRSPEED * math.sin(angle_rad)     # ~22.9 m/s
    
    print(f"Dive angle: {DIVE_ANGLE_DEG}°")
    print(f"Target airspeed: {TARGET_AIRSPEED} m/s")
    print(f"Forward component: {forward_speed:.1f} m/s")
    print(f"Down component: {down_speed:.1f} m/s")
    print(f"Dive distance: {ATTACK_ALTITUDE - MIN_SAFE_ALTITUDE}m")
    
    dive_start_alt = latest_pos.relative_altitude_m
    dive_start_time = asyncio.get_event_loop().time()
    descent_speeds = []
    
    # Get current heading
    heading = latest_attitude.yaw_deg if latest_attitude else 0.0
    
    # Set velocity with geometric dive components
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(
            north_m_s=forward_speed * math.cos(math.radians(heading)),
            east_m_s=forward_speed * math.sin(math.radians(heading)),
            down_m_s=down_speed,  # Descent component from 35° angle
            yaw_deg=heading
        )
    )
    
    try:
        await drone.offboard.start()
        print("✅ Offboard FW dive active")
        print(f"   Commanding {DIVE_ANGLE_DEG}° dive in FW mode")
    except Exception as e:
        print(f"⚠️  Offboard start: {e}")
        pass
    
    # Monitor dive progress
    while True:
        alt = latest_pos.relative_altitude_m if latest_pos else 0
        
        # Update heading dynamically
        heading = latest_attitude.yaw_deg if latest_attitude else heading
        
        # Command: Geometric dive (forward + down at 35° angle)
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(
                north_m_s=forward_speed * math.cos(math.radians(heading)),
                east_m_s=forward_speed * math.sin(math.radians(heading)),
                down_m_s=down_speed,  # 35° dive component
                yaw_deg=heading
            )
        )
        
        if latest_vel:
            sink_rate = latest_vel.down_m_s
            airspeed = math.hypot(latest_vel.north_m_s, latest_vel.east_m_s)
            descent_speeds.append(sink_rate)
            
            pitch = latest_attitude.pitch_deg if latest_attitude else 0.0
            
            print(f"🎯 ALT: {alt:5.1f}m | Descent: {sink_rate:5.1f} m/s | IAS: {airspeed:5.1f} m/s | Pitch: {pitch:5.1f}°", end="\r")
        
        # Pull-up condition
        if alt <= MIN_SAFE_ALTITUDE:
            print(f"\n✅ Pull-up at {alt:.1f}m")
            break
        
        await asyncio.sleep(CONTROL_RATE)
    
    # Calculate dive time
    dive_time = asyncio.get_event_loop().time() - dive_start_time
    dive_dist = dive_start_alt - alt
    
    # Stats
    print(f"\n📊 DIVE STATISTICS:")
    print(f"   Altitude lost: {dive_dist:.1f}m in {dive_time:.1f}s")
    if dive_time > 0:
        print(f"   Average rate: {dive_dist/dive_time:.1f} m/s")
    
    if descent_speeds:
        avg = sum(descent_speeds) / len(descent_speeds)
        max_desc = max(descent_speeds)
        min_desc = min(descent_speeds)
        print(f"   Descent speeds:")
        print(f"     Average: {avg:.1f} m/s")
        print(f"     Maximum: {max_desc:.1f} m/s")
        print(f"     Minimum: {min_desc:.1f} m/s")
        
        if max_desc >= 20.0:
            print(f"   ✅ EXCELLENT - Achieved {max_desc:.1f} m/s at ~{DIVE_ANGLE_DEG}°!")
        elif max_desc >= 15.0:
            print(f"   ✅ GOOD - Achieved {max_desc:.1f} m/s")
        elif max_desc >= 10.0:
            print(f"   ⚠️  MODERATE - Achieved {max_desc:.1f} m/s")
        else:
            print(f"   ❌ SLOW - Only {max_desc:.1f} m/s")
    
    try:
        await drone.offboard.stop()
    except:
        pass
    
    return True

async def phase_recovery(drone):
    """Recovery"""
    print("\n" + "="*50)
    print("🛬 PHASE 5: RECOVERY")
    print("="*50)
    
    await drone.action.hold()
    await asyncio.sleep(2.0)
    print("✅ Holding position")
    return True

# ================= MAIN =================
async def run():
    global latest_pos, latest_vel, latest_attitude, stop_tasks
    
    print("\n" + "="*60)
    print("⚔️  SIMPLE AGGRESSIVE DIVE")
    print("="*60)
    print("Profile: Climb → FW → Cruise → DIVE → Recover")
    print("="*60)
    
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    print("\n⏳ Connecting...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected")
            break
    
    print("⏳ Waiting for vehicle...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("✅ Ready")
            break
    
    original_params = await configure_params(drone)
    
    # Telemetry
    pos_task = asyncio.create_task(position_listener(drone))
    vel_task = asyncio.create_task(velocity_listener(drone))
    att_task = asyncio.create_task(attitude_listener(drone))
    
    while latest_pos is None or latest_vel is None:
        await asyncio.sleep(0.1)
    
    print("\n💪 Arming...")
    await drone.action.arm()
    print("✅ Armed")
    
    try:
        # Execute
        await phase_climb(drone)
        await phase_transition_fw(drone)
        await phase_cruise_straight(drone)
        # SKIP MC transition - dive in FW mode for pitch!
        await phase_dive_fw(drone)  # FW mode dive with pitch
        await phase_recovery(drone)
        
        print("\n" + "="*60)
        print("✅ MISSION COMPLETE")
        print("="*60)
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n🔄 Shutdown...")
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
        
        # Restore params
        for param, value in original_params.items():
            try:
                await drone.param.set_param_float(param, value)
            except:
                pass
        
        print("✅ Done")

if __name__ == "__main__":
    asyncio.run(run())
