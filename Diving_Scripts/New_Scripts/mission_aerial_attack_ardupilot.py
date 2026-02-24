#!/usr/bin/env python3

import asyncio
import math
import subprocess
import os
from mavsdk import System
# ArduPilot uses "GUIDED" mode which maps somewhat to Offboard, 
# but often we must use Position/Velocity NED calls or Action commands.
# MAVSDK's Offboard plugin *can* work if configured correctly, 
# but standard ArduPilot usage often benefits from simple GOTO for position.

from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw)

def calculate_yaw(n1, e1, n2, e2):
    return math.degrees(math.atan2(e2-e1, n2-n1))

async def run():
    print("-- Starting ArduPilot Mission Script")
    drone = System()
    # ArduPilot SITL (MavProxy) usually outputs to 14550 (GCS) or 14551. 
    # MAVSDK default is 14540 (PX4).
    # Since QGC often uses 14550, let's use 14551. 
    # When running sim_vehicle.py, add --out=udp:127.0.0.1:14551
    await drone.connect(system_address="udp://:14551")

    print("-- Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("-- Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    # --- CONFIGURATION ---
    STANDBY_ALTITUDE = 25.0
    TARGET_RELATIVE_NORTH = 125.0 # meters
    TARGET_RELATIVE_EAST = 0.0    # meters
    TARGET_RELATIVE_EAST = 0.0    # meters
    TARGET_RELATIVE_DOWN = -90.0  # meters UP
    
    # --- FLIGHT PERFORMANCE CONFIGURATION ---
    # Adjust these values to tune the attack speed
    SPEED_HORIZONTAL = 1200.0  # cm/s (12 m/s)
    SPEED_UP = 800.0           # cm/s (8 m/s) - INCREASED for faster ascent
    ACCEL_Z = 400.0            # cm/s/s (4 m/s^2) - Vertical acceleration
    ACCEL_XY = 600.0           # cm/s/s (6 m/s^2) - Horizontal acceleration
    
    # ArduPilot specific param setting might differ slightly in naming, 
    # closer to WPNAV_SPEED, WPNAV_SPEED_UP, PILOT_SPEED_UP
    print("-- Setting aggressive flight parameters (ArduPilot)")
    try:
        # Try setting parameters, handle potential type mismatches
        params_to_set = [
            ("WPNAV_SPEED", SPEED_HORIZONTAL), 
            ("WPNAV_SPEED_UP", SPEED_UP), 
            ("PILOT_SPEED_UP", SPEED_UP),
            ("WPNAV_ACCEL", ACCEL_XY),
            ("WPNAV_ACCEL_Z", ACCEL_Z),
            ("PILOT_ACCEL_Z", ACCEL_Z)
        ]
        for name, val in params_to_set:
            try:
                await drone.param.set_param_float(name, val)
                print(f"Set {name} to {val} (float)")
            except Exception:
                try:
                    await drone.param.set_param_int(name, int(val))
                    print(f"Set {name} to {int(val)} (int)")
                except Exception as e:
                    print(f"Failed to set {name}: {e}")
    except Exception as e:
        print(f"Warning: Param setup issues: {e}")

    print("-- Switching to GUIDED mode")
    # ArduPilot requires GUIDED mode for offboard/MAVSDK control.
    # We force it before arming to ensure arming checks pass (STABILIZE might require 0 throttle).
    try:
        await drone.action.hold() # Maps to GUIDED in ArduCopter usually
    except Exception as e:
        print(f"Could not switch to Hold/Guided: {e}")

    print("-- Arming")
    try:
        await drone.action.arm()
    except Exception as e:
        print(f"Arming failed: {e}")
        # Sometimes it takes a moment for EKF to be ready in SITL
        print("Waiting 5s and retrying arming...")
    print("-- Arming")
    # Robust Arming Loop
    max_arm_retries = 10
    for i in range(max_arm_retries):
        # Check if already armed
        is_armed = False
        async for armed in drone.telemetry.armed():
            is_armed = armed
            break
        
        if is_armed:
            print("-- Drone is ARMED.")
            break
            
        print(f"   Attempting to ARM (Try {i+1}/{max_arm_retries})...")
        try:
            await drone.action.hold() # Ensure Guided
            await drone.action.arm()
        except Exception as e:
            print(f"   Arm command failed: {e}")
        
        await asyncio.sleep(2)
    
    # Final check
    async for armed in drone.telemetry.armed():
        if not armed:
            print("CRITICAL ERROR: Failed to ARM drone. Mission cannot proceed.")
            print("Check MavProxy console for PreArm errors (GPS, Compass, etc).")
            return
        break

    print(f"-- Taking off to standby altitude ({STANDBY_ALTITUDE}m)")
    try:
        await drone.action.set_takeoff_altitude(STANDBY_ALTITUDE)
    except: pass
    
    try:
        await drone.action.takeoff()
    except Exception as e:
         print(f"Takeoff command failed: {e}")

    # Wait until we reach standby altitude
    print("-- Waiting for altitude...")
    # Add timeout to avoid infinite loop
    for i in range(60): # 60 seconds max
        curr_alt = 0.0
        is_armed = False
        
        # Get Telemetry Snapshot
        async for position in drone.telemetry.position():
            curr_alt = position.relative_altitude_m
            break
        async for armed in drone.telemetry.armed():
            is_armed = armed
            break

        if curr_alt > STANDBY_ALTITUDE - 1.0:
            print(f"-- Reached standby altitude: {curr_alt:.1f}m")
            break
        
        if not is_armed:
            print("   Drone Disarmed! Re-arming and taking off...")
            try:
                await drone.action.arm()
                await drone.action.takeoff()
            except: pass
        
        # Logic to print status every ~5s
        if i % 5 == 0:
             print(f"   Climbing... current alt: {curr_alt:.1f}m (Armed: {is_armed})")
             if curr_alt < 1.0 and is_armed:
                 print("   Stuck on ground? Retrying Takeoff...")
                 try:
                     await drone.action.takeoff()
                 except: pass
             elif curr_alt >= 1.0:
                 # In air but not at target? Use Goto
                 try:
                     async for pos in drone.telemetry.position():
                         await drone.action.goto_location(pos.latitude_deg, pos.longitude_deg, STANDBY_ALTITUDE, 0)
                         break
                 except: pass

        await asyncio.sleep(1)
        
    # Real altitude wait loop
    print("   (Verifying final altitude)")
    async for position in drone.telemetry.position():
        if position.relative_altitude_m > STANDBY_ALTITUDE - 1.0:
            break
        if position.relative_altitude_m < 0.5:
             print("   Warning: Not climbing? Retrying Takeoff command.")
             try:
                 await drone.action.takeoff()
             except: pass
        await asyncio.sleep(1) # just wait a bit in the loop? 
        # No, async for blocks until next update. Telemetry is fast.
        # We should just break and assume it works or handle it properly.
        # For this script, let's just proceed if we are "close enough" or time follows.
        if position.relative_altitude_m > STANDBY_ALTITUDE - 1.0:
            break
            
    print("-- SCANNING FOR TARGET...")
    await asyncio.sleep(5) 
    
    print("-- TARGET LOCKED! INITIATING INTERCEPT")
    
    # Get current position
    async for position_ned in drone.telemetry.position_velocity_ned():
        current_n = position_ned.position.north_m
        current_e = position_ned.position.east_m
        current_d = position_ned.position.down_m
        break

    # Calculate Intercept Point
    target_n = current_n + TARGET_RELATIVE_NORTH
    target_e = current_e + TARGET_RELATIVE_EAST
    target_d = current_d + TARGET_RELATIVE_DOWN 

    # Compute Yaw
    target_yaw = calculate_yaw(current_n, current_e, target_n, target_e)

    print(f"-- Intercept Coords (NED): N:{target_n:.1f}, E:{target_e:.1f}, D:{target_d:.1f}, Yaw:{target_yaw:.1f}deg")
    
    # --- SPAWN VISUAL TARGET ---
    # --- SPAWN VISUAL TARGET ---
    # Convert NED to ENU for Gazebo
    # Gazebo X = East, Gazebo Y = North, Gazebo Z = Up (-Down)
    # BUT we need to account for the drone's *current actual* position in Gazebo world.
    # MAVSDK Local NED 0,0 might not be Gazebo 0,0.

    drone_gazebo_x = 0.0
    drone_gazebo_y = 0.0
    drone_gazebo_z = 0.0

    print("-- Querying Gazebo for actual drone position...")
    try:
        # Assuming model name is 'iris' or 'ArduCopter' - 'manual' launch usually uses 'iris' from world file
        # Only 'iris' is standard in the world I launched.
        # We can try to list models to find it.
        # But 'iris' is safe for 'iris_arducopter_runway'
        result = subprocess.check_output(["gz", "model", "-m", "iris", "-p"], text=True)
        # Output format is usually varying, but let's try 'gz model -i' which is cleaner? 
        # Actually 'gz model -p' prints formatted pose:
        # "position { x: 1.2 y: 3.4 z: 0.0 } orientation { ... }"
        # Use simple string parsing
        import re
        pos_match = re.search(r"position\s*{\s*x:\s*([-\d.]+)\s*y:\s*([-\d.]+)\s*z:\s*([-\d.]+)", result)
        if pos_match:
            drone_gazebo_x = float(pos_match.group(1))
            drone_gazebo_y = float(pos_match.group(2))
            drone_gazebo_z = float(pos_match.group(3))
            print(f"-- Drone Gazebo Pose: X={drone_gazebo_x:.1f}, Y={drone_gazebo_y:.1f}")
        else:
            print("-- Could not parse Gazebo pose. Assuming 0,0.")
    except Exception as e:
        print(f"-- Failed to query Gazebo pose ({e}). Assuming 0,0.")

    # Target Offset (NED) -> Gazebo (Runway Frame where X=North, Y=East)
    # The previous attempt assumed Standard ENU (X=East, Y=North), causing the box to appear on the right (East) 
    # when it should have been Forward (North).
    #
    # Correction:
    # Gazebo X = North (Forward)
    # Gazebo Y = East (Right)
    
    # Spawn X (North) = Drone_Gazebo_X + Delta_N
    # Spawn Y (East)  = Drone_Gazebo_Y + Delta_E
    spawn_x = drone_gazebo_x + TARGET_RELATIVE_NORTH
    spawn_y = drone_gazebo_y + TARGET_RELATIVE_EAST
    
    # For Height: 
    # MAVSDK Down is -Altitude. 
    # Target Down = -90 (90m up).
    # Gazebo Z is Up.
    # Since parsing Gazebo Pose might fail (defaulting to 0,0,0), we should rely on the calculated Target NED 
    # if we assume the drone took off from Z=0 (Ground).
    # Target D is -115 (115m Up). So Spawn Z should be 115.
    spawn_z = -target_d

    # Use absolute path for robustness
    spawn_script = os.path.expanduser("~/Kavach/spawn_target.sh")
    
    print(f"-- Spawning target at Gazebo coords: X={spawn_x:.1f}, Y={spawn_y:.1f}, Z={spawn_z:.1f}")
    
    try:
        subprocess.Popen([spawn_script, str(spawn_x), str(spawn_y), str(spawn_z)], 
                         stdout=subprocess.DEVNULL, 
                         stderr=subprocess.DEVNULL)
        print("-- Target spawn command sent.")
    except Exception as e:
        print(f"Failed to auto-spawn target: {e}")
        print(f"Run manually: {spawn_script} {spawn_x} {spawn_y} {spawn_z}")

    # --- OFFBOARD ATTACK (ArduPilot) ---
    # use Velocity Control for straight-line intercept
    # vector = Target - Current
    # Velocity = vector.normalized() * SPEED

    print(f"-- Setting initial setpoint (Yaw {target_yaw} deg)")
    # Initial setpoint to start offboard
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, target_yaw))

    try:
        await drone.offboard.start()
        print("-- Offboard started. ENGAGING!")
    except OffboardError as error:
        print(f"Starting offboard mode failed: {error._result.result}")
        print("-- Trying to force GUIDED mode via Hold first...")
        await drone.action.hold()
        await asyncio.sleep(1)
        # Retry
        try:
             await drone.offboard.start()
        except: pass

    # Monitor Proximity & Control Velocity
    hit_threshold = 3.0 # meters
    intercept_complete = False

    while not intercept_complete:
        async for position_ned in drone.telemetry.position_velocity_ned():
            curr_n = position_ned.position.north_m
            curr_e = position_ned.position.east_m
            curr_d = position_ned.position.down_m
            
            # Vector to Target
            dn = target_n - curr_n
            de = target_e - curr_e
            dd = target_d - curr_d
            dist = math.sqrt(dn**2 + de**2 + dd**2)
            
            if dist < hit_threshold:
                print(f"-- TARGET INTERCEPTED! (Distance: {dist:.1f}m)")
                intercept_complete = True
                # Stop
                await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, target_yaw))
                break
            
            # Normalize Vector
            if dist > 0.1:
                vn = (dn / dist) * (SPEED_HORIZONTAL / 100.0) # convert cm/s to m/s
                ve = (de / dist) * (SPEED_HORIZONTAL / 100.0)
                vd = (dd / dist) * (SPEED_HORIZONTAL / 100.0) # Use horizontal max for total mag?
                
                # We can limit Vz separately if needed, but for "Straight Line" 
                # we should just scale the whole vector by the desired speed.
                # Use SPEED_HORIZONTAL as the master "Attack Speed" (e.g. 12 m/s).
                pass
            else:
                vn, ve, vd = 0,0,0

            # Send Velocity Command
            await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, target_yaw))
            
            # Debug occasional print
            # print(f"Dist: {dist:.1f}m, Vel: {vn:.1f}, {ve:.1f}, {vd:.1f}")
            break
        
        await asyncio.sleep(0.05)



    print("-- Attack Run Complete. Recovery...")
    
    # Stop Offboard -> GUIDED/Hold Mode
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed: {error._result.result}")

    print("-- Returning to Launch")
    await drone.action.return_to_launch()

if __name__ == "__main__":
    asyncio.run(run())
