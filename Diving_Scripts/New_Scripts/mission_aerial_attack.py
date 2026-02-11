#!/usr/bin/env python3

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw, VelocityNedYaw)

def calculate_yaw(n1, e1, n2, e2):
    return math.degrees(math.atan2(e2-e1, n2-n1))

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

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
    TARGET_RELATIVE_NORTH = 125.0 # meters (Calculated for ~13s flight at 12m/s)
    TARGET_RELATIVE_EAST = 0.0    # meters
    TARGET_RELATIVE_DOWN = -90.0  # meters UP (Angle ~35 deg)
    
    # Aggressive flight params for interception
    print("-- Setting aggressive flight parameters")
    await drone.param.set_param_float("MPC_XY_VEL_MAX", 12.0)
    await drone.param.set_param_float("MPC_Z_VEL_MAX_UP", 12.0)
    await drone.param.set_param_float("MPC_Z_VEL_MAX_UP", 20.0) # Actually ensure this is high
    await drone.param.set_param_float("MPC_TKO_SPEED", 5.0)

    print("-- Arming")
    await drone.action.arm()

    # --- MANUAL TAKEOFF (OFFBOARD) ---
    # Standard action.takeoff() sometimes times out or behaves oddly with aggressive params.
    # We will use Offboard mode to climb to standby altitude.
    
    print(f"-- Climbing to standby altitude ({STANDBY_ALTITUDE}m) in Offboard Mode")
    
    # Get current position
    async for position_ned in drone.telemetry.position_velocity_ned():
        start_n = position_ned.position.north_m
        start_e = position_ned.position.east_m
        start_d = position_ned.position.down_m
        break

    # Send initial setpoint (Current Lat/Lon, but on ground) before starting Offboard
    await drone.offboard.set_position_ned(PositionNedYaw(start_n, start_e, start_d, 0.0))
    
    try:
        await drone.offboard.start()
        print("-- Offboard started for Takeoff")
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Command Climb
    await drone.offboard.set_position_ned(PositionNedYaw(start_n, start_e, -STANDBY_ALTITUDE, 0.0))
    
    # Wait for climb
    while True:
        async for position in drone.telemetry.position():
            curr_alt = position.relative_altitude_m
            # Print altitude for debugging
            # print(f"Altitude: {curr_alt:.1f}m")
            
            if curr_alt > STANDBY_ALTITUDE - 1.0:
                print(f"-- Reached standby altitude: {curr_alt:.1f}m")
                break
            
            # Resend Setpoint
            await drone.offboard.set_position_ned(PositionNedYaw(start_n, start_e, -STANDBY_ALTITUDE, 0.0))
            break # Break async loop to sleep
        
        # Check completion condition again outside async loop to break while loop
        if curr_alt > STANDBY_ALTITUDE - 1.0:
            break
            
        await asyncio.sleep(0.1)
            
    print("-- SCANNING FOR TARGET...")
    await asyncio.sleep(5) # Simulate scanning delay
    
    print("-- TARGET LOCKED! INITIATING INTERCEPT")
    
    # Get current position to calculate target NED
    async for position_ned in drone.telemetry.position_velocity_ned():
        current_n = position_ned.position.north_m
        current_e = position_ned.position.east_m
        current_d = position_ned.position.down_m
        break

    # Calculate Intercept Point
    target_n = current_n + TARGET_RELATIVE_NORTH
    target_e = current_e + TARGET_RELATIVE_EAST
    target_d = current_d + TARGET_RELATIVE_DOWN # Remember: Down is positive, Up is negative

    # Compute Yaw to face target
    target_yaw = calculate_yaw(current_n, current_e, target_n, target_e)

    print(f"-- Intercept Coords (NED): N:{target_n:.1f}, E:{target_e:.1f}, D:{target_d:.1f}, Yaw:{target_yaw:.1f}deg")
    
    # --- SPAWN VISUAL TARGET (Optional helper) ---
    # Standard Gazebo coordinates are ENU, so we need to convert NED to ENU for spawning
    # Gazebo X = East = NED East
    # Gazebo Y = North = NED North
    # Gazebo Z = Up = -NED Down
    # BUT wait, standard mapping is usually:
    # Gazebo World Frame (ENU): X=East, Y=North, Z=Up
    # PX4 Local Frame (NED): X=North, Y=East, Z=Down
    # So:
    # Gazebo X = PX4 Y (East)
    # Gazebo Y = PX4 X (North)
    # Gazebo Z = -PX4 Z (Up)
    
    spawn_cmd = f"gz model --spawn-file=$(pwd)/models/target_box.sdf --model-name=target_box -x {target_e} -y {target_n} -z {-target_d}"
    print(f"-- To visualize target in Gazebo, run:\n   {spawn_cmd}")

    # --- OFFBOARD ATTACK ---
    print(f"-- Setting initial setpoint (Yaw {target_yaw} deg)")
    # Align to target yaw first?
    await drone.offboard.set_position_ned(PositionNedYaw(current_n, current_e, current_d, target_yaw))

    try:
        await drone.offboard.start()
        print("-- Offboard started. ENGAGING!")
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Go to target
    await drone.offboard.set_position_ned(PositionNedYaw(target_n, target_e, target_d, target_yaw))

    # Monitor Proximity
    hit_threshold = 3.0 # meters
    intercept_complete = False

    while not intercept_complete:
        async for position_ned in drone.telemetry.position_velocity_ned():
            curr_n = position_ned.position.north_m
            curr_e = position_ned.position.east_m
            curr_d = position_ned.position.down_m
            
            # Simple Euclidean distance
            dist = math.sqrt((curr_n - target_n)**2 + (curr_e - target_e)**2 + (curr_d - target_d)**2)
            
            # Continuously send setpoint (MAVSDK requires frequent updates to keep offboard alive)
            await drone.offboard.set_position_ned(PositionNedYaw(target_n, target_e, target_d, target_yaw))
            
            if dist < hit_threshold:
                print(f"-- TARGET INTERCEPTED! (Distance: {dist:.1f}m)")
                intercept_complete = True
                break
            break
        
        await asyncio.sleep(0.05) # Faster loop for aggressive flight

    print("-- Attack Run Complete. Recovery...")
    
    # Halt at target? Or pass through?
    # User said "lock at any location... pretend to be like attack... get hitted".
    # Stopping essentially "hits" it.
    
    # Just to be fancy, let's hold for 2s then Return
    await asyncio.sleep(2.0)
    
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed: {error._result.result}")

    print("-- Returning to Launch")
    await drone.action.return_to_launch()

if __name__ == "__main__":
    asyncio.run(run())
