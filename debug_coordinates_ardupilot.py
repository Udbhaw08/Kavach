#!/usr/bin/env python3
import asyncio
import subprocess
import re
from mavsdk import System
# Port 14551 for ArduPilot SITL
CONNECTION_STRING = "udp://:14551"

async def get_gazebo_pose(model_name="iris"):
    try:
        # ArduCopter SITL often uses "iris" model name, BUT sometimes "ArduCopter" or "copter_iris"
        # Let's try listing models first to be robust
        result_list = subprocess.check_output(["gz", "model", "-l"], text=True)
        # Find first model that looks like a drone
        target_model = "iris"
        for name in result_list.splitlines():
            if "iris" in name or "copter" in name.lower():
                target_model = name.strip()
                break
        
        print(f"Querying model: {target_model}")
        cmd = ["gz", "model", "-m", target_model, "-p"]
        result = subprocess.check_output(cmd, text=True)
        # Parse position
        # position { x: 0.1 y: 0.2 z: 0.3 }
        pos_match = re.search(r"position\s*{\s*x:\s*([-\d.]+)\s*y:\s*([-\d.]+)\s*z:\s*([-\d.]+)", result)
        if pos_match:
            return float(pos_match.group(1)), float(pos_match.group(2)), float(pos_match.group(3))
    except Exception as e:
        print(f"Error getting Gazebo pose: {e}")
    return None, None, None

async def run():
    drone = System()
    print(f"Connecting to {CONNECTION_STRING}...")
    await drone.connect(system_address=CONNECTION_STRING)

    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    print("Note: Ensure Gazebo is running!")
    
    # 1. Get Initial Pose
    start_x, start_y, start_z = await get_gazebo_pose()
    print(f"INITIAL GAZEBO POSE: X={start_x}, Y={start_y}, Z={start_z}")
    
    print("Arming and Taking Off...")
    try:
        await drone.action.hold() # Force GUIDED
        await drone.action.arm()
        await drone.action.takeoff()
    except Exception as e:
        print(f"Takeoff failed: {e}")
        return

    await asyncio.sleep(10) # Wait for takeoff

    # 2. Move 20m NORTH (MAVSDK X)
    print("Moving 20m NORTH (NED X)...")
    try:
        # fly_to is simple GOTO
        async for pos in drone.telemetry.position():
            curr_lat = pos.latitude_deg
            curr_lon = pos.longitude_deg
            # Rough estimation: 1 deg lat ~ 111111 meters
            new_lat = curr_lat + (20.0 / 111132.0)
            await drone.action.goto_location(new_lat, curr_lon, pos.absolute_altitude_m, 0)
            break
            
        await asyncio.sleep(8)
        
        # Check Pose
        curr_x, curr_y, curr_z = await get_gazebo_pose()
        dx = curr_x - start_x
        dy = curr_y - start_y
        print(f"AFTER 20m NORTH: DeltaGazeboX={dx:.2f}, DeltaGazeboY={dy:.2f}")
        
    except Exception as e:
        print(e)
        
    # 3. Move 20m EAST (MAVSDK Y)
    print("Moving 20m EAST (NED Y)...")
    try:
        async for pos in drone.telemetry.position():
            curr_lat = pos.latitude_deg
            curr_lon = pos.longitude_deg
            # Rough estimation: 1 deg lon = 111132 * cos(lat)
            import math
            scale = math.cos(math.radians(curr_lat))
            new_lon = curr_lon + (20.0 / (111132.0 * scale))
            await drone.action.goto_location(curr_lat, new_lon, pos.absolute_altitude_m, 0)
            break
            
        await asyncio.sleep(8)
        
        # Check Pose
        curr_x2, curr_y2, curr_z2 = await get_gazebo_pose()
        dx = curr_x2 - curr_x
        dy = curr_y2 - curr_y
        print(f"AFTER 20m EAST:  DeltaGazeboX={dx:.2f}, DeltaGazeboY={dy:.2f}")
        
    except Exception as e:
        print(e)

    print("Landing...")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(run())
