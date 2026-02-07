#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, Attitude)

async def print_telemetry(drone):
    """
    Continually prints telemetry data to the console, similar to QGC overlay.
    """
    print("-- Starting Telemetry Log")
    async for position_velocity_ned in drone.telemetry.position_velocity_ned():
        # Calculate horizontal speed
        v_n = position_velocity_ned.velocity.north_m_s
        v_e = position_velocity_ned.velocity.east_m_s
        h_speed = (v_n**2 + v_e**2)**0.5
        
        # Vertical speed (simulating QGC which shows positive for up)
        v_z = -position_velocity_ned.velocity.down_m_s
        
        # Altitude (Relative is usually what's shown prominently)
        try:
            position = await drone.telemetry.position().__aiter__().__anext__()
            alt_rel = position.relative_altitude_m
            alt_abs = position.absolute_altitude_m
        except:
            alt_rel = 0.0
            alt_abs = 0.0

        # Print formatted string (using \r to update line if supported, or just print)
        # For scrolling log:
        print(f"[TELEM] Alt: {alt_rel:.1f}m | V-Spd: {v_z:.1f}m/s | H-Spd: {h_speed:.1f}m/s")
        
        # Update rate
        await asyncio.sleep(1)

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
            
    # Start the telemetry printing background task
    asyncio.create_task(print_telemetry(drone))

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Setting ascent speed to 15 m/s")
    # Set maximum vertical velocity up (Manual/Stabilized)
    await drone.param.set_param_float("MPC_Z_VEL_MAX_UP", 50.0)
    # Set auto mission/goto ascent speed (Auto modes)
    await drone.param.set_param_float("MPC_Z_V_AUTO_UP", 20.0)
    # Set takeoff speed
    await drone.param.set_param_float("MPC_TKO_SPEED", 15.0)

    # Log parameters to console
    mpc_z_vel_max_up = await drone.param.get_param_float("MPC_Z_VEL_MAX_UP")
    mpc_z_v_auto_up = await drone.param.get_param_float("MPC_Z_V_AUTO_UP")
    mpc_tko_speed = await drone.param.get_param_float("MPC_TKO_SPEED")
    
    print(f"-- PARAM: MPC_Z_VEL_MAX_UP = {mpc_z_vel_max_up} m/s")
    print(f"-- PARAM: MPC_Z_V_AUTO_UP = {mpc_z_v_auto_up} m/s")
    print(f"-- PARAM: MPC_TKO_SPEED = {mpc_tko_speed} m/s")

    print("-- Setting descent speed to 10 m/s")
    # Set maximum vertical velocity down
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", 10.0)
    # Set auto mission/goto descent speed
    await drone.param.set_param_float("MPC_Z_V_AUTO_DN", 10.0)
    await drone.param.set_param_float("MPC_LAND_SPEED", 10.0) # Careful with this one near ground!

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    print("Waiting for drone to reach takeoff altitude...")
    # Wait a bit for takeoff to finish
    await asyncio.sleep(10)

    print("-- climbing to 400m altitude")
    # Get current position
    async for position in drone.telemetry.position():
        absolute_altitude = position.absolute_altitude_m
        latitude = position.latitude_deg
        longitude = position.longitude_deg
        break
    
    target_altitude = absolute_altitude + 400.0
    await drone.action.goto_location(latitude, longitude, target_altitude, 0)
    
    print("Waiting to reach 400m...")
    async for position in drone.telemetry.position():
        # Check if we are within 5m of target
        if abs(position.absolute_altitude_m - target_altitude) < 5.0:
            print("-- Reached 400m altitude!")
            break
            
    print("-- TRANSITIONING TO FIXED WING")
    await drone.action.transition_to_fixedwing()
    
    

    print("-- Loitering in Fixed Wing mode for 30 seconds...")
    await asyncio.sleep(30)
    
    print("-- STARTING DIVE ATTACK")
    print("-- Setting initial Setpoint (Flat)") 
    # We must send a setpoint before starting offboard mode
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
    
    try:
        await drone.offboard.start()
        print("-- Offboard started")
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- DISARMING AND ABORTING")
        await drone.action.disarm()
        return

    print("-- DIVING: Pitch -50 deg, Thrust 80%")
    # Lock pitch to -50 degrees, high thrust
    
    # We use a flag to break the outer loop
    dive_complete = False
    
    while not dive_complete:
        # Check altitude
        async for position in drone.telemetry.position():
            curr_alt = position.relative_altitude_m
            
            # Send dive command
            await drone.offboard.set_attitude(Attitude(0.0, -50.0, 0.0, 0.8))
            
            if curr_alt < 100.0:
                 print(f"-- Reached pull-up altitude: {curr_alt:.1f}m")
                 dive_complete = True
                 break # Breaks the async for loop
            
            # Rate limit loop slightly
            await asyncio.sleep(0.05)
            # We must break the async for loop to cycle the while loop and allow sleep
            # OR just process one position update per cycle.
            break 
            
    print("-- ABORTING DIVE / RECOVERY")
    # Stop offboard to return to Hold/Auto or switch mode
    try:
        await drone.offboard.stop()
    except:
        pass

    print("-- TRANSITIONING TO MULTICOPTER (BRAKING)")
    await drone.action.transition_to_multicopter()
    await asyncio.sleep(5)

    print("-- Landing")
    await drone.action.land()
    
    print("Waiting for drone to land...")
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            print("-- Landed!")
            break
            
    # Disarm after landing
    print("-- Disarming")
    try:
        await drone.action.disarm()
    except:
        print("Already disarmed")

if __name__ == "__main__":
    asyncio.run(run())
