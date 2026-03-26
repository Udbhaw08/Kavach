#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.action import ActionError

async def run():
    # Connect to the ArduPilot SITL
    # Using 14551 as established in previous setup
    drone = System()
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

    print("-- Switching to GUIDED mode")
    try:
        await drone.action.hold()
    except Exception as e:
        print(f"Failed to set mode: {e}")

    print("-- Arming")
    try:
        await drone.action.arm()
    except ActionError as e:
        print(f"Arming failed: {e}")
        return

    print("-- Taking off to 20 meters")
    try:
        # ArduPilot takeoff altitude is often set via parameter or command
        await drone.action.set_takeoff_altitude(20.0)
        await drone.action.takeoff()
    except ActionError as e:
        print(f"Takeoff failed: {e}")
        return

    # Monitor altitude
    async for position in drone.telemetry.position():
        relative_altitude = position.relative_altitude_m
        print(f"-- Current altitude: {relative_altitude:.1f} m")
        if relative_altitude >= 19.5:
            print("-- Reached 20 meters!")
            break

    print("-- Drone will now hover at 20 meters.")
    # Keep the script alive to maintain connection/monitoring if desired
    # Or just let it finish; the drone stays in GUIDED hover by default in ArduPilot
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(run())
