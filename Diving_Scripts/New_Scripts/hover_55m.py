
import asyncio
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    
    await asyncio.sleep(5)
    
    # Get current position
    async for position in drone.telemetry.position():
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
        current_abs_alt = position.absolute_altitude_m
        current_rel_alt = position.relative_altitude_m
        break

    target_rel_alt = 55.0
    target_abs_alt = current_abs_alt + (target_rel_alt - current_rel_alt)
    
    print(f"-- Climbing to {target_rel_alt}m relative altitude")
    await drone.action.goto_location(current_lat, current_lon, target_abs_alt, 90)

    while True:
        async for position in drone.telemetry.position():
            print(f"Altitude: {position.relative_altitude_m:.1f} m")
            if abs(position.relative_altitude_m - target_rel_alt) < 1.0:
                print("-- Reached target altitude")
                break
        if abs(position.relative_altitude_m - target_rel_alt) < 1.0:
            break
        await asyncio.sleep(1)

    print("-- Holding at 55m")
    while True:
        await asyncio.sleep(5)

if __name__ == "__main__":
    asyncio.run(run())
