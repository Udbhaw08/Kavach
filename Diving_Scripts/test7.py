import asyncio
from mavsdk import System
from mavsdk.offboard import Attitude, OffboardError

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Drone discovered!")
            break

    # 1. PRE-FLIGHT HEALTH CHECK (Fixes COMMAND_DENIED)
    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    # 2. SETUP ATTACK PARAMETERS
    await drone.param.set_param_float("MPC_MAN_TILT_MAX", 65.0) 
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", 25.0)

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off to 10m (Stable Mode)")
    await drone.action.set_takeoff_altitude(10.0)
    await drone.action.takeoff()

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= 9.5:
            break

    # 3. ASCENT TO 40m PERCH
    # Start heartbeat for Offboard
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError:
        return

    print("-- Climbing to 40m...")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.75)) # 75% Thrust Up
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= 40.0:
            print("-- At 40m. Leveling for Dive...")
            await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.5))
            await asyncio.sleep(2)
            break

    # 4. THE NOSE-DOWN ATTACK DIVE
    print("!! INITIATING SEAMLESS DIVE (45° PITCH) !!")
    
    # Attitude: (roll, pitch, yaw, thrust)
    # Pitch = 45.0 degrees (Nose Down)
    # Thrust = 0.85 (Strong forward/downward drive)
    await drone.offboard.set_attitude(Attitude(0.0, 45.0, 0.0, 0.85))

    async for pos in drone.telemetry.position():
        alt = pos.relative_altitude_m
        print(f"Diving... Alt: {alt:.1f}m | Status: PITCHED DOWN", end='\r')
        
        if alt <= 6.0:
            print(f"\n-- PULLING UP: Breaking at {alt:.1f}m")
            # Rapid reverse-pitch to level out
            await drone.offboard.set_attitude(Attitude(0.0, -20.0, 0.0, 0.95))
            await asyncio.sleep(1.0)
            break

    await drone.action.hold()
    print("-- Mission Finished. Stable.")

if __name__ == "__main__":
    asyncio.run(run())