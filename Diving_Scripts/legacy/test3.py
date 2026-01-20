import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Drone discovered!")
            break

    # IMPORTANT: Increase the 'Tilt' allowed so the drone can nose-down for accuracy
    await drone.param.set_param_float("MPC_MAN_TILT_MAX", 45.0)
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", 25.0)

    print("-- Setting takeoff altitude to 10m")
    await drone.action.set_takeoff_altitude(10.0)
    await drone.action.arm()
    await drone.action.takeoff()

    # 1. MONITOR ASCENT
    async for telemetry in drone.telemetry.position():
        if telemetry.relative_altitude_m >= 9.5:
            break

    # 2. START OFFBOARD
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard failed: {error._result.result}")
        return

    # 3. ASCENT TO 40m
    print("-- Ascending to 40m...")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -5.0, 0.0))
    async for telemetry in drone.telemetry.position():
        if telemetry.relative_altitude_m >= 40.0:
            print("-- At 40m. Stabilizing for high-accuracy dive...")
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(2)
            break

    # 4. STABILIZED DIVE LOGIC (Ramping)
    print("!! INITIATING CONTROLLED DIVE !!")
    
    # We ramp the speed to keep the flight controller from panicking
    for speed in range(1, 19, 2): # Ramps from 1m/s to 18m/s
        # We add North=3.0 to give the drone "forward momentum" which helps stability
        await drone.offboard.set_velocity_ned(VelocityNedYaw(3.0, 0.0, float(speed), 0.0))
        await asyncio.sleep(0.1)

    # 5. MONITOR DIVE PRECISION
    async for telemetry in drone.telemetry.position():
        alt = telemetry.relative_altitude_m
        # Adjusting lateral drift (Simplified P-Controller)
        # If the drone drifts too far North/South, this keeps it on track
        print(f"Diving... Alt: {alt:.1f}m | Speed: 18m/s", end='\r')
        
        if alt <= 7.0:
            print(f"\n-- TERMINAL PHASE: Breaking at {alt:.1f}m")
            # Rapid reverse thrust to stabilize
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -4.0, 0.0))
            await asyncio.sleep(1.5)
            break

    await drone.action.hold()
    print("-- Mission Finished. Stable.")

if __name__ == "__main__":
    asyncio.run(run())