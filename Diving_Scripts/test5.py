import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Drone discovered!")
            break

    # 1. SETUP & PARAMETERS
    # Allow more aggressive tilting for a realistic dive angle
    await drone.param.set_param_float("MPC_MAN_TILT_MAX", 60.0) 
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", 25.0)

    await drone.action.set_takeoff_altitude(10.0)
    await drone.action.arm()
    await drone.action.takeoff()

    # Wait for stable air at 10m
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= 9.5: break

    # 2. ASCENT TO 40m
    print("-- Ascending to 40m...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.start()
    # Body Frame: Negative Z is UP
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -5.0, 0.0))

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= 40.0:
            print("-- At 40m. Leveling for dive...")
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(2)
            break

    # 3. THE NOSE-DOWN DIVE
    print("!! INITIATING NOSE-DOWN DIVE !!")
    
    # Body Frame Logic:
    # forward_m_s = 5.0 (This forces the drone to PITCH DOWN to move forward)
    # down_m_s = 18.0 (This provides the vertical descent)
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(5.0, 0.0, 18.0, 0.0))

    async for pos in drone.telemetry.position():
        alt = pos.relative_altitude_m
        # Print actual vertical speed from your logs
        print(f"Diving... Alt: {alt:.1f}m | Pitching Forward", end='\r')
        
        if alt <= 7.0:
            print(f"\n-- PULLING UP: Breaking at {alt:.1f}m")
            # Rapid reverse thrust to level out
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(-2.0, 0.0, -5.0, 0.0))
            await asyncio.sleep(1.5)
            break

    await drone.action.hold()
    print("-- Mission Finished. Drone should have tilted significantly.")

if __name__ == "__main__":
    asyncio.run(run())
    