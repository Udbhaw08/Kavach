import asyncio
import csv
from datetime import datetime
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
    dive_data = [] # List to store logs
    await drone.param.set_param_float("MPC_MAN_TILT_MAX", 60.0) 
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", 25.0)

    print("-- Setting takeoff altitude to 10m")
    await drone.action.set_takeoff_altitude(10.0)
    await drone.action.arm()
    await drone.action.takeoff()

    # Wait for stable air at 10m
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= 9.5: break

    # 2. ASCENT TO 40m
    print("-- Ascending to 40m...")
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard failed: {e}")
        return

    # Body Frame: Negative Z is UP
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, -5.0, 0.0))

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= 40.0:
            print("-- At 40m. Leveling for dive...")
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(2)
            break

    # 3. THE NOSE-DOWN DIVE WITH LOGGING
    print("!! INITIATING NOSE-DOWN DIVE !!")
    
    # Body Frame Logic: forward=5.0 forces the PITCH DOWN
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(5.0, 0.0, 18.0, 0.0))

    # Capture high-frequency telemetry during the dive
    async for pos in drone.telemetry.position():
        # Get current velocity for the log
        async for vel in drone.telemetry.velocity_ned():
            alt = pos.relative_altitude_m
            
            # Record data point
            current_log = {
                "timestamp": datetime.now().strftime("%H:%M:%S.%f"),
                "alt": round(alt, 2),
                "lat": pos.latitude_deg,
                "lon": pos.longitude_deg,
                "v_down": round(vel.down_m_s, 2)
            }
            dive_data.append(current_log)
            
            print(f"Diving... Alt: {current_log['alt']}m | V_Down: {current_log['v_down']} m/s", end='\r')
            break 
        
        if pos.relative_altitude_m <= 7.0:
            print(f"\n-- PULLING UP: Breaking at {pos.relative_altitude_m:.1f}m")
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(-2.0, 0.0, -5.0, 0.0))
            await asyncio.sleep(1.5)
            break

    # 4. SAVE DATA & HOLD
    await drone.action.hold()
    
    if dive_data:
        filename = f"dive_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=dive_data[0].keys())
            writer.writeheader()
            writer.writerows(dive_data)
        print(f"-- Logs saved to {filename}")

    print("-- Mission Finished.")

if __name__ == "__main__":
    asyncio.run(run())