import asyncio
import csv
from datetime import datetime
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

    # Prepare for data logging
    dive_data = []
    
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", 25.0)
    await drone.action.set_takeoff_altitude(10.0)
    await drone.action.arm()
    await drone.action.takeoff()

    # Wait for stable air at 10m
    async for telemetry in drone.telemetry.position():
        if telemetry.relative_altitude_m >= 9.5:
            break

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard failed: {error}")
        return

    # PHASE: Ascent to 40m
    print("-- Ascending to 40m...")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -5.0, 0.0))
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= 40.0:
            print("-- Ready at 40m. Recording Dive Data...")
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(2)
            break

    # --- THE DIVE ---
    print("!! INITIATING DIVE !!")
    # Set the dive command: 18m/s down, 2m/s North (to keep it "dart-like")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(2.0, 0.0, 18.0, 0.0))

    # Start capturing high-frequency logs during the dive
    async for pos in drone.telemetry.position():
        async for vel in drone.telemetry.velocity_ned():
            alt = pos.relative_altitude_m
            # Log: Timestamp, Lat, Lon, Alt, North_Vel, East_Vel, Down_Vel
            current_log = {
                "timestamp": datetime.now().strftime("%H:%M:%S.%f"),
                "alt": round(alt, 2),
                "north_pos": round(pos.latitude_deg, 6),
                "east_pos": round(pos.longitude_deg, 6),
                "v_down": round(vel.down_m_s, 2)
            }
            dive_data.append(current_log)
            
            print(f"ALT: {current_log['alt']}m | V_DOWN: {current_log['v_down']} m/s | DRIFT: {current_log['north_pos']}", end='\r')
            break # Break internal velocity loop to sync with position

        if alt <= 6.5:
            print(f"\n-- BREAKING: {alt}m reached.")
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -2.0, 0.0))
            break

    # Save logs to CSV for your review
    filename = f"dive_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=dive_data[0].keys())
        writer.writeheader()
        writer.writerows(dive_data)
    
    print(f"-- Logs saved to {filename}")
    await drone.action.hold()

if __name__ == "__main__":
    asyncio.run(run())