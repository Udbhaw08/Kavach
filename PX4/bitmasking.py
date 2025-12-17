import asyncio
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected")
            break

    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(8)

    # Initial setpoints (MANDATORY)
    for _ in range(20):
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.1)

    try:
        await drone.offboard.start()
        print("🟢 OFFBOARD started")
    except OffboardError as e:
        print("❌ OFFBOARD failed:", e)
        return

    # Climb to ~50m
    print("⬆️ Climbing")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, -3.0, 0.0)
    )
    await asyncio.sleep(15)

    # Fly forward at 10 m/s
    print("➡️ Flying at 10 m/s")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(10.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(20)

    # Stop
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(2)

    await drone.offboard.stop()
    await drone.action.land()

asyncio.run(run())
