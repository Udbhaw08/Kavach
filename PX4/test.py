import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

async def main():
    drone = System()
    await drone.connect(system_address="udp://0.0.0.0:14540")
    
    print("Connecting...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    print("Arming...")
    await drone.action.arm()

    print("Starting Offboard...")
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, -5, 0))
    
    try:
        await drone.offboard.start()
        print("Offboard started!")
    except OffboardError as e:
        print("Offboard start failed:", e)
        return

    print("Moving forward (NED X positive)...")
    await drone.offboard.set_position_ned(PositionNedYaw(2, 0, -5, 0))
    await asyncio.sleep(5)

    print("Stopping...")
    await drone.offboard.stop()

asyncio.run(main())
