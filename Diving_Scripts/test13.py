#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    VelocityNedYaw,
    PositionNedYaw
)

# ---------------- CONFIG ----------------
TAKEOFF_ALT = 5.0
TARGET_ALT = 70.0
CLIMB_RATE = -10.0          # m/s (negative = UP)
OFFBOARD_RATE = 0.1        # 10 Hz
# ----------------------------------------


async def wait_for_connection(drone):
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected to drone")
            return


async def wait_for_position_lock(drone):
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✅ Position lock acquired")
            return


async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    await wait_for_connection(drone)
    await wait_for_position_lock(drone)

    # -------- Arm & Takeoff --------
    print("🚁 Arming...")
    await drone.action.arm()

    print(f"⬆️ Taking off to {TAKEOFF_ALT} m...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT)
    await drone.action.takeoff()

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= TAKEOFF_ALT - 0.5:
            print("✅ Takeoff altitude reached")
            break

    # -------- ENTER OFFBOARD (POSITION FIRST) --------
    print("📡 Initializing Offboard with position hold...")

    for _ in range(15):
        await drone.offboard.set_position_ned(
            PositionNedYaw(
                north_m=0.0,
                east_m=0.0,
                down_m=-TAKEOFF_ALT,
                yaw_deg=0.0
            )
        )
        await asyncio.sleep(OFFBOARD_RATE)

    try:
        await drone.offboard.start()
        print("✅ Offboard started successfully")
    except OffboardError as e:
        print(f"❌ Offboard failed: {e}")
        return

    # -------- SWITCH TO VELOCITY CONTROL --------
    print("⬆️ Climbing toward 20 m at 6 m/s")

    reached = False
    while not reached:
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(
                north_m_s=0.0,
                east_m_s=0.0,
                down_m_s=CLIMB_RATE,
                yaw_deg=0.0
            )
        )

        async for pos in drone.telemetry.position():
            print(f"📏 Altitude: {pos.relative_altitude_m:.2f} m")
            if pos.relative_altitude_m >= TARGET_ALT - 0.5:
                reached = True
            break

        await asyncio.sleep(OFFBOARD_RATE)

    # -------- HOLD --------
    print("🛑 Holding position at 20 m")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    )

    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(main())
