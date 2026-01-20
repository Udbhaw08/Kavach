#!/usr/bin/env python3
import asyncio
import math
from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    VelocityNedYaw,
    PositionNedYaw,
    Attitude
)

# ---------------- CONFIG ----------------
TAKEOFF_ALT = 5.0
TARGET_ALT = 70.0
CLIMB_RATE = -6.0          # m/s (negative = UP)
OFFBOARD_RATE = 0.1        # 10 Hz

ATTITUDE_DURATION = 10.0   # seconds
PITCH_DEG = -10.0          # nose down
THRUST = 0.6               # hover-ish
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


async def arm_if_needed(drone):
    async for armed in drone.telemetry.armed():
        if armed:
            print("ℹ️ Vehicle already armed")
            return
        break

    print("🚁 Arming...")
    await drone.action.arm()
    print("✅ Armed")


async def main():
    drone = System()
    await drone.connect(system_address="udpin://127.0.0.1:14540")

    await wait_for_connection(drone)
    await wait_for_position_lock(drone)

    # -------- ARM --------
    await arm_if_needed(drone)

    # -------- TAKEOFF --------
    print(f"⬆️ Taking off to {TAKEOFF_ALT} m...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT)
    await drone.action.takeoff()

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= TAKEOFF_ALT - 0.5:
            print("✅ Takeoff altitude reached")
            break

    # -------- ENTER OFFBOARD --------
    print("📡 Initializing Offboard with position hold...")
    for _ in range(15):
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -TAKEOFF_ALT, 0.0)
        )
        await asyncio.sleep(OFFBOARD_RATE)

    try:
        await drone.offboard.start()
        print("✅ Offboard started")
    except OffboardError as e:
        print(f"❌ Offboard failed: {e}")
        return

    # -------- CLIMB --------
    print(f"⬆️ Climbing to {TARGET_ALT} m")
    while True:
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, CLIMB_RATE, 0.0)
        )

        async for pos in drone.telemetry.position():
            print(f"📏 Altitude: {pos.relative_altitude_m:.2f} m")
            if pos.relative_altitude_m >= TARGET_ALT - 0.5:
                break
            break

        if pos.relative_altitude_m >= TARGET_ALT - 0.5:
            break

        await asyncio.sleep(OFFBOARD_RATE)

    # -------- HOLD --------
    print("🛑 Holding altitude")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(2)

    # -------- ATTITUDE TEST --------
    print("🧭 Attitude control test")
    pitch_rad = math.radians(PITCH_DEG)

    start_time = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start_time < ATTITUDE_DURATION:
        await drone.offboard.set_attitude(
            Attitude(
                0.0,            # roll (rad)
                pitch_rad,      # pitch (rad)
                0.0,            # yaw (rad)
                THRUST          # thrust (0–1)
            )
        )
        await asyncio.sleep(OFFBOARD_RATE)

    # -------- NEUTRAL --------
    print("🛑 Returning to neutral attitude")
    for _ in range(20):
        await drone.offboard.set_attitude(
            Attitude(0.0, 0.0, 0.0, THRUST)
        )
        await asyncio.sleep(OFFBOARD_RATE)

    print("✅ Attitude test complete")
    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(main())
