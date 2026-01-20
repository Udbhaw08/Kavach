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

# ---------------- USER PARAMS ----------------
DIVE_SPEED = 5.0          # m/s downward
TERMINAL_ALTITUDE = 2.0   # meters
Kp_HORIZONTAL = 0.004
LOCK_STABILITY_TIME = 3.0
# --------------------------------------------

# ---------------- SYSTEM CONFIG --------------
TAKEOFF_ALT = 5.0
CRUISE_ALT = 70.0
CLIMB_RATE = -6.0         # up
OFFBOARD_RATE = 0.1       # 10 Hz
DIVE_PITCH_DEG = -35.0    # controlled dive angle
HOVER_THRUST = 0.6
DIVE_THRUST = 0.42
# --------------------------------------------


async def wait_for_connection(drone):
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected")
            return


async def wait_for_position_lock(drone):
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("✅ Position lock")
            return


async def arm_if_needed(drone):
    async for armed in drone.telemetry.armed():
        if armed:
            print("ℹ️ Already armed")
            return
        break
    print("🚁 Arming")
    await drone.action.arm()


async def main():
    drone = System()
    await drone.connect(system_address="udpin://127.0.0.1:14540")

    await wait_for_connection(drone)
    await wait_for_position_lock(drone)
    await arm_if_needed(drone)

    # -------- TAKEOFF --------
    print("⬆️ Takeoff")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT)
    await drone.action.takeoff()

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= TAKEOFF_ALT - 0.5:
            break

    # -------- OFFBOARD INIT --------
    for _ in range(15):
        await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -TAKEOFF_ALT, 0.0)
        )
        await asyncio.sleep(OFFBOARD_RATE)

    try:
        await drone.offboard.start()
        print("✅ Offboard")
    except OffboardError as e:
        print("❌ Offboard failed:", e)
        return

    # -------- CLIMB TO CRUISE ALT --------
    print("⬆️ Climbing to cruise altitude")
    while True:
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, CLIMB_RATE, 0.0)
        )

        async for pos in drone.telemetry.position():
            print(f"📏 Alt: {pos.relative_altitude_m:.1f} m")
            if pos.relative_altitude_m >= CRUISE_ALT - 0.5:
                break
            break

        if pos.relative_altitude_m >= CRUISE_ALT - 0.5:
            break

        await asyncio.sleep(OFFBOARD_RATE)

    # -------- STABILITY LOCK --------
    print(f"🔒 Stabilizing for {LOCK_STABILITY_TIME}s")
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
    )
    await asyncio.sleep(LOCK_STABILITY_TIME)

    # -------- CONTROLLED DIVE --------
    print("⬇️ Controlled dive started")
    pitch_rad = math.radians(DIVE_PITCH_DEG)

    while True:
        async for pos in drone.telemetry.position():
            altitude = pos.relative_altitude_m
            break

        if altitude <= TERMINAL_ALTITUDE:
            print("🛑 Terminal altitude reached")
            break

        # Vertical speed limiter
        down_speed = min(DIVE_SPEED, altitude)

        # Horizontal damping (simple P-control)
        north_correction = -Kp_HORIZONTAL * 0.0
        east_correction = -Kp_HORIZONTAL * 0.0

        await drone.offboard.set_attitude(
            Attitude(
                0.0,
                pitch_rad,
                0.0,
                DIVE_THRUST
            )
        )

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(
                north_correction,
                east_correction,
                down_speed,
                0.0
            )
        )

        print(f"⬇️ Altitude: {altitude:.2f} m")
        await asyncio.sleep(OFFBOARD_RATE)

    # -------- RECOVERY --------
    print("🛑 Recovering to neutral")
    for _ in range(30):
        await drone.offboard.set_attitude(
            Attitude(0.0, 0.0, 0.0, HOVER_THRUST)
        )
        await asyncio.sleep(OFFBOARD_RATE)

    print("✅ Dive sequence complete — holding")
    while True:
        await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(main())
