#!/usr/bin/env python3
"""
PX4 MAVSDK – PURE BALLISTIC DIVE
- Velocity-only OFFBOARD control
- LOCAL NED frame
- No mid-course corrections
"""

import asyncio
import math
import time
from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    VelocityNedYaw
)

# ---------------- USER CONFIG ---------------- #

TAKEOFF_ALT = 40.0        # meters AGL
CMD_RATE = 20.0           # Hz

CRUISE_SPEED = 8.0        # m/s
CRUISE_KP = 0.3
ARRIVE_TOL = 1.0          # meters

HOLD_OFFSET = 10.0        # meters before target

# Ballistic dive
DIVE_SPEED = 25.0         # horizontal m/s
DIVE_VZ = 12.0            # down m/s (NED +Z)
DIVE_MIN_ALT = 2.0        # meters
DIVE_TIMEOUT = 20.0       # sec


# --------------------------------------------------------
#                   UTILS
# --------------------------------------------------------

async def get_local_position(drone):
    async for pos in drone.telemetry.position_velocity_ned():
        return pos.position


async def get_altitude(drone):
    async for pos in drone.telemetry.position():
        return pos.relative_altitude_m


# --------------------------------------------------------
#              ARM AND TAKEOFF
# --------------------------------------------------------

async def arm_and_takeoff(drone):
    print("[ARM] Waiting for vehicle...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    print("[ARM] Arming")
    await drone.action.arm()
    await drone.action.takeoff()

    while True:
        alt = await get_altitude(drone)
        print(f"[TAKEOFF] Altitude: {alt:.1f} m")
        if alt >= TAKEOFF_ALT * 0.95:
            break
        await asyncio.sleep(0.5)

    print("[TAKEOFF] Reached target altitude")


# --------------------------------------------------------
#            CRUISE TO POINT (NED)
# --------------------------------------------------------

async def cruise_to_point(drone, Nt, Et, Dt):
    print(f"[CRUISE] Target N:{Nt:.1f} E:{Et:.1f} D:{Dt:.1f}")

    dt = 1.0 / CMD_RATE
    t0 = time.time()

    while True:
        pos = await get_local_position(drone)
        dN = Nt - pos.north_m
        dE = Et - pos.east_m
        dD = Dt - pos.down_m

        dist = math.sqrt(dN*dN + dE*dE + dD*dD)
        print(f"[CRUISE] dist={dist:.2f}")

        if dist <= ARRIVE_TOL:
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(0, 0, 0, 0)
            )
            return True

        if time.time() - t0 > 120:
            return False

        spd = min(CRUISE_SPEED, max(1.0, CRUISE_KP * dist))
        vx = dN / dist * spd
        vy = dE / dist * spd
        vz = dD / dist * spd

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(vx, vy, vz, 0)
        )

        await asyncio.sleep(dt)


# --------------------------------------------------------
#             YAW ALIGN TO TARGET
# --------------------------------------------------------

async def yaw_to_face_target(drone, targetN, targetE):
    pos = await get_local_position(drone)
    dN = targetN - pos.north_m
    dE = targetE - pos.east_m

    yaw_rad = math.atan2(dE, dN)
    yaw_deg = math.degrees(yaw_rad)

    print(f"[YAW] Aligning to {yaw_deg:.1f}°")

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0, 0, 0, yaw_deg)
    )

    await asyncio.sleep(1.0)
    return yaw_rad


# --------------------------------------------------------
#              PURE BALLISTIC DIVE
# --------------------------------------------------------

async def ballistic_dive(drone, yaw_rad):
    print("[DIVE] PURE BALLISTIC – NO CORRECTIONS")

    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)

    vx = cos_y * DIVE_SPEED
    vy = sin_y * DIVE_SPEED
    vz = DIVE_VZ

    dt = 1.0 / CMD_RATE
    t0 = time.time()

    while True:
        alt = await get_altitude(drone)
        print(f"[DIVE] Alt={alt:.2f}")

        if alt <= DIVE_MIN_ALT:
            print("[DIVE] Minimum altitude reached")
            return True

        if time.time() - t0 > DIVE_TIMEOUT:
            print("[DIVE] Timeout")
            return False

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(vx, vy, vz, math.degrees(yaw_rad))
        )

        await asyncio.sleep(dt)


# --------------------------------------------------------
#                        MAIN
# --------------------------------------------------------

async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    await arm_and_takeoff(drone)

    # Start offboard
    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0, 0, 0, 0)
    )

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print("Offboard failed:", e)
        return

    pos = await get_local_position(drone)
    startN, startE, startD = pos.north_m, pos.east_m, pos.down_m

    # Example target
    targetN = startN - 40
    targetE = startE - 80
    targetD = startD

    vecN = targetN - startN
    vecE = targetE - startE
    L = math.sqrt(vecN*vecN + vecE*vecE)

    holdN = targetN - (vecN/L) * HOLD_OFFSET
    holdE = targetE - (vecE/L) * HOLD_OFFSET
    holdD = startD

    ok = await cruise_to_point(drone, holdN, holdE, holdD)
    if not ok:
        print("Cruise failed")
        return

    yaw_rad = await yaw_to_face_target(drone, targetN, targetE)

    await ballistic_dive(drone, yaw_rad)

    print("[POST] Landing")
    await drone.offboard.stop()
    await drone.action.land()

    await asyncio.sleep(5)


if __name__ == "__main__":
    asyncio.run(main())
