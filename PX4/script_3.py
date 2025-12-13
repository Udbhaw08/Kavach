#!/usr/bin/env python3
"""
PX4 MAVSDK – PURE BALLISTIC DIVE (FINAL, FIXED TAKEOFF)
- OFFBOARD velocity control ONLY
- Proper PX4 takeoff using velocity (NO action.takeoff)
- Console logs
"""
from mavsdk.offboard import VelocityBodyYawspeed
import asyncio
import math
import time
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

# ================= USER CONFIG ================= #

TAKEOFF_ALT = 40.0
TAKEOFF_VZ = -1.5        # m/s UP (NED negative)
CMD_RATE = 20.0

CRUISE_SPEED = 8.0
CRUISE_KP = 0.3
ARRIVE_TOL = 1.0
NAV_TIMEOUT = 120.0

HOLD_OFFSET = 10.0

DIVE_SPEED = 25.0
DIVE_VZ = 12.0
DIVE_MIN_ALT = 2.0
DIVE_TIMEOUT = 20.0

PHASE = "INIT"

# =================================================


# ---------------- TELEMETRY HELPERS ---------------- #

async def get_local_position(drone):
    async for pv in drone.telemetry.position_velocity_ned():
        return pv.position, pv.velocity


async def get_altitude(drone):
    async for pos in drone.telemetry.position():
        return pos.relative_altitude_m


# ---------------- BACKGROUND LOGGER ---------------- #

async def telemetry_logger(drone):
    global PHASE
    async for pv in drone.telemetry.position_velocity_ned():
        p = pv.position
        v = pv.velocity
        print(
            f"[{PHASE:<7}] "
            f"N={p.north_m:7.2f} "
            f"E={p.east_m:7.2f} "
            f"D={p.down_m:7.2f} | "
            f"Vn={v.north_m_s:6.2f} "
            f"Ve={v.east_m_s:6.2f} "
            f"Vd={v.down_m_s:6.2f}"
        )
        await asyncio.sleep(0.1)


# ---------------- ARM + OFFBOARD + TAKEOFF ---------------- #

async def arm_and_takeoff(drone):
    global PHASE

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[SYS] Connected")
            break

    PHASE = "HEALTH"
    print("[HEALTH] Waiting for local position...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            print("[HEALTH] OK")
            break
        await asyncio.sleep(0.5)

    PHASE = "OFF_PRE"
    for _ in range(15):
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                forward_m_s=25.0,   # aggressive forward
                right_m_s=0.0,
                down_m_s=15.0,      # TRUE dive
                yawspeed_deg_s=0.0
            )
        )
        await asyncio.sleep(0.1)

    PHASE = "OFFBRD"
    await drone.offboard.start()

    PHASE = "ARM"
    await drone.action.arm()
    print("[ARM] Armed")

    # ---------- VELOCITY TAKEOFF ----------
    PHASE = "TAKEOFF"
    print("[TAKEOFF] Velocity climb")

    while True:
        alt = await get_altitude(drone)
        print(f"[TAKEOFF] Altitude={alt:.2f} m")

        if alt >= TAKEOFF_ALT:
            print("[TAKEOFF] Reached target altitude")
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(0, 0, 0, 0)
            )
            break

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0, 0, TAKEOFF_VZ, 0)
        )

        await asyncio.sleep(1.0 / CMD_RATE)


# ---------------- CRUISE TO HOLD ---------------- #

async def cruise_to_point(drone, Nt, Et, Dt):
    global PHASE
    PHASE = "CRUISE"

    dt = 1.0 / CMD_RATE
    t0 = time.time()

    while True:
        pos, _ = await get_local_position(drone)

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

        if time.time() - t0 > NAV_TIMEOUT:
            return False

        spd = min(CRUISE_SPEED, max(1.0, CRUISE_KP * dist))
        vx = dN / dist * spd
        vy = dE / dist * spd
        vz = dD / dist * spd

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(vx, vy, vz, 0)
        )
        await asyncio.sleep(dt)


# ---------------- YAW ALIGN ---------------- #

async def yaw_to_face_target(drone, targetN, targetE):
    global PHASE
    PHASE = "ALIGN"

    pos, _ = await get_local_position(drone)
    yaw = math.atan2(targetE - pos.east_m, targetN - pos.north_m)
    yaw_deg = math.degrees(yaw)

    print(f"[ALIGN] Yaw={yaw_deg:.1f}°")

    await drone.offboard.set_velocity_ned(
        VelocityNedYaw(0, 0, 0, yaw_deg)
    )
    await asyncio.sleep(1.0)
    return yaw


# ---------------- BALLISTIC DIVE ---------------- #

async def ballistic_dive(drone, yaw):
    global PHASE
    PHASE = "DIVE"

    vx = math.cos(yaw) * DIVE_SPEED
    vy = math.sin(yaw) * DIVE_SPEED
    vz = DIVE_VZ

    print(f"[DIVE] vx={vx:.1f} vy={vy:.1f} vz={vz:.1f}")

    t0 = time.time()
    dt = 1.0 / CMD_RATE

    while True:
        alt = await get_altitude(drone)
        print(f"[DIVE] Alt={alt:.2f}")

        if alt <= DIVE_MIN_ALT:
            return True

        if time.time() - t0 > DIVE_TIMEOUT:
            return False

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(vx, vy, vz, math.degrees(yaw))
        )
        await asyncio.sleep(dt)


# ---------------- MAIN ---------------- #

async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    logger = asyncio.create_task(telemetry_logger(drone))

    await arm_and_takeoff(drone)

    pos, _ = await get_local_position(drone)
    startN, startE, startD = pos.north_m, pos.east_m, pos.down_m

    targetN = startN - 40
    targetE = startE - 80

    vecN = targetN - startN
    vecE = targetE - startE
    L = math.sqrt(vecN*vecN + vecE*vecE)

    holdN = targetN - (vecN / L) * HOLD_OFFSET
    holdE = targetE - (vecE / L) * HOLD_OFFSET

    await cruise_to_point(drone, holdN, holdE, startD)

    yaw = await yaw_to_face_target(drone, targetN, targetE)

    await ballistic_dive(drone, yaw)

    PHASE = "LAND"
    await drone.offboard.stop()
    await drone.action.land()

    await asyncio.sleep(5)
    logger.cancel()


if __name__ == "__main__":
    asyncio.run(main())
