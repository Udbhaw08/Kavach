import asyncio
import csv
import math
import os
from datetime import datetime

from mavsdk import System
from mavsdk.offboard import (
    OffboardError,
    VelocityBodyYawspeed,
    Attitude
)

# ================= CONFIG =================
TAKEOFF_ALT = 0.0
DIVE_ENTRY_ALT = 40.0
ABORT_ALT = 4.0

ASCENT_SPEED = 8.0
ASCENT_ANGLE_DEG = 45.0

CRUISE_FORWARD = 6.0
CONTROL_DT = 0.05

latest_pos = None
latest_vel = None
stop_tasks = False


# ================= TELEMETRY =================
async def position_listener(drone):
    global latest_pos, stop_tasks
    async for pos in drone.telemetry.position():
        if stop_tasks:
            break
        latest_pos = pos


async def velocity_listener(drone):
    global latest_vel, stop_tasks
    async for vel in drone.telemetry.velocity_ned():
        if stop_tasks:
            break
        latest_vel = vel


async def wait_until_armable(drone):
    async for health in drone.telemetry.health():
        if health.is_armable:
            return
        await asyncio.sleep(0.2)


# ================= ATTITUDE HELPERS =================
async def smooth_pitch_ramp(drone, start_deg, end_deg, duration):
    steps = int(duration / CONTROL_DT)
    for i in range(steps):
        pitch = start_deg + (end_deg - start_deg) * (i / steps)
        await drone.offboard.set_attitude(
            Attitude(
                roll_deg=0.0,
                pitch_deg=pitch,     # negative = nose down
                yaw_deg=0.0,
                thrust_value=0.6
            )
        )
        await asyncio.sleep(CONTROL_DT)


# ================= MAIN =================
async def run():
    global latest_pos, latest_vel, stop_tasks

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    await wait_until_armable(drone)

    # -------- PX4 LIMITS --------
    await drone.param.set_param_float("MPC_Z_VEL_MAX_UP", 8.0)
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", 8.0)
    await drone.param.set_param_float("MPC_XY_VEL_MAX", 25.0)
    await drone.param.set_param_float("MPC_TILTMAX_AIR", 60.0)
    await asyncio.sleep(1.0)

    # -------- ARM --------
    await drone.action.arm()

    # -------- TELEMETRY --------
    pos_task = asyncio.create_task(position_listener(drone))
    vel_task = asyncio.create_task(velocity_listener(drone))

    while latest_pos is None or latest_vel is None:
        await asyncio.sleep(0.1)

    # -------- OFFBOARD START --------
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print("Offboard failed:", e)
        return

    # ======================================================
    # 1️⃣ VECTOR ASCENT
    # ======================================================
    print("Vector ascent...")

    theta = math.radians(ASCENT_ANGLE_DEG)
    ascent_forward = ASCENT_SPEED * math.cos(theta)
    ascent_up = -ASCENT_SPEED * math.sin(theta)

    while latest_pos.relative_altitude_m < DIVE_ENTRY_ALT - 0.5:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(ascent_forward, 0.0, ascent_up, 0.0)
        )
        await asyncio.sleep(CONTROL_DT)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    await asyncio.sleep(1.0)

    # ======================================================
    # 2️⃣ DIVE ENTRY — SMOOTH ATTITUDE TRANSITION
    # ======================================================
    print("Smooth dive entry (attitude control)...")

    await smooth_pitch_ramp(
        drone,
        start_deg=-5.0,
        end_deg=-60.0,
        duration=1.2
    )

    # ======================================================
    # 3️⃣ TRUE ATTITUDE DIVE
    # ======================================================
    print("TRUE DIVE (ATTITUDE HOLD)")
    log = []

    while True:
        alt = latest_pos.relative_altitude_m
        vdown = latest_vel.down_m_s

        await drone.offboard.set_attitude(
            Attitude(
                roll_deg=0.0,
                pitch_deg=-60.0,
                yaw_deg=0.0,
                thrust_value=0.55
            )
        )

        log.append({
            "time": datetime.now().isoformat(),
            "altitude": round(alt, 2),
            "v_down": round(vdown, 2)
        })

        print(f"ALT {alt:5.1f} | Vdown {vdown:5.1f}", end="\r")

        if alt <= ABORT_ALT:
            print("\nABORT / IMPACT")
            break

        await asyncio.sleep(CONTROL_DT)

    # -------- CLEAN EXIT --------
    await drone.offboard.stop()
    await drone.action.hold()

    stop_tasks = True
    pos_task.cancel()
    vel_task.cancel()

    # -------- LOG --------
    os.makedirs("logs", exist_ok=True)
    filename = f"logs/mission_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=log[0].keys())
        writer.writeheader()
        writer.writerows(log)

    print(f"\nLog saved to {filename}")


if __name__ == "__main__":
    asyncio.run(run())
