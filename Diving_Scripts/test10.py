import asyncio
import csv
import math
from datetime import datetime
from mavsdk import System
# We use VelocityNedYaw for the dive to fix the twitching
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed, VelocityNedYaw
import os

# ================= CONFIG =================
TAKEOFF_ALT = 0.0
DIVE_ENTRY_ALT = 40.0
ABORT_ALT = 4.0

ASCENT_SPEED = 8.0          # total ascent speed (m/s)
ASCENT_ANGLE_DEG = 45.0     # can be changed dynamically

CRUISE_FORWARD = 6.0        # target acquisition
PRE_DIVE_FORWARD = 10.0     # force pitch BEFORE dive
DIVE_FORWARD = 12.0
DIVE_DOWN = 8.0

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


# ================= DEBUGGING FUNCTION =================
async def wait_until_armable(drone):
    print("Waiting for drone to become armable...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("\nDrone is armable!")
            return
        
        # This will print the status so you know what is missing
        print(f"  Status: Global_Pos={health.is_global_position_ok}, "
              f"Home_Pos={health.is_home_position_ok}, "
              f"Armable={health.is_armable}   ", end="\r")
        
        await asyncio.sleep(1.0)


# ================= MAIN =================
async def run():
    global latest_pos, latest_vel, stop_tasks

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    # This will now print why it's stuck!
    await wait_until_armable(drone)

    # -------- PX4 LIMITS (CRITICAL) --------
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

    # -------- OFFBOARD --------
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print("Offboard failed:", e)
        return

    # ======================================================
    # 1️⃣ VECTOR ASCENT (NOT STRAIGHT UP)
    # ======================================================
    print("Vector ascent...")

    theta = math.radians(ASCENT_ANGLE_DEG)
    ascent_forward = ASCENT_SPEED * math.cos(theta)
    ascent_up = -ASCENT_SPEED * math.sin(theta)

    while latest_pos.relative_altitude_m < DIVE_ENTRY_ALT - 0.5:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(
                ascent_forward,
                0.0,
                ascent_up,
                0.0
            )
        )
        await asyncio.sleep(CONTROL_DT)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    await asyncio.sleep(1.0)

    # ======================================================
    # 2️⃣ CRUISE / TARGET ACQUISITION
    # ======================================================
    print("Target acquisition at 40 m...")
    start = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start < 7.0:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(CRUISE_FORWARD, 0, 0, 0)
        )
        await asyncio.sleep(CONTROL_DT)

    # ======================================================
    # 3️⃣ PRE-DIVE FORWARD LOCK
    # ======================================================
    print("Locking forward vector before dive...")
    start = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start < 1.5:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(PRE_DIVE_FORWARD, 0, -0.5, 0)
        )
        await asyncio.sleep(CONTROL_DT)

    # ======================================================
    # 4️⃣ TRUE DIVE (NED FRAME)
    # ======================================================
    print("TRUE DIVE INITIATED (NED FRAME)")
    log = []

    while True:
        alt = latest_pos.relative_altitude_m
        vdown = latest_vel.down_m_s

        # Using NED Frame to prevent twitching/instability
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(
                DIVE_FORWARD,   # North (m/s)
                0.0,            # East (m/s)
                DIVE_DOWN,      # Down (m/s)
                0.0             # Yaw (deg)
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

    # -------- EXIT --------
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    await drone.action.hold()

    stop_tasks = True
    pos_task.cancel()
    vel_task.cancel()

    LOG_DIR = "logs"
    os.makedirs(LOG_DIR, exist_ok=True)

    filename = os.path.join(
        LOG_DIR,
        f"mission_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=log[0].keys())
        writer.writeheader()
        writer.writerows(log)

    print(f"Log saved to {filename}")


if __name__ == "__main__":
    asyncio.run(run())