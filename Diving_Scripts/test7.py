import asyncio
import csv
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# ================= CONFIG =================
TAKEOFF_ALT = 10.0
DIVE_ENTRY_ALT = 40.0
ABORT_ALT = 8.0

FORWARD_PRE_DIVE = 6.0
FORWARD_DIVE = 8.0
DIVE_DOWN_VEL = 8.0
CONTROL_DT = 0.05

latest_pos = None
latest_vel = None
stop_tasks = False


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


async def run():
    global latest_pos, latest_vel, stop_tasks

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    await wait_until_armable(drone)

    # Safety params
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", DIVE_DOWN_VEL)
    await drone.param.set_param_float("MPC_XY_VEL_MAX", 20.0)
    await asyncio.sleep(1.0)

    # Arm & takeoff
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT)
    await drone.action.takeoff()

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= TAKEOFF_ALT - 0.5:
            break

    pos_task = asyncio.create_task(position_listener(drone))
    vel_task = asyncio.create_task(velocity_listener(drone))

    while latest_pos is None or latest_vel is None:
        await asyncio.sleep(0.1)

    # Offboard
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    await drone.offboard.start()

    # Climb
    while latest_pos.relative_altitude_m < DIVE_ENTRY_ALT - 0.5:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0, 0, -3.0, 0)
        )
        await asyncio.sleep(CONTROL_DT)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    await asyncio.sleep(1.0)

    # Pre-dive (pitch build)
    start = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start < 1.0:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(FORWARD_PRE_DIVE, 0, 0, 0)
        )
        await asyncio.sleep(CONTROL_DT)

    # Dive
    log = []
    while True:
        alt = latest_pos.relative_altitude_m
        vdown = latest_vel.down_m_s

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(FORWARD_DIVE, 0, DIVE_DOWN_VEL, 0)
        )

        log.append({
            "time": datetime.now().isoformat(),
            "altitude": round(alt, 2),
            "v_down": round(vdown, 2)
        })

        print(f"ALT {alt:5.1f} | Vdown {vdown:5.1f}", end="\r")

        if alt <= ABORT_ALT:
            break

        await asyncio.sleep(CONTROL_DT)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
    await drone.action.hold()

    stop_tasks = True
    pos_task.cancel()
    vel_task.cancel()

    with open(f"dive_log_{datetime.now().strftime('%H%M%S')}.csv", "w") as f:
        writer = csv.DictWriter(f, fieldnames=log[0].keys())
        writer.writeheader()
        writer.writerows(log)

    print("\nMission complete")


if __name__ == "__main__":
    asyncio.run(run())
