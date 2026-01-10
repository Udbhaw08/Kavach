import asyncio
import csv
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

# ================= CONFIG =================
TAKEOFF_ALT = 10.0
DIVE_ENTRY_ALT = 40.0
MIN_ABORT_ALT = 8.0

MAX_DOWN_VEL = 8.0
FORWARD_VEL = 6.0
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
    print("Waiting for vehicle to become armable...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("Vehicle is armable")
            return
        await asyncio.sleep(0.2)


async def run():
    global latest_pos, latest_vel, stop_tasks

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    await wait_until_armable(drone)

    # -------- Safe params --------
    await drone.param.set_param_float("MPC_Z_VEL_MAX_UP", 5.0)
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", 8.0)
    await drone.param.set_param_float("MPC_XY_VEL_MAX", 12.0)
    await drone.param.set_param_float("MPC_ACC_DOWN_MAX", 6.0)
    await asyncio.sleep(1.0)

    # -------- Arm & takeoff --------
    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT)
    await drone.action.takeoff()

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= TAKEOFF_ALT - 0.5:
            break

    # -------- Start telemetry --------
    pos_task = asyncio.create_task(position_listener(drone))
    vel_task = asyncio.create_task(velocity_listener(drone))

    while latest_pos is None or latest_vel is None:
        await asyncio.sleep(0.1)

    # -------- Start offboard --------
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard failed: {e}")
        stop_tasks = True
        return

    # -------- Climb to entry --------
    print("Ascending to dive entry altitude...")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, -3.0, 0))

    while latest_pos.relative_altitude_m < DIVE_ENTRY_ALT:
        await asyncio.sleep(0.1)

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    await asyncio.sleep(1.5)

    # -------- Guided descent --------
    print("Starting guided descent")
    log = []

    while True:
        alt = latest_pos.relative_altitude_m
        v_down = latest_vel.down_m_s

        desired_down = min(
            MAX_DOWN_VEL,
            max(1.0, 0.4 * (alt - MIN_ABORT_ALT))
        )

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(
                north_m_s=FORWARD_VEL,
                east_m_s=0.0,
                down_m_s=desired_down,
                yaw_deg=0.0
            )
        )

        log.append({
            "time": datetime.now().isoformat(),
            "altitude_m": round(alt, 2),
            "v_down_m_s": round(v_down, 2)
        })

        print(f"ALT {alt:5.1f} m | Vdown {v_down:5.1f} m/s", end="\r")

        if alt <= MIN_ABORT_ALT:
            print("\nAbort altitude reached")
            break

        await asyncio.sleep(CONTROL_DT)
        

    # -------- Exit cleanly --------
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    await drone.action.hold()

    stop_tasks = True
    await asyncio.sleep(0.5)

    pos_task.cancel()
    vel_task.cancel()

    if log:
        filename = f"guided_descent_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(filename, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=log[0].keys())
            writer.writeheader()
            writer.writerows(log)
        print(f"Log saved to {filename}")

    print("Mission complete — exiting cleanly")


if __name__ == "__main__":
    asyncio.run(run())
