import asyncio
import csv
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

# ================= CONFIG =================
TAKEOFF_ALT = 10.0
DIVE_ENTRY_ALT = 40.0
ABORT_ALT = 4.0

TAKEOFF_SPEED = 10.0
FORWARD_PRE_DIVE = 8.0     # build pitch
FORWARD_DIVE = 10.0        # dive forward speed
DIVE_DOWN_VEL = 8.0        # vertical dive speed

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
    print("Waiting for vehicle to become armable...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("Vehicle is armable")
            return
        await asyncio.sleep(0.2)


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
    await drone.param.set_param_float("MPC_TKO_SPEED", TAKEOFF_SPEED)
    await drone.param.set_param_float("MPC_Z_VEL_MAX_UP", TAKEOFF_SPEED)
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", DIVE_DOWN_VEL)
    await drone.param.set_param_float("MPC_XY_VEL_MAX", 25.0)
    await asyncio.sleep(1.0)

    # -------- ARM & TAKEOFF --------
    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT)
    await drone.action.takeoff()

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= TAKEOFF_ALT - 0.5:
            break

    # -------- TELEMETRY --------
    pos_task = asyncio.create_task(position_listener(drone))
    vel_task = asyncio.create_task(velocity_listener(drone))

    while latest_pos is None or latest_vel is None:
        await asyncio.sleep(0.1)

    # -------- OFFBOARD --------
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print("Offboard failed:", e)
        return

    # -------- CLIMB TO ENTRY ALT --------
    print("Climbing to dive entry altitude...")
    while latest_pos.relative_altitude_m < DIVE_ENTRY_ALT - 0.5:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(0.0, 0.0, -3.0, 0.0)
        )
        await asyncio.sleep(CONTROL_DT)

    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await asyncio.sleep(1.0)

    print(f"Starting dive at altitude: {latest_pos.relative_altitude_m:.1f} m")

    # -------- PRE-DIVE (BUILD PITCH, HOLD ALT) --------
    print("Building forward speed (nose down)...")
    start = asyncio.get_event_loop().time()
    while asyncio.get_event_loop().time() - start < 1.5:
        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(FORWARD_PRE_DIVE, 0.0, -0.5, 0.0)
        )
        await asyncio.sleep(CONTROL_DT)

    # -------- SAFETY CHECK --------
    if latest_pos.relative_altitude_m < 10.0:
        print("Too low to dive safely, aborting")
        return

    # -------- TRUE DIVE --------
    print("INITIATING TRUE DIVE")
    log = []

    while True:
        alt = latest_pos.relative_altitude_m
        v_down = latest_vel.down_m_s

        await drone.offboard.set_velocity_body(
            VelocityBodyYawspeed(FORWARD_DIVE, 0.0, DIVE_DOWN_VEL, 0.0)
        )

        log.append({
            "time": datetime.now().isoformat(),
            "altitude_m": round(alt, 2),
            "v_down_m_s": round(v_down, 2)
        })

        print(f"ALT {alt:5.1f} m | Vdown {v_down:5.1f} m/s", end="\r")

        if alt <= ABORT_ALT:
            print("\nABORT @ 4 m")
            break

        await asyncio.sleep(CONTROL_DT)

    # -------- EXIT --------
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
    await drone.action.hold()

    stop_tasks = True
    pos_task.cancel()
    vel_task.cancel()

    if log:
        filename = f"real_dive_{datetime.now().strftime('%H%M%S')}.csv"
        with open(filename, "w") as f:
            writer = csv.DictWriter(f, fieldnames=log[0].keys())
            writer.writeheader()
            writer.writerows(log)

    print("Mission complete")


if __name__ == "__main__":
    asyncio.run(run())
