import asyncio
import csv
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

# ================= CONFIG =================
DIVE_ENTRY_ALT = 40.0
MIN_ABORT_ALT = 8.0
MAX_DOWN_VEL = 8.0
FORWARD_VEL = 6.0
CONTROL_DT = 0.05

# ================= SHARED STATE =================
latest_pos = None
latest_vel = None


async def position_listener(drone):
    global latest_pos
    async for pos in drone.telemetry.position():
        latest_pos = pos


async def velocity_listener(drone):
    global latest_vel
    async for vel in drone.telemetry.velocity_ned():
        latest_vel = vel


async def wait_until_armable(drone):
    print("Waiting for vehicle to become armable...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("Vehicle is armable")
            return
        await asyncio.sleep(0.2)


async def run():
    global latest_pos, latest_vel

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    # ================= WAIT FOR ARMABLE =================
    await wait_until_armable(drone)

    # ================= SAFE PARAMS =================
    await drone.param.set_param_float("MPC_Z_VEL_MAX_UP", 5.0)
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", 8.0)
    await drone.param.set_param_float("MPC_XY_VEL_MAX", 12.0)
    await drone.param.set_param_float("MPC_ACC_DOWN_MAX", 6.0)
    await drone.param.set_param_float("MPC_JERK_AUTO", 6.0)

    # Give PX4 time to apply params
    await asyncio.sleep(1.0)

    # ================= ARM & TAKEOFF =================
    print("Arming...")
    await drone.action.arm()

    await drone.action.set_takeoff_altitude(10.0)
    print("Taking off...")
    await drone.action.takeoff()

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= 9.5:
            break

    # ================= START TELEMETRY =================
    asyncio.create_task(position_listener(drone))
    asyncio.create_task(velocity_listener(drone))

    while latest_pos is None or latest_vel is None:
        await asyncio.sleep(0.1)

    # ================= START OFFBOARD =================
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard failed: {e}")
        return

    # ================= CLIMB =================
    print("Ascending to dive entry altitude...")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, -3.0, 0))

    while latest_pos.relative_altitude_m < DIVE_ENTRY_ALT:
        await asyncio.sleep(0.1)

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    await asyncio.sleep(1.5)

    # ================= GUIDED DESCENT =================
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
            "timestamp": datetime.now().isoformat(),
            "altitude_m": round(alt, 2),
            "v_down_m_s": round(v_down, 2)
        })

        print(
            f"ALT {alt:5.1f} m | Vdown {v_down:5.1f} m/s",
            end="\r"
        )

        if alt <= MIN_ABORT_ALT:
            print("\nAbort altitude reached")
            break

        await asyncio.sleep(CONTROL_DT)

    # ================= EXIT =================
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    await drone.action.hold()

    # ================= SAVE LOG =================
    if log:
        filename = f"guided_descent_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(filename, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=log[0].keys())
            writer.writeheader()
            writer.writerows(log)
        print(f"Log saved to {filename}")

    print("Mission complete")


if __name__ == "__main__":
    asyncio.run(run())
import asyncio
import csv
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

# ================= CONFIG =================
DIVE_ENTRY_ALT = 40.0
MIN_ABORT_ALT = 8.0
MAX_DOWN_VEL = 8.0
FORWARD_VEL = 6.0
CONTROL_DT = 0.05

# ================= SHARED STATE =================
latest_pos = None
latest_vel = None


async def position_listener(drone):
    global latest_pos
    async for pos in drone.telemetry.position():
        latest_pos = pos


async def velocity_listener(drone):
    global latest_vel
    async for vel in drone.telemetry.velocity_ned():
        latest_vel = vel


async def wait_until_armable(drone):
    print("Waiting for vehicle to become armable...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("Vehicle is armable")
            return
        await asyncio.sleep(0.2)


async def run():
    global latest_pos, latest_vel

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    # ================= WAIT FOR ARMABLE =================
    await wait_until_armable(drone)

    # ================= SAFE PARAMS =================
    await drone.param.set_param_float("MPC_Z_VEL_MAX_UP", 5.0)
    await drone.param.set_param_float("MPC_Z_VEL_MAX_DN", 8.0)
    await drone.param.set_param_float("MPC_XY_VEL_MAX", 12.0)
    await drone.param.set_param_float("MPC_ACC_DOWN_MAX", 6.0)
    await drone.param.set_param_float("MPC_JERK_AUTO", 6.0)

    # Give PX4 time to apply params
    await asyncio.sleep(1.0)

    # ================= ARM & TAKEOFF =================
    print("Arming...")
    await drone.action.arm()

    await drone.action.set_takeoff_altitude(10.0)
    print("Taking off...")
    await drone.action.takeoff()

    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m >= 9.5:
            break

    # ================= START TELEMETRY =================
    asyncio.create_task(position_listener(drone))
    asyncio.create_task(velocity_listener(drone))

    while latest_pos is None or latest_vel is None:
        await asyncio.sleep(0.1)

    # ================= START OFFBOARD =================
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard failed: {e}")
        return

    # ================= CLIMB =================
    print("Ascending to dive entry altitude...")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, -3.0, 0))

    while latest_pos.relative_altitude_m < DIVE_ENTRY_ALT:
        await asyncio.sleep(0.1)

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    await asyncio.sleep(1.5)

    # ================= GUIDED DESCENT =================
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
            "timestamp": datetime.now().isoformat(),
            "altitude_m": round(alt, 2),
            "v_down_m_s": round(v_down, 2)
        })

        print(
            f"ALT {alt:5.1f} m | Vdown {v_down:5.1f} m/s",
            end="\r"
        )

        if alt <= MIN_ABORT_ALT:
            print("\nAbort altitude reached")
            break

        await asyncio.sleep(CONTROL_DT)

    # ================= EXIT =================
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
    await drone.action.hold()

    # ================= SAVE LOG =================
    if log:
        filename = f"guided_descent_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(filename, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=log[0].keys())
            writer.writeheader()
            writer.writerows(log)
        print(f"Log saved to {filename}")

    print("Mission complete")


if __name__ == "__main__":
    asyncio.run(run())
