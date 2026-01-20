import asyncio
import math
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import OffboardError, Attitude

# ================= CONFIG =================
DIVE_ENTRY_ALT = 40.0     # meters
ABORT_ALT = 3.0           # stop condition
CONTROL_DT = 0.05

# Dive geometry
ENTRY_PITCH_DEG = -10.0   # gentle commit
DIVE_PITCH_DEG = -70.0    # ballistic dive angle
ENTRY_TIME = 0.8          # seconds to ramp
DIVE_THRUST = 0.55        # keep motors loaded (don’t cut)

latest_pos = None
stop_tasks = False


# ================= TELEMETRY =================
async def position_listener(drone):
    global latest_pos, stop_tasks
    async for pos in drone.telemetry.position():
        if stop_tasks:
            break
        latest_pos = pos


async def wait_until_armable(drone):
    async for health in drone.telemetry.health():
        if health.is_armable:
            return
        await asyncio.sleep(0.2)


# ================= ATTITUDE HELPERS =================
async def pitch_ramp(drone, start_deg, end_deg, duration, thrust):
    steps = max(1, int(duration / CONTROL_DT))
    for i in range(steps):
        pitch = start_deg + (end_deg - start_deg) * (i / steps)
        await drone.offboard.set_attitude(
            Attitude(
                roll_deg=0.0,
                pitch_deg=pitch,   # NEGATIVE = nose down
                yaw_deg=0.0,
                thrust_value=thrust
            )
        )
        await asyncio.sleep(CONTROL_DT)


# ================= MAIN =================
async def run():
    global latest_pos, stop_tasks

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected")
            break

    await wait_until_armable(drone)

    # Arm
    await drone.action.arm()

    # Telemetry
    pos_task = asyncio.create_task(position_listener(drone))
    while latest_pos is None:
        await asyncio.sleep(0.1)

    # OFFBOARD warmup (required)
    await drone.offboard.set_attitude(
        Attitude(0.0, 0.0, 0.0, 0.6)
    )

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print("Offboard failed:", e)
        return

    # ======================================================
    # 1️⃣ CLIMB STRAIGHT TO ENTRY ALT (STABLE)
    # ======================================================
    print("Climbing to entry altitude...")
    while latest_pos.relative_altitude_m < DIVE_ENTRY_ALT:
        await drone.offboard.set_attitude(
            Attitude(
                roll_deg=0.0,
                pitch_deg=0.0,
                yaw_deg=0.0,
                thrust_value=0.65
            )
        )
        await asyncio.sleep(CONTROL_DT)

    # Small pause
    await asyncio.sleep(0.5)

    # ======================================================
    # 2️⃣ COMMIT PHASE (NO HESITATION)
    # ======================================================
    print("Committing to dive...")
    await pitch_ramp(
        drone,
        start_deg=ENTRY_PITCH_DEG,
        end_deg=DIVE_PITCH_DEG,
        duration=ENTRY_TIME,
        thrust=DIVE_THRUST
    )

    # ======================================================
    # 3️⃣ BALLISTIC DIVE (HOLD GEOMETRY)
    # ======================================================
    print("BALLISTIC DIVE (NO RECOVERY)")
    while True:
        alt = latest_pos.relative_altitude_m

        await drone.offboard.set_attitude(
            Attitude(
                roll_deg=0.0,
                pitch_deg=DIVE_PITCH_DEG,
                yaw_deg=0.0,
                thrust_value=DIVE_THRUST
            )
        )

        print(f"ALT {alt:5.1f}", end="\r")

        if alt <= ABORT_ALT:
            print("\nIMPACT / END")
            break

        await asyncio.sleep(CONTROL_DT)

    # ======================================================
    # 4️⃣ END (SIM ONLY)
    # ======================================================
    await drone.offboard.stop()
    stop_tasks = True
    pos_task.cancel()


if __name__ == "__main__":
    asyncio.run(run())
