#!/usr/bin/env python3
"""
Top-Up Approach Mission (Multicopter behavior)
==============================================
Mission profile:
1. Connect + arm
2. Initial takeoff to ~10m
3. Aggressive stable ascent to lock altitude (default 30m)
4. Hover/lock for fixed duration (default 5s)
5. Attack airborne target relative to lock-point (+60m north by default)
6. Recovery via RTL (fallback HOLD)
"""

import argparse
import asyncio
import csv
import math
from collections import deque
from datetime import datetime
from pathlib import Path

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw


# ================= DEFAULT CONFIG =================
LOCK_ALTITUDE_M = 30.0
LOCK_HOLD_S = 5.0
# 150m at 12m/s gives ~12.5s attack duration (within requested 10-15s window).
TARGET_REL_NORTH_M = 150.0
TARGET_REL_EAST_M = 0.0
TARGET_REL_UP_M = 10.0
ASCENT_MIN_M_S = 12.0
ASCENT_CMD_M_S = 12.5
ATTACK_SPEED_M_S = 12.0
HIT_RADIUS_M = 2.5
ATTACK_TIMEOUT_S = 45.0
MIN_SAFE_ALT_M = 3.0
MAX_TILT_DEG = 60.0
CONTROL_DT_S = 0.05
INITIAL_TAKEOFF_ALT_M = 10.0


# ================= GLOBAL STATE =================
latest_pos = None
latest_vel = None
latest_att = None
latest_pv_ned = None
stop_tasks = False
mission_log = []


def parse_args():
    parser = argparse.ArgumentParser(
        description="Top-Up Approach mission for PX4 SITL multicopter control."
    )
    parser.add_argument("--system-address", default="udp://:14540")
    parser.add_argument("--lock-alt", type=float, default=LOCK_ALTITUDE_M)
    parser.add_argument("--hold-sec", type=float, default=LOCK_HOLD_S)
    parser.add_argument("--target-north", type=float, default=TARGET_REL_NORTH_M)
    parser.add_argument("--target-east", type=float, default=TARGET_REL_EAST_M)
    parser.add_argument("--target-up", type=float, default=TARGET_REL_UP_M)
    parser.add_argument("--ascent-min", type=float, default=ASCENT_MIN_M_S)
    parser.add_argument("--ascent-cmd", type=float, default=ASCENT_CMD_M_S)
    parser.add_argument("--attack-speed", type=float, default=ATTACK_SPEED_M_S)
    parser.add_argument("--hit-radius", type=float, default=HIT_RADIUS_M)
    return parser.parse_args()


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


async def attitude_listener(drone):
    global latest_att, stop_tasks
    async for att in drone.telemetry.attitude_euler():
        if stop_tasks:
            break
        latest_att = att


async def position_velocity_ned_listener(drone):
    global latest_pv_ned, stop_tasks
    async for pv_ned in drone.telemetry.position_velocity_ned():
        if stop_tasks:
            break
        latest_pv_ned = pv_ned


# ================= LOGGING =================
def log_entry(
    phase,
    altitude_m=None,
    climb_rate_m_s=0.0,
    climb_rate_avg_m_s=0.0,
    dist_to_target_m=0.0,
    hit_flag=0,
    abort_reason="",
):
    if altitude_m is None and latest_pos is not None:
        altitude_m = latest_pos.relative_altitude_m

    vn = latest_vel.north_m_s if latest_vel else 0.0
    ve = latest_vel.east_m_s if latest_vel else 0.0
    vd = latest_vel.down_m_s if latest_vel else 0.0

    mission_log.append(
        {
            "timestamp": datetime.now().isoformat(),
            "phase": phase,
            "altitude_m": round(float(altitude_m or 0.0), 3),
            "vn_m_s": round(vn, 3),
            "ve_m_s": round(ve, 3),
            "vd_m_s": round(vd, 3),
            "climb_rate_m_s": round(float(climb_rate_m_s), 3),
            "climb_rate_avg_m_s": round(float(climb_rate_avg_m_s), 3),
            "dist_to_target_m": round(float(dist_to_target_m), 3),
            "hit_flag": int(hit_flag),
            "abort_reason": abort_reason,
        }
    )


def save_log():
    if not mission_log:
        return None

    logs_dir = Path(__file__).resolve().parents[1] / "Diving_Scripts" / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    filename = logs_dir / f"mission_top_up_approach_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    with filename.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=mission_log[0].keys())
        writer.writeheader()
        writer.writerows(mission_log)

    return filename


# ================= SAFETY =================
def safety_status():
    if latest_pos is not None and latest_pos.relative_altitude_m < MIN_SAFE_ALT_M:
        return False, f"below minimum safe altitude ({latest_pos.relative_altitude_m:.2f}m)"

    if latest_att is not None:
        roll = abs(latest_att.roll_deg)
        pitch = abs(latest_att.pitch_deg)
        if max(roll, pitch) > MAX_TILT_DEG:
            return False, f"excessive tilt (roll={roll:.1f}, pitch={pitch:.1f})"

    return True, ""


# ================= PARAMETER MANAGEMENT =================
async def configure_aggressive_params(drone):
    params_to_set = {
        "MPC_Z_VEL_MAX_UP": 15.0,
        "MPC_Z_V_AUTO_UP": 15.0,
        "MPC_TKO_SPEED": 12.0,
        "MPC_ACC_UP_MAX": 12.0,
        "MPC_JERK_MAX": 35.0,
        "MPC_XY_VEL_MAX": 14.0,
        "MPC_TILTMAX_AIR": 50.0,
    }
    backup = {}
    print("Configuring aggressive ascent parameters...")
    for name, value in params_to_set.items():
        try:
            old_val = await drone.param.get_param_float(name)
            backup[name] = old_val
            await drone.param.set_param_float(name, value)
            print(f"  {name}: {old_val:.2f} -> {value:.2f}")
        except Exception as exc:
            print(f"  {name}: skipped ({exc})")
    await asyncio.sleep(0.8)
    return backup


async def restore_params(drone, backup):
    if not backup:
        return
    print("Restoring mission parameters...")
    for name, old_val in backup.items():
        try:
            await drone.param.set_param_float(name, old_val)
            print(f"  {name}: restored -> {old_val:.2f}")
        except Exception as exc:
            print(f"  {name}: restore failed ({exc})")


# ================= UTILITIES =================
async def wait_connection(drone):
    print("Waiting for connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected to PX4.")
            return


async def wait_armable(drone):
    print("Waiting for vehicle armable...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("Vehicle is armable.")
            return
        await asyncio.sleep(0.2)


async def wait_telemetry_ready():
    while any(v is None for v in [latest_pos, latest_vel, latest_att, latest_pv_ned]):
        await asyncio.sleep(0.1)


async def offboard_warmup(drone):
    for _ in range(12):
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.05)


# ================= PHASES =================
async def phase_connect_and_arm(drone):
    print("Arming...")
    await drone.action.arm()
    print("Armed.")


async def phase_takeoff_initial(drone):
    print(f"Initial takeoff to ~{INITIAL_TAKEOFF_ALT_M:.1f}m...")
    await drone.action.set_takeoff_altitude(INITIAL_TAKEOFF_ALT_M)
    await drone.action.takeoff()

    while latest_pos is None or latest_pos.relative_altitude_m < INITIAL_TAKEOFF_ALT_M - 1.0:
        if latest_pos is not None:
            log_entry("TAKEOFF", altitude_m=latest_pos.relative_altitude_m)
        await asyncio.sleep(0.1)

    print(f"Takeoff complete at {latest_pos.relative_altitude_m:.1f}m.")


async def phase_ascent_fast_stable(drone, lock_alt_m, ascent_min_m_s, ascent_cmd_m_s):
    print("\nPHASE: AGGRESSIVE ASCENT")
    print(f"Target lock altitude: {lock_alt_m:.1f}m")
    print(f"Ascent command: {ascent_cmd_m_s:.1f} m/s, expected minimum: {ascent_min_m_s:.1f} m/s")

    # Ensure command stays strictly above minimum intent if user passes bad values.
    ascent_cmd_m_s = max(ascent_cmd_m_s, ascent_min_m_s + 0.2)

    rolling = deque(maxlen=30)
    low_rate_duration_s = 0.0
    low_rate_flagged = False
    peak_climb_m_s = 0.0
    last_print_t = 0.0
    loop = asyncio.get_event_loop()

    while True:
        safe, reason = safety_status()
        if not safe:
            log_entry("ASCENT", abort_reason=f"safety_abort:{reason}")
            raise RuntimeError(f"Safety abort in ascent: {reason}")

        alt = latest_pos.relative_altitude_m
        climb_rate = -latest_vel.down_m_s
        rolling.append(climb_rate)
        climb_avg = sum(rolling) / len(rolling)
        peak_climb_m_s = max(peak_climb_m_s, climb_rate)

        if alt < lock_alt_m - 2.0 and climb_avg < ascent_min_m_s:
            low_rate_duration_s += CONTROL_DT_S
            if low_rate_duration_s > 1.5 and not low_rate_flagged:
                low_rate_flagged = True
                print(
                    f"WARNING: sustained climb average below {ascent_min_m_s:.1f} m/s "
                    f"(avg={climb_avg:.2f})."
                )
        else:
            low_rate_duration_s = 0.0

        log_entry(
            "ASCENT",
            altitude_m=alt,
            climb_rate_m_s=climb_rate,
            climb_rate_avg_m_s=climb_avg,
        )

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(
                north_m_s=0.0,
                east_m_s=0.0,
                down_m_s=-ascent_cmd_m_s,
                yaw_deg=latest_att.yaw_deg if latest_att else 0.0,
            )
        )

        now_t = loop.time()
        if now_t - last_print_t >= 0.2:
            print(
                f"ALT {alt:5.1f}m | ascent(inst) {climb_rate:5.2f} m/s | "
                f"ascent(avg) {climb_avg:5.2f} m/s | peak {peak_climb_m_s:5.2f} m/s",
                end="\r",
            )
            last_print_t = now_t

        if alt >= lock_alt_m:
            print(f"\nReached lock altitude {alt:.1f}m.")
            print(
                "Ascent speed summary: "
                f"instant={climb_rate:.2f} m/s, avg={climb_avg:.2f} m/s, peak={peak_climb_m_s:.2f} m/s"
            )
            break

        await asyncio.sleep(CONTROL_DT_S)

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, latest_att.yaw_deg if latest_att else 0.0))


async def phase_lock_hover(drone, hold_sec):
    print("\nPHASE: LOCK HOVER")
    print(f"Holding for {hold_sec:.1f}s...")
    start = asyncio.get_event_loop().time()

    while True:
        safe, reason = safety_status()
        if not safe:
            log_entry("LOCK", abort_reason=f"safety_abort:{reason}")
            raise RuntimeError(f"Safety abort in lock hover: {reason}")

        elapsed = asyncio.get_event_loop().time() - start
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, latest_att.yaw_deg if latest_att else 0.0))

        log_entry("LOCK", altitude_m=latest_pos.relative_altitude_m)

        print(
            f"LOCK hover: {max(0.0, hold_sec-elapsed):4.1f}s left | ALT {latest_pos.relative_altitude_m:5.1f}m",
            end="\r",
        )
        if elapsed >= hold_sec:
            print("\nLock hover complete.")
            break
        await asyncio.sleep(CONTROL_DT_S)

    lock_n = latest_pv_ned.position.north_m
    lock_e = latest_pv_ned.position.east_m
    lock_d = latest_pv_ned.position.down_m
    return lock_n, lock_e, lock_d


async def phase_attack_air_target(
    drone,
    lock_n,
    lock_e,
    lock_d,
    target_rel_north,
    target_rel_east,
    target_rel_up,
    hit_radius,
    attack_speed_m_s,
):
    print("\nPHASE: AIRBORNE TARGET ATTACK")
    target_n = lock_n + target_rel_north
    target_e = lock_e + target_rel_east
    target_d = lock_d - target_rel_up
    yaw_cmd = latest_att.yaw_deg if latest_att else 0.0

    print(f"Target NED -> N:{target_n:.2f}, E:{target_e:.2f}, D:{target_d:.2f}")
    print(f"Hit radius: {hit_radius:.2f}m")

    start = asyncio.get_event_loop().time()
    while True:
        safe, reason = safety_status()
        if not safe:
            log_entry("ATTACK", abort_reason=f"safety_abort:{reason}")
            raise RuntimeError(f"Safety abort in attack: {reason}")

        curr_n = latest_pv_ned.position.north_m
        curr_e = latest_pv_ned.position.east_m
        curr_d = latest_pv_ned.position.down_m

        dist = math.sqrt((curr_n - target_n) ** 2 + (curr_e - target_e) ** 2 + (curr_d - target_d) ** 2)

        # Velocity vector toward target at commanded attack speed.
        # Slow down only in terminal region to avoid overshoot.
        speed_cmd = attack_speed_m_s if dist > 12.0 else max(2.0, dist)
        if dist > 0.01:
            vn = speed_cmd * (target_n - curr_n) / dist
            ve = speed_cmd * (target_e - curr_e) / dist
            vd = speed_cmd * (target_d - curr_d) / dist
        else:
            vn = 0.0
            ve = 0.0
            vd = 0.0

        await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, yaw_cmd))
        log_entry("ATTACK", altitude_m=latest_pos.relative_altitude_m, dist_to_target_m=dist)

        print(f"Distance to target: {dist:6.2f}m | ALT {latest_pos.relative_altitude_m:5.1f}m", end="\r")

        if dist <= hit_radius:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, yaw_cmd))
            log_entry("ATTACK", altitude_m=latest_pos.relative_altitude_m, dist_to_target_m=dist, hit_flag=1)
            print(f"\nTARGET INTERCEPTED at {dist:.2f}m.")
            return True

        if asyncio.get_event_loop().time() - start > ATTACK_TIMEOUT_S:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, yaw_cmd))
            log_entry("ATTACK", altitude_m=latest_pos.relative_altitude_m, dist_to_target_m=dist, abort_reason="attack_timeout")
            print("\nAttack timeout reached.")
            return False

        await asyncio.sleep(CONTROL_DT_S)


async def phase_recovery(drone):
    print("\nPHASE: RECOVERY")
    try:
        await drone.offboard.stop()
    except Exception:
        pass

    try:
        await drone.action.return_to_launch()
        print("RTL command sent.")
    except Exception as exc:
        print(f"RTL failed ({exc}), switching to HOLD.")
        await drone.action.hold()

    log_entry("RECOVERY", altitude_m=latest_pos.relative_altitude_m if latest_pos else 0.0)


async def run():
    global stop_tasks
    args = parse_args()
    drone = System()
    await drone.connect(system_address=args.system_address)

    print("=" * 64)
    print("TOP-UP APPROACH MISSION")
    print("=" * 64)
    print(f"System: {args.system_address}")
    print(f"Lock altitude: {args.lock_alt:.1f}m | Hold: {args.hold_sec:.1f}s")
    print(
        "Target rel (N,E,UP): "
        f"({args.target_north:.1f}, {args.target_east:.1f}, {args.target_up:.1f}) m"
    )
    print(f"Ascent min/cmd: {args.ascent_min:.1f}/{args.ascent_cmd:.1f} m/s")
    print(f"Attack speed: {args.attack_speed:.1f} m/s")

    backup = {}
    tasks = []
    mission_success = False

    try:
        await wait_connection(drone)
        await wait_armable(drone)

        tasks = [
            asyncio.create_task(position_listener(drone)),
            asyncio.create_task(velocity_listener(drone)),
            asyncio.create_task(attitude_listener(drone)),
            asyncio.create_task(position_velocity_ned_listener(drone)),
        ]

        await wait_telemetry_ready()
        backup = await configure_aggressive_params(drone)

        await phase_connect_and_arm(drone)
        await phase_takeoff_initial(drone)

        await offboard_warmup(drone)
        try:
            await drone.offboard.start()
            print("Offboard active.")
        except OffboardError as exc:
            raise RuntimeError(f"Offboard start failed: {exc}") from exc

        await phase_ascent_fast_stable(
            drone=drone,
            lock_alt_m=args.lock_alt,
            ascent_min_m_s=args.ascent_min,
            ascent_cmd_m_s=args.ascent_cmd,
        )

        lock_n, lock_e, lock_d = await phase_lock_hover(drone, args.hold_sec)

        mission_success = await phase_attack_air_target(
            drone=drone,
            lock_n=lock_n,
            lock_e=lock_e,
            lock_d=lock_d,
            target_rel_north=args.target_north,
            target_rel_east=args.target_east,
            target_rel_up=args.target_up,
            hit_radius=args.hit_radius,
            attack_speed_m_s=args.attack_speed,
        )

        await phase_recovery(drone)

        print("=" * 64)
        print(f"MISSION COMPLETE (success={mission_success})")
        print("=" * 64)

    except Exception as exc:
        print(f"\nMission error: {exc}")
        log_entry("ERROR", abort_reason=str(exc))
        try:
            await phase_recovery(drone)
        except Exception:
            pass
    finally:
        stop_tasks = True
        await asyncio.sleep(0.2)
        for task in tasks:
            task.cancel()

        try:
            await restore_params(drone, backup)
        except Exception as exc:
            print(f"Parameter restore warning: {exc}")

        log_path = save_log()
        if log_path is not None:
            print(f"Log saved: {log_path}")


if __name__ == "__main__":
    asyncio.run(run())
