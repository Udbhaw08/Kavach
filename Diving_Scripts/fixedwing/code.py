#!/usr/bin/env python3
"""
VTOL AGGRESSIVE FIXED-WING DIVE (RAW ATTITUDE)
=============================================
SIMULATION ONLY (PX4 SITL)

- PX4 used only as motor mixer
- Exact pitch control via SET_ATTITUDE_TARGET
- No PX4 navigation / velocity smoothing
- Guaranteed visible ~35° dive
"""

import time
import math
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

# ================= CONFIG =================
ATTACK_ALTITUDE = 250.0
MIN_SAFE_ALTITUDE = 30.0

CLIMB_THRUST = 0.7
DIVE_THRUST = 1.0
PULLUP_THRUST = 0.9

DIVE_PITCH_DEG = -35.0
PULLUP_PITCH_DEG = 25.0

DIVE_DURATION = 4.0
PULLUP_DURATION = 2.0

RATE_HZ = 50
DT = 1.0 / RATE_HZ

# ================= CONNECT =================
master = mavutil.mavlink_connection("udp:127.0.0.1:14540")
master.wait_heartbeat()
print("✅ Connected to PX4 SITL")

BOOT_TIME = time.time()

def ms_since_boot():
    return int((time.time() - BOOT_TIME) * 1000)

# ================= HELPERS =================
def set_mode(mode):
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode
    )

def arm():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )

def vtol_fw():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_VTOL_TRANSITION,
        0,
        mavutil.mavlink.MAV_VTOL_STATE_FW,
        0, 0, 0, 0, 0, 0
    )

def send_attitude(pitch_deg, thrust):
    q = QuaternionBase([0.0, math.radians(pitch_deg), 0.0]).q

    master.mav.set_attitude_target_send(
        ms_since_boot(),
        master.target_system,
        master.target_component,
        0b00000100,   # ignore yaw angle, use yaw rate
        q,
        0.0,
        0.0,
        0.0,
        thrust
    )

def get_altitude():
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
    return msg.relative_alt / 1000.0

# ================= MODES =================
OFFBOARD = 6

# ================= OFFBOARD PRIME =================
print("⏳ Priming offboard...")
for _ in range(50):
    send_attitude(0.0, 0.5)
    time.sleep(0.02)

set_mode(OFFBOARD)
arm()
print("🔓 Armed & OFFBOARD enabled")

# ================= CLIMB =================
print("🚀 Climbing to attack altitude")
while True:
    alt = get_altitude()
    send_attitude(0.0, CLIMB_THRUST)
    print(f"ALT {alt:6.1f} m", end="\r")
    if alt >= ATTACK_ALTITUDE:
        break
    time.sleep(DT)

print(f"\n✅ Reached {alt:.1f} m")

# ================= TRANSITION TO FW =================
print("🔄 Transitioning to FIXED-WING")
vtol_fw()
time.sleep(3.0)

# ================= STABILIZE =================
print("✈️ Stabilizing")
for _ in range(50):
    send_attitude(0.0, 0.6)
    time.sleep(DT)

# ================= RAW DIVE =================
print("⚡ RAW 35° FIXED-WING DIVE")

start = time.time()
while time.time() - start < DIVE_DURATION:
    alt = get_altitude()
    send_attitude(DIVE_PITCH_DEG, DIVE_THRUST)
    print(f"ALT {alt:6.1f} m | Pitch {DIVE_PITCH_DEG}°", end="\r")

    if alt <= MIN_SAFE_ALTITUDE:
        break
    time.sleep(DT)

print("\n↗ PULL-UP")

# ================= PULL-UP =================
start = time.time()
while time.time() - start < PULLUP_DURATION:
    send_attitude(PULLUP_PITCH_DEG, PULLUP_THRUST)
    time.sleep(DT)

print("🛬 RECOVERY")
send_attitude(0.0, 0.6)
time.sleep(1.0)

print("✅ MANEUVER COMPLETE (SIMULATION)")
