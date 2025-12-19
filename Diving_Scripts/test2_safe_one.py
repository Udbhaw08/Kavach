#!/usr/bin/env python3
import asyncio
import cv2
import numpy as np
import math
import time

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw, OffboardError

# ================= CONFIG =================
TAKEOFF_TIME = 5.0          # seconds
SEARCH_SPEED = 1.5
SEARCH_YAW_RATE = 15.0
DIVE_SPEED = 6.0
TERMINAL_ALTITUDE = 2.0
Kp_HORIZONTAL = 0.004
LOCK_STABILITY_TIME = 3.0
MIN_DETECTION_AREA = 20
CMD_DT = 0.1                # 10 Hz

# ================= CAMERA =================
class DummyCamera:
    def __init__(self):
        self.w, self.h = 640, 480
        self.t = 0

    def read(self):
        self.t += 0.1
        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        cx = int(self.w/2 + 100 * math.cos(self.t))
        cy = int(self.h/2 + 100 * math.sin(self.t))
        cv2.circle(img, (cx, cy), 20, (0,0,255), -1)
        return True, img

    def release(self):
        pass

# ================= VISION =================
def detect_red_target(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = (
        cv2.inRange(hsv, (0,100,100), (10,255,255)) +
        cv2.inRange(hsv, (160,100,100), (180,255,255))
    )

    cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, None, False

    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_DETECTION_AREA:
        return None, None, False

    M = cv2.moments(c)
    cx = int(M["m10"]/M["m00"])
    cy = int(M["m01"]/M["m00"])
    return cx, cy, True

# ================= PX4 =================
async def connect_px4():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ PX4 Connected")
            return drone

async def start_offboard(drone):
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(TAKEOFF_TIME)

    # Warm-up OFFBOARD (MANDATORY)
    for _ in range(20):
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0,0))
        await asyncio.sleep(CMD_DT)

    await drone.offboard.start()
    print("🟢 OFFBOARD ACTIVE")

# ================= PHASES =================
async def phase_search(drone, cam):
    print("🔍 SEARCH")
    start = time.time()
    while time.time() - start < 60:
        _, frame = cam.read()
        cx, cy, found = detect_red_target(frame)

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(SEARCH_SPEED, 0, 0, SEARCH_YAW_RATE)
        )

        if found:
            print("🎯 TARGET FOUND")
            return True

        await asyncio.sleep(CMD_DT)
    return False

async def phase_track(drone, cam):
    print("🎯 TRACK")
    stable = 0
    required = int(LOCK_STABILITY_TIME / CMD_DT)

    while True:
        _, frame = cam.read()
        h,w = frame.shape[:2]
        cx, cy, found = detect_red_target(frame)
        if not found:
            stable = 0
            continue

        ex = cx - w//2
        ey = cy - h//2
        err = math.hypot(ex, ey)

        vx = 0
        vy = Kp_HORIZONTAL * ex
        yaw_rate = Kp_HORIZONTAL * ex * 10

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(vx, vy, 0, yaw_rate)
        )

        stable = stable + 1 if err < 30 else 0
        if stable >= required:
            print("✅ LOCKED")
            return True

        await asyncio.sleep(CMD_DT)

async def phase_dive(drone, cam):
    print("🚀 DIVE")
    async for pos in drone.telemetry.position():
        alt = pos.relative_altitude_m
        break

    while alt > TERMINAL_ALTITUDE:
        _, frame = cam.read()
        h,w = frame.shape[:2]
        cx, cy, found = detect_red_target(frame)

        vx = vy = 0
        if found:
            vx = -Kp_HORIZONTAL * (cy - h//2)
            vy =  Kp_HORIZONTAL * (cx - w//2)

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(vx, vy, DIVE_SPEED, 0)
        )

        async for pos in drone.telemetry.position():
            alt = pos.relative_altitude_m
            break

        await asyncio.sleep(CMD_DT)

    print("💥 IMPACT")

# ================= MAIN =================
async def main():
    cam = DummyCamera()
    drone = await connect_px4()
    await start_offboard(drone)

    if not await phase_search(drone, cam):
        return
    if not await phase_track(drone, cam):
        return

    await phase_dive(drone, cam)

    await drone.offboard.stop()
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(main())
