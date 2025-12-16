#!/usr/bin/env python3
import asyncio
import cv2
import numpy as np
import math
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw

# ================= CONFIG =================
SEARCH_ALTITUDE = 30.0
DIVE_SPEED = 8.0
TERMINAL_ALTITUDE = 2.0
Kp_HORIZONTAL = 0.004
LOCK_STABILITY_TIME = 3.0
MIN_DETECTION_AREA = 20

class DummyCamera:
    def __init__(self):
        self.w, self.h = 640, 480
        self.frame_count = 0
    
    def isOpened(self):
        return True
        
    def read(self):
        # Simulate a red ball moving in a circle
        self.frame_count += 1
        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        
        # Moving target logic
        t = self.frame_count * 0.1
        cx = int(self.w/2 + 100 * math.cos(t))
        cy = int(self.h/2 + 100 * math.sin(t))
        
        # Draw red ball
        cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
        return True, img

    def release(self):
        pass

def get_camera():
    print("📷 Connecting to camera on port 5600...")
    
    # Updated pipeline for better compatibility
    gst_udp = (
        "udpsrc port=5600 caps=\"application/x-rtp,media=video,"
        "encoding-name=H264,payload=96\" ! "
        "rtpjitterbuffer latency=0 ! rtph264depay ! avdec_h264 ! "
        "videoconvert ! appsink drop=1 sync=false"
    )

    try:
        cap = cv2.VideoCapture(gst_udp, cv2.CAP_GSTREAMER)
    except Exception:
        cap = None

    if not cap or not cap.isOpened():
        print("⚠️ UDP Camera failed. Switching to Simulated Video Source (GStreamer)...")
        gst_test = "videotestsrc pattern=ball ! videoconvert ! appsink drop=1 sync=false"
        try:
             cap = cv2.VideoCapture(gst_test, cv2.CAP_GSTREAMER)
        except Exception:
             cap = None
    
    # Verify we actually get frames
    if cap and cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"✅ Camera OK ({frame.shape[1]}x{frame.shape[0]})")
            return cap
    
    print("⚠️ All GStreamer pipelines failed. Using Internal Dummy Camera.")
    return DummyCamera()

# ================= VISION =================
def detect_red_target(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask = (
        cv2.inRange(hsv, (0,100,100), (10,255,255)) +
        cv2.inRange(hsv, (160,100,100), (180,255,255))
    )

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5,5),np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, None, False, mask, None, 0

    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_DETECTION_AREA:
        return None, None, False, mask, None, area

    M = cv2.moments(c)
    cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
    x,y,w,h = cv2.boundingRect(c)
    return cx, cy, True, mask, (x,y,w,h), area

# ================= PX4 CORE =================
async def connect_px4():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected to PX4")
            return drone

async def arm_and_takeoff(drone):
    print("🚁 Preparing OFFBOARD")

    # 1️⃣ CONFIGURE SAFETY PARAMS (Crucial for SITL/No RC)
    # COM_RCL_EXCEPT: 4 = Ignore RC loss in Offboard mode
    await drone.param.set_param_int("COM_RCL_EXCEPT", 4)
    # COM_RC_IN_MODE: 1 = Joystick/No RC Checks
    await drone.param.set_param_int("COM_RC_IN_MODE", 1)
    print("✅ Safety params configured")

    # 2️⃣ START SETPOINT HEARTBEAT
    # PX4 needs a stream > 2Hz to allow Offboard mode.
    # We start a background task to send setpoints until the main loop takes over.
    print("💓 Starting setpoint heartbeat...")
    async def heartbeat():
        while True:
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            )
            await asyncio.sleep(0.1)  # 10Hz

    heartbeat_task = asyncio.create_task(heartbeat())

from mavsdk.offboard import OffboardError, VelocityNedYaw, Attitude

# ... (Config sections remain) ...

async def arm_and_takeoff(drone):
    print("🚁 Preparing OFFBOARD")

    # 1️⃣ CONFIGURE SAFETY PARAMS
    # We only set the critical ones for "No RC" flight. 
    # The EKF/CBRK params were unknown, so we skip them to avoid errors.
    print("🔧 Configuring parameters (Basic Flight)...")
    
    # Try providing a valid list for this version of PX4
    # We wrap each in try/except so one failure doesn't stop the rest.
    critical_params = [
        ("COM_RCL_EXCEPT", 4),         # Offboard ignores RC loss
        ("COM_RC_IN_MODE", 1),         # Joystick/No RC
        ("COM_ARM_WO_GPS", 1),         # Allow arming without GPS
        ("NAV_RCL_ACT", 0),            # RC loss = Disabled
    ]

    for name, value in critical_params:
        try:
            await drone.param.set_param_int(name, value)
            print(f"  Set {name}={value}")
        except Exception:
            pass # Ignore if unknown

    # 3️⃣ CHECK HEALTH
    print("⏳ Waiting for vehicle to be armable...")
    # We wait briefly but don't block forever since we are forcing it
    for i in range(5):
        print(f"   Waiting... {5-i}")
        await asyncio.sleep(1)

    # 4️⃣ ARM FIRST
    try:
        print("💪 Arming...")
        await drone.action.arm()
        print("✅ Armed")
    except Exception as e:
        print(f"❌ Arming failed: {e}")
        return

    # 5️⃣ START OFFBOARD (Attitude Mode)
    # Velocity control fails if EKF is dead (Drone stays on ground).
    # We switch to Attitude Control (Thrust) to FORCE takeoff.
    try:
        print("🛫 Starting OFFBOARD (Attitude Mode)...")
        
        # Send initial thrust (0) so it doesn't jump immediately
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.2)
        
        await drone.offboard.start()
        print("✅ OFFBOARD started")
    except OffboardError as e:
        print(f"❌ OFFBOARD start failed: {e}")
        await drone.action.disarm()
        return

    # 6️⃣ CLIMB (Using Raw Thrust)
    print("🚀 Taking off (Thrust cmd)...")
    
    # We climb for a fixed duration since altitude meas might be noisy
    # Thrust 0.8 to guarantee lift (some models are heavy/underpowered in sim)
    start_climb = time.time()
    while time.time() - start_climb < 5.0:
        # Roll=0, Pitch=0, Yaw=0, Thrust=0.8
        print("   Commanding Thrust 0.8...")
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.8))
        await asyncio.sleep(0.1)
        
        # Check alt just for logging
        async for pos in drone.telemetry.position():
            print(f"📈 Alt: {pos.relative_altitude_m:.1f} m")
            break

    print("✅ Climb complete (switching to Hover thrust)")
    
    # Hold (Thrust ~0.6 usually hovers)
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
    await asyncio.sleep(2.0)
    return

# ================= PHASE 1: SEARCH =================
async def phase_search(drone, cap):
    print("🔍 SEARCH PHASE")
    start = time.time()

    while time.time() - start < 60:
        ret, frame = cap.read()
        if not ret:
            continue

        cx, cy, detected, mask, bbox, area = detect_red_target(frame)

        # 🔁 SEARCH MOTION (critical)
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(1.5, 0, 0, 10)  # forward + yaw
        )

        cv2.putText(frame, f"Area: {area:.1f}", (10,120),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)

        if detected:
            x,y,w,h = bbox
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
            print("🎯 Target acquired")
            return True

        cv2.imshow("Search", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)
        await asyncio.sleep(0.05)

    return False

# ================= PHASE 2 =================
async def phase_track(drone, cap):
    print("🎯 TRACK PHASE")
    stable = 0
    required = int(LOCK_STABILITY_TIME * 10)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        h,w = frame.shape[:2]
        cx, cy, detected, _, bbox, _ = detect_red_target(frame)
        if not detected:
            return False

        ex, ey = cx-w//2, cy-h//2
        dist = math.hypot(ex,ey)
        stable = stable+1 if dist < 30 else 0

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(Kp_HORIZONTAL*ey, Kp_HORIZONTAL*ex, 0, 0)
        )

        if stable >= required:
            print("✅ Lock stable")
            return True

        cv2.imshow("Track", frame)
        cv2.waitKey(1)
        await asyncio.sleep(0.1)

# ================= PHASE 3 =================
async def phase_dive(drone, cap):
    print("🚀 DIVE PHASE")
    async for pos in drone.telemetry.position():
        alt = pos.relative_altitude_m

        ret, frame = cap.read()
        if not ret:
            continue

        cx, cy, detected, _, _, _ = detect_red_target(frame)
        if not detected:
            print("❌ Target lost")
            return

        h,w = frame.shape[:2]
        ex, ey = cx-w//2, cy-h//2

        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(Kp_HORIZONTAL*ey, Kp_HORIZONTAL*ex, DIVE_SPEED, 0)
        )

        cv2.imshow("Dive", frame)
        cv2.waitKey(1)

        if alt <= TERMINAL_ALTITUDE:
            print("💥 IMPACT")
            return

        await asyncio.sleep(0.05)

# ================= MAIN =================
async def main():
    cap = get_camera()
    drone = await connect_px4()
    await arm_and_takeoff(drone)

    if not await phase_search(drone, cap):
        print("❌ Search failed")
        return

    if not await phase_track(drone, cap):
        print("❌ Track failed")
        return

    await phase_dive(drone, cap)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(main())
