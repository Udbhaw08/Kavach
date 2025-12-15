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

# ================= CAMERA =================
def get_camera():
    print("📷 Connecting to camera on port 5600...")
    gst = (
        "udpsrc port=5600 caps=\"application/x-rtp,media=video,"
        "encoding-name=H264,payload=96\" ! "
        "rtpjitterbuffer ! rtph264depay ! avdec_h264 ! "
        "videoconvert ! appsink drop=1 sync=false"
    )
    cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    time.sleep(1)

    if not cap.isOpened():
        raise RuntimeError("Camera failed")

    for _ in range(20):
        ret, frame = cap.read()
        if ret:
            print(f"✅ Camera OK ({frame.shape[1]}x{frame.shape[0]})")
            return cap

    raise RuntimeError("Camera no frames")

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

    # Verify params took effect
    print("🔧 Configuring parameters (Aggressive SITL Overrides)...")
    
    params_to_set = [
        ("CBRK_SUPPLY_CHK", 894281),   # Power
        ("CBRK_USB_CHK", 197848),      # USB
        ("CBRK_VELPOSERR", 201607),    # EKF Status Check
        ("COM_ARM_WO_GPS", 1),         # Allow Arming without GPS
        ("COM_RC_IN_MODE", 1),         # Joystick/No RC
        ("COM_RCL_EXCEPT", 4),         # Offboard ignores RC loss
        ("NAV_RCL_ACT", 0),            # RC loss = Disabled
        ("NAV_DLL_ACT", 0),            # Data Link loss = Disabled
    ]

    for name, value in params_to_set:
        try:
            await drone.param.set_param_int(name, value)
            print(f"  Set {name}={value}")
        except Exception as e:
            print(f"  ⚠️ Failed {name}: {e}")

    # Relax EKF limits (Floats) - Nuclear Option
    float_params = [
        ("COM_ARM_EKF_HGT", 500.0), # Disable height check
        ("COM_ARM_EKF_POS", 500.0), # Disable pos check
        ("COM_ARM_EKF_YAW", 500.0), # Disable yaw check
        ("COM_ARM_IMU_ACC", 500.0), # Disable IMU check
        ("COM_ARM_MAG_ANG", 500.0), # Disable Mag check
    ]

    for name, value in float_params:
        try:
            await drone.param.set_param_float(name, value)
            print(f"  Set {name}={value}")
        except Exception as e:
            print(f"  ⚠️ Failed {name}: {e}")

    # 3️⃣ CHECK HEALTH
    print("⏳ Waiting for vehicle to be armable...")
    start_wait = time.time()
    async for health in drone.telemetry.health():
        if health.is_armable: # Ignored local position check if possible
            print("✅ Vehicle is ready to arm")
            break
        
        if time.time() - start_wait > 30:
            print("⚠️ Timed out waiting for armable state. Attempting to arm anyway...")
            break
            
        # Print status text if we are stuck
        # print(f"   Waiting... (Armable: {health.is_armable})")
        await asyncio.sleep(1)

    # Give PX4 time to receive initial setpoints (Heartbeat is already running)
    await asyncio.sleep(2.0) 

    # 4️⃣ ARM FIRST (Try arming in Hold/Manual mode)
    try:
        print("💪 Arming...")
        await drone.action.arm()
        print("✅ Armed")
    except Exception as e:
        print(f"❌ Arming failed: {e}")
        # Try force arming logic here if needed, but usually redundant
        heartbeat_task.cancel()
        return

    # 5️⃣ START OFFBOARD
    try:
        print("🛫 Starting OFFBOARD...")
        # Send one explicit setpoint to be sure
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.5) # Give it time to propagate
        
        await drone.offboard.start()
        print("✅ OFFBOARD started")
    except OffboardError as e:
        print(f"❌ OFFBOARD start failed: {e}")
        # Disarm if offboard fails
        await drone.action.disarm()
        heartbeat_task.cancel()
        return

    # Stop the dummy heartbeat so the main climb logic can take over control
    heartbeat_task.cancel()

    # 6️⃣ CLIMB
    print("🚀 Taking off...")
    
    # Send an initial stream of 0 setpoints to hold position before climbing
    for _ in range(10):
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        )
        await asyncio.sleep(0.1)

    async for pos in drone.telemetry.position():
        alt = pos.relative_altitude_m
        print(f"📈 Alt: {alt:.1f} m")
        
        if alt >= SEARCH_ALTITUDE * 0.95:
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
            )
            print("✅ Takeoff complete")
            return

        # NED: negative Z = up (Climb)
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, -1.5, 0.0)
        )
        
        await asyncio.sleep(0.2)

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
