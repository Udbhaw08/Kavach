#!/usr/bin/env python3
import asyncio
import cv2
import numpy as np
import math
import time

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, Attitude

# ================= CONFIG =================
DIVE_SPEED = 5.0         
TERMINAL_ALTITUDE = 2.0  
Kp_HORIZONTAL = 0.004    
LOCK_STABILITY_TIME = 3.0
MIN_DETECTION_AREA = 20

# ================= CAMERA SIMULATION =================
class DummyCamera:
    def __init__(self):
        self.w, self.h = 640, 480
        self.frame_count = 0
    
    def isOpened(self):
        return True
        
    def read(self):
        self.frame_count += 1
        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        
        # Red ball moving in circle
        t = self.frame_count * 0.1
        cx = int(self.w/2 + 100 * math.cos(t))
        cy = int(self.h/2 + 100 * math.sin(t))
        
        cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
        return True, img

    def release(self):
        pass

def get_camera():
    print("📷 Connecting to camera...")
    # Try generic UDP stream
    gst_udp = "udpsrc port=5600 ! application/x-rtp,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink"
    try:
        cap = cv2.VideoCapture(gst_udp, cv2.CAP_GSTREAMER)
        if cap.isOpened():
             print("✅ UDP Camera Connected")
             return cap
    except:
        pass
    print("⚠️ Using Simulation Camera.")
    return DummyCamera()

# ================= VISION =================
def detect_red_target(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = (
        cv2.inRange(hsv, (0, 70, 50), (10, 255, 255)) +
        cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))
    )
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5,5),np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not cnts: return None, None, False, mask, None, 0

    c = max(cnts, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_DETECTION_AREA: return None, None, False, mask, None, area

    M = cv2.moments(c)
    if M["m00"] == 0: return None, None, False, mask, None, 0
    cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
    x,y,w,h = cv2.boundingRect(c)
    return cx, cy, True, mask, (x,y,w,h), area

# ================= FLIGHT CONTROL (BULLETPROOF) =================
async def connect_px4():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("⏳ Connecting to Drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Connected to PX4")
            break
    return drone

async def arm_and_takeoff_nogps(drone):
    print("🚁 Preparing Mission (NO GPS MODE)")

    # 1. PARAMETERS
    print("🔧 Setting Safety Params...")
    try:
        await drone.param.set_param_int("COM_ARM_WO_GPS", 1)
        await drone.param.set_param_int("COM_RCL_EXCEPT", 4)
    except: pass

    # 2. WAIT FOR ARMABLE
    print("⏳ Waiting for Arm Ready...")
    async for health in drone.telemetry.health():
        if health.is_armable:
            print("✅ Ready to Arm")
            break
        await asyncio.sleep(1)

    # 3. ARM
    try:
        await drone.action.arm()
        print("💪 Armed")
    except Exception as e:
        print(f"❌ Arming failed: {e}")
        return False

    # 4. START HIGH-SPEED HEARTBEAT (The Fix)
    print("💓 Starting High-Speed Heartbeat...")
    
    # Run this in background: Send packet every 0.05s
    async def fast_heartbeat():
        while True:
            await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
            await asyncio.sleep(0.05) 
            
    heartbeat_task = asyncio.create_task(fast_heartbeat())

    # 5. WARM UP (Crucial!)
    print("⏳ Warming up connection (4s)...")
    await asyncio.sleep(4) 

    # 6. TRY TO SWITCH (With Retry)
    print("🎮 Switching to OFFBOARD...")
    for attempt in range(3):
        try:
            await drone.offboard.start()
            print("✅ OFFBOARD Control Active")
            break
        except OffboardError as e:
            print(f"⚠️ Attempt {attempt+1} failed: {e}")
            print("   Retrying in 2 seconds...")
            await asyncio.sleep(2)
    else:
        print("❌ Failed to switch after 3 attempts")
        heartbeat_task.cancel()
        return False

    # 7. FORCE CLIMB
    print("🚀 Forcing Climb (Thrust 70%)...")
    heartbeat_task.cancel() # Stop the 'Stay Still' command

    # Climb: 3 seconds
    for _ in range(30):
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.7))
        await asyncio.sleep(0.1)
    
    # Hover: 2 seconds
    print("✅ Hovering")
    for _ in range(20):
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
        await asyncio.sleep(0.1)

    return True

# ================= MISSION PHASES =================
async def phase_search(drone, cap):
    print("🔍 SEARCH PHASE")
    start = time.time()
    while time.time() - start < 60:
        ret, frame = cap.read()
        if not ret: continue

        cx, cy, detected, _, _, _ = detect_red_target(frame)

        try:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(0.5, 0, 0, 15))
        except:
            # Fallback for No-GPS drift
            await drone.offboard.set_attitude(Attitude(0.0, -5.0, 0.0, 0.6))

        cv2.imshow("Search", frame)
        cv2.waitKey(1)

        if detected:
            print("🎯 Target Found!")
            return True
        await asyncio.sleep(0.05)
    return False

async def phase_track(drone, cap):
    print("🎯 TRACK PHASE")
    stable_count = 0
    required = int(LOCK_STABILITY_TIME * 10)

    while True:
        ret, frame = cap.read()
        if not ret: continue

        h,w = frame.shape[:2]
        cx, cy, detected, _, _, _ = detect_red_target(frame)
        
        if not detected:
            print("❌ Lost target")
            try: await drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0,0))
            except: pass
            stable_count = 0
            continue

        err_x, err_y = cx - w//2, cy - h//2
        if math.hypot(err_x, err_y) < 40:
            stable_count += 1
            print(f"   Locking {stable_count}/{required}")
        else:
            stable_count = 0

        vel_fwd = -1 * err_y * Kp_HORIZONTAL
        vel_right = err_x * Kp_HORIZONTAL

        try:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(vel_fwd, vel_right, 0, 0))
        except:
             # Tilt fallback
             roll = err_x * 0.05
             pitch = -1 * err_y * 0.05
             await drone.offboard.set_attitude(Attitude(roll, pitch, 0.0, 0.6))

        cv2.imshow("Track", frame)
        cv2.waitKey(1)

        if stable_count >= required:
            print("✅ Locked!")
            return True
        await asyncio.sleep(0.1)

async def phase_dive(drone, cap):
    print("🚀 DIVE PHASE")
    while True:
        async for pos in drone.telemetry.position():
            alt = pos.relative_altitude_m
            break
        
        if alt <= TERMINAL_ALTITUDE:
            print(f"💥 IMPACT at {alt:.1f}m")
            try: await drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0,0))
            except: pass
            return

        ret, frame = cap.read()
        if not ret: continue

        h,w = frame.shape[:2]
        cx, cy, detected, _, _, _ = detect_red_target(frame)
        
        vel_fwd, vel_right = 0, 0
        if detected:
            vel_fwd = -1 * (cy - h//2) * Kp_HORIZONTAL
            vel_right = (cx - w//2) * Kp_HORIZONTAL

        try:
            await drone.offboard.set_velocity_ned(VelocityNedYaw(vel_fwd, vel_right, DIVE_SPEED, 0))
        except:
             await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.4))

        cv2.imshow("Dive", frame)
        cv2.waitKey(1)
        await asyncio.sleep(0.05)

# ================= MAIN =================
async def main():
    cap = get_camera()
    drone = await connect_px4()
    
    if await arm_and_takeoff_nogps(drone):
        if await phase_search(drone, cap):
            if await phase_track(drone, cap):
                await phase_dive(drone, cap)

    cap.release()
    cv2.destroyAllWindows()
    try: await drone.action.land()
    except: pass

if __name__ == "__main__":
    asyncio.run(main())
