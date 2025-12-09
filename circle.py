#!/usr/bin/env python3
# -*- coding: utf-8 -*-



from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import math, time

# ---------------- CONNECT TO VEHICLE ---------------- #
print("Connecting to vehicle on: tcp:127.0.0.1:5760")
vehicle = connect('127.0.0.1:14551', wait_ready=True)

# ---------------- TAKEOFF FUNCTION ---------------- #
def arm_and_takeoff(aTargetAltitude):
    print("Arming motors and taking off...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    vehicle.simple_takeoff(aTargetAltitude)
    time.sleep(6)
    print(f"Reached approx altitude: {vehicle.location.global_relative_frame.alt:.2f} m")

# ---------------- VELOCITY COMMAND ---------------- #
def send_ned_velocity(vx, vy, vz, duration):
    """
    Move the drone with specified velocities (m/s) in NED frame.
    vx -> North/South (+ forward, - backward)
    vy -> East/West (+ right, - left)
    vz -> Up/Down (- up, + down)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,   # bitmask: use velocity only
        0, 0, 0,              # position (ignored)
        vx, vy, vz,           # velocity (used)
        0, 0, 0,              # acceleration (ignored)
        0, 0                  # yaw, yaw_rate (ignored)
    )

    for _ in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

# ---------------- CIRCLE FLIGHT ---------------- #
def fly_circle(radius=10, speed=2, duration=30):
    """
    Fly a circle using continuous velocity control.
    radius: meters
    speed: m/s (tangential velocity)
    duration: total time of circular motion (s)
    """
    print(f"Starting circular flight (radius={radius}m, speed={speed}m/s)")
    start_time = time.time()
    omega = speed / radius   # angular velocity (rad/s)
    while time.time() - start_time < duration:
        t = time.time() - start_time
        vx = speed * math.cos(omega * t)   
        vy = speed * math.sin(omega * t)   
        send_ned_velocity(vx, vy, 0, 1)
        print(f"→ vx={vx:.2f}  vy={vy:.2f}")
    print("Circle complete.")

arm_and_takeoff(10)

fly_circle(radius=10, speed=2, duration=40)

print("Landing...")
vehicle.mode = VehicleMode("LAND")

while vehicle.armed:
    print(" Altitude:", round(vehicle.location.global_relative_frame.alt, 2))
    time.sleep(1)

vehicle.close()
print("Mission Complete ✅")
