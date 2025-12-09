#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
AI TRANSCOM | L-Shape Flight Demo (DroneKit + Gazebo)
1. Arm and takeoff to target altitude (accurate correction)
2. Fly straight 10m north
3. Turn right (+90° yaw)
4. Fly 20m east
5. Land
"""

from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import argparse

# ---------------- CONNECT TO VEHICLE ---------------- #
parser = argparse.ArgumentParser(description='L-Shape Drone Flight with Altitude Correction')
parser.add_argument('--connect', help="Vehicle connection target string.")
args = parser.parse_args()

connection_string = args.connect

if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
else:
    sitl = None

print("Connecting to vehicle on: %s" % connection_string)
vehicle = connect('127.0.0.1:14550', wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    """Arms vehicle and fly to target altitude."""
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {aTargetAltitude} m")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.2f} m")
        if alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# ---------------- LOCATION HELPERS ---------------- #
def get_location_metres(original_location, dNorth, dEast):
    """Returns a location `dNorth` and `dEast` meters from original location."""
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """Returns ground distance in meters between two LocationGlobal objects."""
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

# ---------------- MOVE AND YAW ---------------- #
def condition_yaw(heading, relative=False):
    """Rotate to a specific yaw heading."""
    if relative:
        is_relative = 1
    else:
        is_relative = 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,
        0,  
        1,  
        is_relative,
        0, 0, 0
    )
    vehicle.send_mavlink(msg)

def goto(dNorth, dEast):
    """Moves the vehicle by dNorth and dEast meters."""
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    vehicle.simple_goto(targetLocation)

    start_time = time.time()
    while vehicle.mode.name == "GUIDED":
        remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target:", round(remainingDistance, 2))
        if remainingDistance <= 0.5:
            print("Reached target")
            break
        if time.time() - start_time > 20:
            print("Timeout while moving to target, continuing...")
            break
        time.sleep(1)

def goto_3d(dNorth, dEast, target_alt):
    """Moves the vehicle to a 3D target (dNorth, dEast, target_alt)."""
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    
    targetLocation.alt = target_alt
    
    print(f"Moving to North:{dNorth}, East:{dEast}, Alt:{target_alt}")
    # Command desired 3D position: the script requests the target location+altitude via simple_goto.
    # The vehicle's autopilot (flight controller) receives this setpoint and controls motors/attitude
    # to reach the requested altitude — the script does not directly drive motors.
    vehicle.simple_goto(targetLocation)

    start_time = time.time()
    while vehicle.mode.name == "GUIDED":
        remainingDistance = get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        currentAlt = vehicle.location.global_relative_frame.alt
        # altDiff is the absolute difference (error) between current altitude and the requested target altitude.
        # We read currentAlt from the vehicle telemetry; we compare it to target_alt to know when altitude is reached.
        altDiff = abs(currentAlt - target_alt)
        
        print(f"Dist: {remainingDistance:.2f}m, Alt: {currentAlt:.2f}m")
        
        if remainingDistance <= 1.0 and altDiff <= 1.0:
            print("Reached 3D target")
            break
        if time.time() - start_time > 30:
            print("Timeout while moving to 3D target, continuing...")
            break
        time.sleep(1)

TARGET_ALT = 20  

print("Arming and taking off...")
arm_and_takeoff(TARGET_ALT)

print("\nTop of Z: Going straight 10 meters east")
vehicle.groundspeed = 3
goto(0, 10)
 
print("\nDiagonal of Z: Diving to 5m while moving South-West")

goto_3d(-10, -25, 5)

print("\nBottom of Z: Going straight 10 meters east")
vehicle.groundspeed = 3
goto(0, 10)


print("\nLanding...")
vehicle.mode = VehicleMode("LAND")

while vehicle.armed:
    print(" Altitude:", round(vehicle.location.global_relative_frame.alt, 2))
    time.sleep(1)

print("Landed and disarmed.")
vehicle.close()

