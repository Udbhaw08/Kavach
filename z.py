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
    """Arms vehicle and fly to target altitude (fast simulation version)."""
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

    # simplified for simulation
    time.sleep(5)  # allow climb time (~5 seconds in SITL)

    # Optional small correction (not safety-critical)
    target = vehicle.location.global_relative_frame
    target.alt = aTargetAltitude
    vehicle.simple_goto(target)
    time.sleep(3)
    print(f"Takeoff complete, current altitude: {vehicle.location.global_relative_frame.alt:.2f} m")

    """Arms vehicle and fly to target altitude with correction."""
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

    start_time = time.time()
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.2f} m")
        if alt >= aTargetAltitude * 0.90:
            print(f"Reached target altitude ({alt:.2f} m)")
            break
        if time.time() - start_time > 25:
            print("Timeout: altitude did not reach target, continuing anyway.")
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

TARGET_ALT = 20  

print("Arming and taking off...")
arm_and_takeoff(TARGET_ALT)

print("\nGoing straight 10 meters east")
vehicle.groundspeed = 3
goto(0, 10)

print("\nGoinghellloo")
goto(-10, -25, 5)
print("\nGoing straight 10 meters north")
vehicle.groundspeed = 3
goto(0, 10)


print("\nLanding...")
vehicle.mode = VehicleMode("LAND")

while vehicle.armed:
    print(" Altitude:", round(vehicle.location.global_relative_frame.alt, 2))
    time.sleep(1)

print("Landed and disarmed.")
vehicle.close()

