

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import argparse


# ---------------- CONNECT ---------------- #
parser = argparse.ArgumentParser()
parser.add_argument("--connect", help="Connection string")
args = parser.parse_args()

connection_string = args.connect if args.connect else "udp:127.0.0.1:14550"
print("Connecting to:", connection_string)

vehicle = connect(connection_string, wait_ready=True)


# ---------------- ARM + TAKEOFF ---------------- #
def arm_and_takeoff(alt):
    print("Waiting for vehicle to become armable...")
    while not vehicle.is_armable:
        time.sleep(1)

    print("Arming...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off to", alt, "m")
    vehicle.simple_takeoff(alt)

    while True:
        curr = vehicle.location.global_relative_frame.alt
        print(" Alt:", curr)
        if curr >= alt * 0.95:
            print("Reached takeoff altitude.")
            break
        time.sleep(1)


# ---------------- NED POSITION FUNCTION (YOUR CODE) ---------------- #
def send_ned_position(north, east, down):
    """
    Send LOCAL_NED target position (absolute in EKF frame).
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,    # YOUR BITMASK
        north, east, down,     # N, E, D (D positive = DOWN)
        0, 0, 0,               # velocities ignored
        0, 0, 0,               # accelerations ignored
        0, 0                   # yaw ignored
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


# ------------- RELATIVE NED MOVEMENT ------------- #
def move_by_ned(dN, dE, dD):
    """
    Move relative to current EKF local origin.
    """
    loc = vehicle.location.local_frame

    # Wait if EKF not ready
    while loc.north is None:
        print("Waiting for EKF local frame...")
        time.sleep(0.5)
        loc = vehicle.location.local_frame

    targetN = loc.north + dN
    targetE = loc.east  + dE
    targetD = loc.down  + dD

    print(f"[MOVE] ΔN:{dN}, ΔE:{dE}, ΔD:{dD}  → Target(N:{targetN}, E:{targetE}, D:{targetD})")
    send_ned_position(targetN, targetE, targetD)

    time.sleep(3)  # allow motion to complete


# ---------------- MISSION ---------------- #
print("Arming and taking off...")
arm_and_takeoff(20)

print("\nTop of Z → Move 10m East")
move_by_ned(0, 10, 0)

print("\nDiagonal → Move South 10m, West 25m, DOWN 15m")
move_by_ned(-10, -25, +15)


print("\nBottom of Z → Move 10m East")
move_by_ned(0, 50, 0)

print("\nLanding...")
vehicle.mode = VehicleMode("LAND")

while vehicle.armed:
    print(" Alt:", vehicle.location.global_relative_frame.alt)
    time.sleep(1)

print("Mission Complete. Vehicle Disarmed.")
vehicle.close()
