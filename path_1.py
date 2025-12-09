

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import argparse
import math


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
# def send_ned_position(north, east, down):
#     """
#     Send LOCAL_NED target position (absolute in EKF frame).
#     """
#     msg = vehicle.message_factory.set_position_target_local_ned_encode(
#         0,  # time_boot_ms
#         0, 0,
#         mavutil.mavlink.MAV_FRAME_LOCAL_NED,
#         0b0000111111111000,    # YOUR BITMASK
#         north, east, down,     # N, E, D (D positive = DOWN)
#         0, 0, 0,               # velocities ignored
#         0, 0, 0,               # accelerations ignored
#         0, 0                   # yaw ignored
#     )
#     vehicle.send_mavlink(msg)
#     vehicle.flush()



# ---------- Replace send_ned_velocity and move_to_point_with_velocity ----------

def send_body_velocity(vx_body, vy_body, vz_body):
    """
    Sends velocity in BODY_OFFSET_NED frame (forward,right,down).
    vx_body = forward (m/s), vy_body = right (m/s), vz_body = down (m/s)
    Uses velocity-only type_mask = 3527 (0x0DC7).
    """
    # clamp to safe values (optional)
    max_v = 5.0
    vx_body = max(-max_v, min(max_v, vx_body))
    vy_body = max(-max_v, min(max_v, vy_body))
    vz_body = max(-1.5, min(1.5, vz_body))  # keep vertical small

    # type_mask = 3527 => velocity only
    type_mask = 3527

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,                      # time_boot_ms (not used)
        0, 0,                   # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # body frame, velocity relative to heading
        type_mask,
        0, 0, 0,                # x, y, z positions (ignored)
        vx_body, vy_body, vz_body,  # vx, vy, vz in body frame (m/s)
        0, 0, 0,                # afx, afy, afz (ignored)
        0, 0                    # yaw, yaw_rate (ignored)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


def move_to_point_with_velocity(targetN, targetE, targetD, speed=2.0, arrive_tol=0.5):
    """
    Move to an absolute EKF-local (N,E,D) point using repeated body-frame velocity commands.
    This converts the required NED velocity into the vehicle's body frame using yaw.
    """
    print(f"Moving to (N:{targetN:.3f}, E:{targetE:.3f}, D:{targetD:.3f}) at {speed} m/s")

    # wait for local_frame to be valid
    loc = vehicle.location.local_frame
    while loc.north is None:
        print("Waiting for EKF local frame...")
        time.sleep(0.2)
        loc = vehicle.location.local_frame

    # ensure GUIDED mode
    if vehicle.mode.name != "GUIDED":
        print("Switching to GUIDED mode...")
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)

    rate_hz = 5.0
    dt = 1.0 / rate_hz

    while True:
        loc = vehicle.location.local_frame
        currN = loc.north
        currE = loc.east
        currD = loc.down

        # safety: if any are None, wait
        if currN is None:
            print("Local frame lost, waiting...")
            time.sleep(0.2)
            continue

        # delta in NED
        dN = targetN - currN
        dE = targetE - currE
        dD = targetD - currD
        dist = math.sqrt(dN**2 + dE**2 + dD**2)

        print(f"Distance remaining: {dist:.2f} m")

        if dist <= arrive_tol:
            print("Reached target (within tolerance). Stopping.")
            send_body_velocity(0,0,0)
            break

        # compute NED velocity vector (unit * speed)
        dirN = dN / dist
        dirE = dE / dist
        dirD = dD / dist

        vx_ned = dirN * speed
        vy_ned = dirE * speed
        vz_ned = dirD * speed   # down positive in NED

        # get vehicle yaw (rad). vehicle.attitude.yaw exists in dronekit
        yaw = vehicle.attitude.yaw
        if yaw is None:
            yaw = 0.0

        # rotate NED velocity into BODY frame:
        # body_x = cos(yaw)*north + sin(yaw)*east
        # body_y = -sin(yaw)*north + cos(yaw)*east
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        vx_body = cos_y * vx_ned + sin_y * vy_ned
        vy_body = -sin_y * vx_ned + cos_y * vy_ned
        vz_body = vz_ned  # down is same sign

        # debug print
        # print(f"yaw={yaw:.3f} | vx_ned={vx_ned:.2f}, vy_ned={vy_ned:.2f} -> vx_body={vx_body:.2f}, vy_body={vy_body:.2f}")

        # send velocity in body frame
        send_body_velocity(vx_body, vy_body, vz_body)

        time.sleep(dt)



# # ------------- RELATIVE NED MOVEMENT ------------- #
# def move_by_ned(dN, dE, dD):
#     """
#     Move relative to current EKF local origin.
#     """
#     loc = vehicle.location.local_frame

#     # Wait if EKF not ready
#     while loc.north is None:
#         print("Waiting for EKF local frame...")
#         time.sleep(0.5)
#         loc = vehicle.location.local_frame

#     targetN = loc.north + dN
#     targetE = loc.east  + dE
#     targetD = loc.down  + dD

#     print(f"[MOVE] ΔN:{dN}, ΔE:{dE}, ΔD:{dD}  → Target(N:{targetN}, E:{targetE}, D:{targetD})")
#     send_ned_position(targetN, targetE, targetD)

#     time.sleep(3)  # allow motion to complete




# ---------------- MISSION ---------------- #
print("Arming and taking off...")
arm_and_takeoff(20)

# print("\nTop of Z → Move 10m East")
# move_by_ned(0, 10, 0)

# print("\nDiagonal → Move South 10m, West 25m, DOWN 15m")
# move_by_ned(-10, -25, +15)


# print("\nBottom of Z → Move 10m East")
# move_by_ned(0, 50, 0)

# print("\nLanding...")
# vehicle.mode = VehicleMode("LAND")

# while vehicle.armed:
#     print(" Alt:", vehicle.location.global_relative_frame.alt)
#     time.sleep(1)

# Get current local frame
loc = vehicle.location.local_frame
startN = loc.north
startE = loc.east
startD = loc.down

# End target (relative example)
endN = startN - 10
endE = startE - 20
endD = startD + 10

move_to_point_with_velocity(endN, endE, endD, speed=10)


print("Mission Complete. Vehicle Disarmed.")
vehicle.close()
