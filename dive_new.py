#!/usr/bin/env python3
"""
PURE BALLISTIC DIVE SCRIPT
- Cruise to a hold point
- Yaw lock toward target
- Pure ballistic dive (straight line, no path correction)
- Auto LAND after dive
"""

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time, argparse, math, sys, signal


# ---------------- USER CONFIG ---------------- #

TAKEOFF_ALT = 40.0          # climb altitude
HOLD_OFFSET = 10.0          # distance from target before dive
ARRIVE_TOL = 1.0            # cruise accuracy
CMD_RATE_HZ = 15.0          # control frequency

CRUISE_SPEED = 8.0          # PID cruise speed
CRUISE_KP = 0.3             # smoothing factor

# Ballistic dive
DIVE_SPEED = 25.0           # forward horizontal component (≈ 90 km/h)
DIVE_VZ = 12.0              # vertical down speed
DIVE_MIN_ALT = 2.0          # stop dive before ground
DIVE_MAX_TIME = 20.0        # failsafe timeout

NAV_TIMEOUT = 120.0


# -------------------------------------------------------- #
#                      CONNECT + SAFETY
# -------------------------------------------------------- #

parser = argparse.ArgumentParser()
parser.add_argument("--connect", help="connection string")
args = parser.parse_args()

connection_string = args.connect if args.connect else "udp:127.0.0.1:14550"
print("Connecting to:", connection_string)

vehicle = connect(connection_string, wait_ready=True, timeout=60)


def safe_stop(reason="Stopping"):
    print("\n[SAFE STOP] ", reason)
    for _ in range(10):
        send_ned_velocity(0,0,0)
        time.sleep(0.05)
    try:
        vehicle.mode = VehicleMode("LOITER")
    except:
        pass
    try:
        vehicle.close()
    except:
        pass
    sys.exit(0)


def signal_handler(sig, frame):
    safe_stop("Keyboard Interrupt")
signal.signal(signal.SIGINT, signal_handler)


# -------------------------------------------------------- #
#                    ARM AND TAKEOFF
# -------------------------------------------------------- #

def arm_and_takeoff(alt):
    print("Waiting for vehicle to become armable...")
    while not vehicle.is_armable:
        time.sleep(0.5)

    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(0.5)

    print(f"Taking off to {alt} m")
    vehicle.simple_takeoff(alt)

    while True:
        h = vehicle.location.global_relative_frame.alt
        print(" Alt:", round(h, 2))
        if h >= alt * 0.95:
            print("Reached takeoff altitude.")
            break
        time.sleep(0.5)


# -------------------------------------------------------- #
#                 BASIC VELOCITY SENDER
# -------------------------------------------------------- #

def send_ned_velocity(vx, vy, vz):
    """
    vx = North, vy = East, vz = Down
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        3527,       # velocity only
        0,0,0,
        vx, vy, vz,
        0,0,0,
        0,0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


# -------------------------------------------------------- #
#                 SIMPLE CRUISE TO POINT
# -------------------------------------------------------- #

def cruise_to_point(Nt, Et, Dt, speed=CRUISE_SPEED):
    """
    Simple proportional velocity approach (not PID-overkill)
    """

    print(f"[CRUISE] → target N:{Nt:.2f}  E:{Et:.2f}  D:{Dt:.2f}")

    rate_dt = 1.0 / CMD_RATE_HZ
    t0 = time.time()

    while True:
        lf = vehicle.location.local_frame
        if lf.north is None:
            time.sleep(0.1)
            continue

        dN = Nt - lf.north
        dE = Et - lf.east
        dD = Dt - lf.down
        dist = math.sqrt(dN*dN + dE*dE + dD*dD)

        print(f"[CRUISE] dist = {dist:.2f} m")

        if dist <= ARRIVE_TOL:
            print("[CRUISE] Arrived.")
            send_ned_velocity(0,0,0)
            return True

        if time.time() - t0 > NAV_TIMEOUT:
            print("[CRUISE] Timeout!")
            return False

        # proportional speed
        scale = CRUISE_KP * dist
        spd = min(speed, max(1.0, scale))

        vx = (dN/dist) * spd
        vy = (dE/dist) * spd
        vz = (dD/dist) * spd

        send_ned_velocity(vx, vy, vz)
        time.sleep(rate_dt)


# -------------------------------------------------------- #
#               YAW ALIGN TOWARD TARGET
# -------------------------------------------------------- #

def send_yaw(yaw_deg, yaw_speed=90, direction=1, relative=0):
    """
    Yaw control using MAV_CMD_CONDITION_YAW.
    yaw_deg: absolute yaw angle in degrees
    yaw_speed: deg/sec
    direction: 1=clockwise, -1=counterclockwise
    relative: 0=absolute, 1=relative
    """

    msg = vehicle.message_factory.command_long_encode(
        0, 1,                                   # target system, component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,                                      # confirmation
        yaw_deg,                                # param1: yaw angle
        yaw_speed,                              # param2: yaw speed
        direction,                              # param3: CCW=1 / CW=-1
        relative,                               # param4: absolute/relative
        0, 0, 0                                  # param5,6,7 MUST EXIST
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()



# -------------------------------------------------------- #
#               PURE BALLISTIC DIVE
# -------------------------------------------------------- #


def yaw_to_face(targetN, targetE):
    """
    Computes yaw angle (in radians) to face the target.
    Sends yaw command and returns yaw_rad.
    """
    lf = vehicle.location.local_frame
    currN = lf.north
    currE = lf.east

    # Compute yaw = arctan2(East, North)
    dN = targetN - currN
    dE = targetE - currE

    yaw_rad = math.atan2(dE, dN)
    yaw_deg = math.degrees(yaw_rad)

    print(f"[YAW] Align to {yaw_deg:.1f}°")

    # Use your yaw command sender
    send_yaw(yaw_deg, yaw_speed=45, direction=1, relative=0)

    time.sleep(1.0)   # allow yaw to settle
    return yaw_rad


def ballistic_dive_pure(yaw_rad, dive_speed=DIVE_SPEED, dive_vz=DIVE_VZ,
                        min_alt=DIVE_MIN_ALT, max_time=DIVE_MAX_TIME):

    print("\n[PUR E DIVE] Ballistic dive — NO corrections")
    print(f" Locked heading = {math.degrees(yaw_rad):.1f}°")

    # heading vector
    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)

    vx = cos_y * dive_speed
    vy = sin_y * dive_speed
    vz = dive_vz

    print(f" Command: vx={vx:.2f}  vy={vy:.2f}  vz={vz:.2f}")

    dt = 1.0 / CMD_RATE_HZ
    t0 = time.time()

    while True:
        lf = vehicle.location.local_frame
        if lf.down is None:
            time.sleep(0.05)
            continue

        alt = -lf.down
        print(f"[DIVE] alt={alt:.2f}")

        if alt <= min_alt:
            print("[DIVE] Reached minimum altitude — stopping.")
            send_ned_velocity(0,0,0)
            return True

        if time.time() - t0 > max_time:
            print("[DIVE] TIMEOUT — stopping dive.")
            send_ned_velocity(0,0,0)
            return False

        send_ned_velocity(vx, vy, vz)
        time.sleep(dt)


# -------------------------------------------------------- #
#                         MAIN
# -------------------------------------------------------- #

if __name__ == "__main__":
    try:
        arm_and_takeoff(TAKEOFF_ALT)

        # wait EKF
        time.sleep(1.0)
        lf = vehicle.location.local_frame
        while lf.north is None:
            print("Waiting EKF...")
            time.sleep(0.2)
            lf = vehicle.location.local_frame

        startN, startE, startD = lf.north, lf.east, lf.down

        # EXAMPLE TARGET (adjust as needed)
        targetN = startN - 40
        targetE = startE - 80
        targetD = startD

        # compute hold point
        vecN = targetN - startN
        vecE = targetE - startE
        L = math.sqrt(vecN*vecN + vecE*vecE)

        holdN = targetN - (vecN/L) * HOLD_OFFSET
        holdE = targetE - (vecE/L) * HOLD_OFFSET
        holdD = startD

        print(f"HOLD POINT → N:{holdN:.2f}  E:{holdE:.2f}")

        # Cruise phase
        ok = cruise_to_point(holdN, holdE, holdD)
        if not ok:
            safe_stop("Failed to reach hold point")

        # Yaw lock
        yaw_rad = yaw_to_face(targetN, targetE)

        # Pure ballistic dive
        ballast_ok = ballistic_dive_pure(yaw_rad)

        print("[POST] Switching to LOITER → LAND")
        vehicle.mode = VehicleMode("LOITER")
        time.sleep(1.0)
        vehicle.mode = VehicleMode("LAND")

        time.sleep(3)
        vehicle.close()
        print("MISSION COMPLETE.")

    except Exception as e:
        print("Exception:", e)
        safe_stop("Exception")
