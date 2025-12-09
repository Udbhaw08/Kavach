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


# ---------------- SEND LOCAL_NED VELOCITY ---------------- #
def send_ned_velocity(vx, vy, vz):
    """
    Send velocity in LOCAL_NED frame (world-fixed).
    vx = North (m/s), vy = East (m/s), vz = Down (m/s).
    Uses velocity-only type_mask = 3527.
    """
    # safe clamps
    max_horiz = 8.0
    max_vert = 2.0
    vx = max(-max_horiz, min(max_horiz, vx))
    vy = max(-max_horiz, min(max_horiz, vy))
    vz = max(-max_vert, min(max_vert, vz))

    type_mask = 3527  # velocity only
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0, 0, 0,        # pos ignored
        vx, vy, vz,     # velocities (m/s)
        0, 0, 0,        # acc ignored
        0, 0            # yaw/yaw rate ignored
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


# ---------- P-style velocity navigator (LOCAL_NED) ----------
def move_to_point_with_velocity(targetN, targetE, targetD,
                                speed_max=3.0,
                                speed_min=0.3,
                                k_p=0.6,
                                arrive_tol=0.6,
                                rate_hz=10.0,
                                max_time=120.0):
    """
    Move to absolute EKF-local (N,E,D) using world-frame velocities (LOCAL_NED).
    - speed_max: maximum horizontal speed (m/s)
    - speed_min: minimum commanded speed (m/s)
    - k_p: proportional gain (v = k_p * dist)
    - arrive_tol: tolerance in meters
    - rate_hz: command frequency
    - max_time: safety timeout (seconds)
    """
    print(f"Moving to (N:{targetN:.3f}, E:{targetE:.3f}, D:{targetD:.3f}) using LOCAL_NED, vmax={speed_max}")

    # Wait for EKF local frame
    loc = vehicle.location.local_frame
    start_wait = time.time()
    while loc.north is None:
        if time.time() - start_wait > 10:
            raise RuntimeError("Timeout waiting for local_frame to initialize")
        print("Waiting for EKF local frame...")
        time.sleep(0.2)
        loc = vehicle.location.local_frame

    # ensure GUIDED
    if vehicle.mode.name != "GUIDED":
        print("Switching to GUIDED mode...")
        vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)

    dt = 1.0 / rate_hz
    t0 = time.time()
    prev_vx = 0.0
    prev_vy = 0.0
    prev_vz = 0.0
    max_acc = 1.5  # m/s^2 cap per axis

    while True:
        loc = vehicle.location.local_frame
        currN = loc.north
        currE = loc.east
        currD = loc.down

        if currN is None:
            print("Local frame lost, waiting...")
            time.sleep(0.2)
            continue

        # delta in NED
        dN = targetN - currN
        dE = targetE - currE
        dD = targetD - currD
        dist = math.sqrt(dN**2 + dE**2 + dD**2)

        # status print
        print(f"Distance remaining: {dist:.2f} m")

        # arrived?
        if dist <= arrive_tol:
            print("Reached target (within tolerance). Stopping.")
            for _ in range(6):
                send_ned_velocity(0, 0, 0)
                time.sleep(0.05)
            break

        # timeout safety
        if time.time() - t0 > max_time:
            print("Timeout reached: stopping.")
            for _ in range(6):
                send_ned_velocity(0, 0, 0)
                time.sleep(0.05)
            break

        # proportional desired speed (clamped)
        desired_speed = k_p * dist
        desired_speed = max(speed_min, min(speed_max, desired_speed))

        # NED velocity components (world frame)
        dirN = dN / dist
        dirE = dE / dist
        dirD = dD / dist
        vx_cmd = dirN * desired_speed
        vy_cmd = dirE * desired_speed
        vz_cmd = dirD * desired_speed  # down positive

        # accel limiting (per axis)
        dvx = vx_cmd - prev_vx
        dvy = vy_cmd - prev_vy
        dvz = vz_cmd - prev_vz
        max_dv = max_acc * dt
        if abs(dvx) > max_dv:
            vx_cmd = prev_vx + math.copysign(max_dv, dvx)
        if abs(dvy) > max_dv:
            vy_cmd = prev_vy + math.copysign(max_dv, dvy)
        if abs(dvz) > max_dv:
            vz_cmd = prev_vz + math.copysign(max_dv, dvz)

        # debug optional: uncomment for more info
        # print(f"cmd vx={vx_cmd:.2f}, vy={vy_cmd:.2f}, vz={vz_cmd:.2f}")

        # send world-frame velocity
        send_ned_velocity(vx_cmd, vy_cmd, vz_cmd)

        prev_vx = vx_cmd
        prev_vy = vy_cmd
        prev_vz = vz_cmd

        time.sleep(dt)

    # hold position briefly
    for _ in range(8):
        send_ned_velocity(0, 0, 0)
        time.sleep(0.05)


# ---------------- MISSION ---------------- #
print("Arming and taking off...")
arm_and_takeoff(20)

# small pause to let EKF settle
time.sleep(2)

# Get current local frame AFTER takeoff
loc = vehicle.location.local_frame
while loc.north is None:
    print("Waiting for EKF local frame (post-takeoff)...")
    time.sleep(0.2)
    loc = vehicle.location.local_frame

startN = loc.north
startE = loc.east
startD = loc.down
print(f"Start EKF Position: N={startN:.3f}, E={startE:.3f}, D={startD:.3f}")

# End target (relative example)
endN = startN - 10
endE = startE - 20
endD = startD + 10

# Run the velocity-based move (tune speed_max / k_p as needed)
move_to_point_with_velocity(endN, endE, endD,
                            speed_max=10.0,   # try 2-3 m/s first for SITL
                            speed_min=0.2,
                            k_p=0.6,
                            arrive_tol=0.6,
                            rate_hz=10.0,
                            max_time=90.0)

print("\nLanding...")
vehicle.mode = VehicleMode("LAND")

while vehicle.armed:
    print(" Alt:", vehicle.location.global_relative_frame.alt)
    time.sleep(1)

print("Mission Complete. Vehicle Disarmed.")
vehicle.close()
