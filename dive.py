#!/usr/bin/env python3
"""
Hybrid Cruise -> Lock -> Ballistic Dive demo for DroneKit + ArduPilot SITL (Gazebo).

Usage:
    python3 hybrid_dive.py --connect udp:127.0.0.1:14550

Phases:
  1. Takeoff to cruise altitude
  2. PID-guided cruise to pre-dive hold point (world-frame velocity)
  3. Lock / stabilize and yaw-align toward target
  4. Ballistic dive: continuous high-speed velocity command toward target until impact distance reached
  5. Safe-stop / hold or land

Tune these values before a real run.
"""

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time, argparse, math, sys, signal

# ---------------- CONFIG ---------------- #
TAKEOFF_ALT = 40.0         # Cruise altitude (meters)
HOLD_OFFSET = 10.0         # Meters away from target along approach vector to hold before dive
ARRIVE_TOL = 1.0           # Tolerance for cruise hold (m)
CMD_RATE_HZ = 15.0        # Command frequency (Hz)
CRUISE_MAX = 12.0          # Cruise max horizontal speed (m/s) - safe initial
CRUISE_MIN = 1.0
CRUISE_KP = 0.5
CRUISE_MAX_ACC = 6.0      # m/s^2 for cruise transitions
PID_TUNING = {
    "N": {"Kp": 0.9, "Ki": 0.05, "Kd": 0.25, "imax": 4.0},
    "E": {"Kp": 0.9, "Ki": 0.05, "Kd": 0.25, "imax": 4.0},
    "D": {"Kp": 0.9, "Ki": 0.03, "Kd": 0.2,  "imax": 3.0}
}

# Ballistic dive parameters
DIVE_SPEED = 22.0          # horizontal component magnitude (m/s) — adjust to reach ~80 km/h
DIVE_VZ_MAX = 10.0         # max downward speed (m/s)
DIVE_MIN_ALT = 2.0         # stop dive at this altitude (m above ground)
DIVE_MAX_TIME = 30.0       # safety timeout for dive (s)

# Safety/timeouts
NAV_TIMEOUT = 120.0
FULL_TIMEOUT = 300.0

# ---------------- CONNECT ---------------- #
parser = argparse.ArgumentParser()
parser.add_argument("--connect", help="connection string")
args = parser.parse_args()
connection_string = args.connect if args.connect else "udp:127.0.0.1:14550"
print("Connecting to:", connection_string)
vehicle = connect(connection_string, wait_ready=True, timeout=60)

# Safe stop helper
def safe_stop_and_exit(reason="Exit"):
    try:
        print("\n[SAFE STOP] ", reason)
        for _ in range(8):
            send_ned_velocity(0,0,0)
            time.sleep(0.05)
        vehicle.mode = VehicleMode("LOITER")
    except Exception:
        pass
    try:
        vehicle.close()
    except Exception:
        pass
    sys.exit(0)

def _signal_handler(sig, frame):
    safe_stop_and_exit("KeyboardInterrupt")
signal.signal(signal.SIGINT, _signal_handler)

# ---------------- ARM + TAKEOFF ---------------- #
def arm_and_takeoff(alt):
    print("Waiting for vehicle to become armable...")
    while not vehicle.is_armable:
        time.sleep(0.5)
    print("Arming...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(0.5)
    print(f"Taking off to {alt} m")
    vehicle.simple_takeoff(alt)
    while True:
        curr = vehicle.location.global_relative_frame.alt
        print(" Alt:", round(curr,3))
        if curr >= alt * 0.95:
            print("Reached takeoff altitude.")
            break
        time.sleep(0.6)

# ---------------- SEND LOCAL_NED VELOCITY ---------------- #
def send_ned_velocity(vx, vy, vz):
    """
    Send velocity in LOCAL_NED frame (world-fixed).
    vx = North, vy = East, vz = Down (m/s)
    type_mask = 3527 => velocity only
    """
    # clamp safety
    max_h = max(CRUISE_MAX, DIVE_SPEED, 1.0) * 1.2
    max_v = max(DIVE_VZ_MAX, 5.0)
    vx = max(-max_h, min(max_h, vx))
    vy = max(-max_h, min(max_h, vy))
    vz = max(-max_v, min(max_v, vz))

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        3527,
        0,0,0,
        vx, vy, vz,
        0,0,0,
        0,0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# ---------------- Simple Helper: yaw to face a NED vector ---------------- #
def yaw_to_face(targetN, targetE):
    loc = vehicle.location.local_frame
    if loc.north is None:
        return

    currN = loc.north
    currE = loc.east

    dN = targetN - currN
    dE = targetE - currE

    desired_yaw = math.atan2(dE, dN)  # radians
    yaw_deg = math.degrees(desired_yaw)

    print(f"Yaw align to {yaw_deg:.1f} deg")

    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        yaw_deg,     # param1: yaw angle
        5,           # param2: yaw speed (deg/s)
        1,           # param3: direction
        0,           # param4: absolute yaw
        0,           # param5 (unused)
        0,           # param6 (unused)
        0            # param7 (required)
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()
    time.sleep(1)

# ---------------- PID navigator (world-frame LOCAL_NED velocities) ---------------- #
class AxisPID:
    def __init__(self, Kp, Ki, Kd, imax):
        self.Kp = Kp; self.Ki = Ki; self.Kd = Kd; self.imax = imax
        self.integral = 0.0; self.prev_err = None
    def reset(self):
        self.integral = 0.0; self.prev_err = None
    def update(self, err, dt):
        if self.prev_err is None:
            derivative = 0.0
        else:
            derivative = (err - self.prev_err) / dt if dt>0 else 0.0
        self.integral += err * dt
        if abs(self.integral) > self.imax:
            self.integral = math.copysign(self.imax, self.integral)
        out = self.Kp*err + self.Ki*self.integral + self.Kd*derivative
        self.prev_err = err
        return out

def cruise_to_point(targetN, targetE, targetD,
                    arrive_tol=ARRIVE_TOL,
                    rate_hz=CMD_RATE_HZ,
                    timeout=NAV_TIMEOUT):
    """
    PID guided cruise to absolute EKF-local point using LOCAL_NED velocities.
    """
    print(f"[CRUISE] target N:{targetN:.2f}, E:{targetE:.2f}, D:{targetD:.2f}")
    loc = vehicle.location.local_frame
    start_wait = time.time()
    while loc.north is None:
        if time.time() - start_wait > 10:
            raise RuntimeError("local_frame not ready")
        print("Waiting for EKF local frame...")
        time.sleep(0.2)
        loc = vehicle.location.local_frame

    pidN = AxisPID(**PID_TUNING["N"])
    pidE = AxisPID(**PID_TUNING["E"])
    pidD = AxisPID(**PID_TUNING["D"])

    dt = 1.0 / rate_hz
    t0 = time.time()
    prev_vx = prev_vy = prev_vz = 0.0
    max_dv = CRUISE_MAX_ACC * dt

    while True:
        loc = vehicle.location.local_frame
        currN = loc.north; currE = loc.east; currD = loc.down
        if currN is None:
            time.sleep(0.1); continue

        dN = targetN - currN; dE = targetE - currE; dD = targetD - currD
        dist = math.sqrt(dN*dN + dE*dE + dD*dD)
        print(f"[CRUISE] dist {dist:.2f} m")

        if dist <= arrive_tol:
            print("[CRUISE] arrived")
            for _ in range(6):
                send_ned_velocity(0,0,0); time.sleep(0.05)
            return True

        if time.time() - t0 > timeout:
            print("[CRUISE] timeout"); return False

        # PID outputs (could be big)
        vx_u = pidN.update(dN, dt)
        vy_u = pidE.update(dE, dt)
        vz_u = pidD.update(dD, dt)

        # scale horizontal magnitude to CRUISE range using proportional scaling
        mag = math.sqrt(vx_u*vx_u + vy_u*vy_u)
        if mag > 0.001:
            scale = CRUISE_KP * dist
            # limit to range
            desire = max(CRUISE_MIN, min(CRUISE_MAX, scale))
            factor = desire / mag
        else:
            factor = 0.0

        vx_cmd = vx_u * factor
        vy_cmd = vy_u * factor
        vz_cmd = vz_u * factor

        # accel limiting
        dvx = vx_cmd - prev_vx; dvy = vy_cmd - prev_vy; dvz = vz_cmd - prev_vz
        if abs(dvx) > max_dv: vx_cmd = prev_vx + math.copysign(max_dv, dvx)
        if abs(dvy) > max_dv: vy_cmd = prev_vy + math.copysign(max_dv, dvy)
        if abs(dvz) > max_dv: vz_cmd = prev_vz + math.copysign(max_dv, dvz)

        prev_vx, prev_vy, prev_vz = vx_cmd, vy_cmd, vz_cmd

        # debug:
        # print(f"[CRUISE] cmd vx={vx_cmd:.2f}, vy={vy_cmd:.2f}, vz={vz_cmd:.2f}")
        send_ned_velocity(vx_cmd, vy_cmd, vz_cmd)
        time.sleep(dt)

# ---------------- Ballistic dive ---------------- #
def ballistic_dive(targetN, targetE, targetD,
                   dive_speed=DIVE_SPEED, 
                   dive_vz_max=DIVE_VZ_MAX,
                   min_alt=DIVE_MIN_ALT, 
                   max_time=DIVE_MAX_TIME):

    print(f"[DIVE] Launching ballistic dive toward N:{targetN:.2f}, E:{targetE:.2f}, D:{targetD:.2f}")
    t0 = time.time()
    dt = 1.0 / CMD_RATE_HZ

    while True:
        loc = vehicle.location.local_frame
        if loc.north is None:
            time.sleep(0.02)
            continue

        currN = loc.north
        currE = loc.east
        currD = loc.down

        alt = -currD   # altitude above ground (positive)

        # STOP when drone reaches ground / target altitude
        if alt <= min_alt:
            print(f"[DIVE] reached min alt {alt:.2f} m — stopping dive")
            send_ned_velocity(0,0,0)
            return True

        if time.time() - t0 > max_time:
            print("[DIVE] dive timeout")
            send_ned_velocity(0,0,0)
            return False

        # fixed direction vector from drone → target
        dN = targetN - currN
        dE = targetE - currE

        horiz = math.sqrt(dN*dN + dE*dE)
        if horiz < 0.001:
            vx = 0; vy = 0
        else:
            vx = (dN / horiz) * dive_speed
            vy = (dE / horiz) * dive_speed

        # FORCE FAST DOWNWARD VERTICAL SPEED
        vz = dive_vz_max   # positive = DOWN in NED

        print(f"[DIVE] alt={alt:.2f} vx={vx:.2f} vy={vy:.2f} vz={vz:.2f}")
        send_ned_velocity(vx, vy, vz)

        time.sleep(dt)


# ---------------- MAIN ---------------- #
if __name__ == "__main__":
    try:
        start_time = time.time()
        print("Arming and taking off...")
        arm_and_takeoff(TAKEOFF_ALT)

        # wait for local frame
        time.sleep(1.5)
        loc = vehicle.location.local_frame
        while loc.north is None:
            print("Waiting for EKF local frame (post-takeoff)...")
            time.sleep(0.2); loc = vehicle.location.local_frame

        startN = loc.north; startE = loc.east; startD = loc.down
        print(f"Start EKF Pos: N={startN:.3f}, E={startE:.3f}, D={startD:.3f}")

        # Define ground target relative to start (example)
        # You can replace targetN/E/D with absolute EKF coordinates or compute from lat/lon
        targetN = startN - 40.0   # 40m south in example
        targetE = startE - 80.0   # 80m west in example
        # For target down: ground is near startD; we assume same ground => target down = startD (or 0)
        targetD = startD

        # Compute pre-dive hold point (HOLD_OFFSET meters back along approach vector)
        # Approach vector = from hold -> target
        vecN = targetN - startN
        vecE = targetE - startE
        vec_horiz = math.sqrt(vecN*vecN + vecE*vecE) + 1e-8
        # hold point = target - unit_vec * HOLD_OFFSET
        holdN = targetN - (vecN / vec_horiz) * HOLD_OFFSET
        holdE = targetE - (vecE / vec_horiz) * HOLD_OFFSET
        holdD = startD  # maintain cruise altitude (~down stays same)

        print(f"Target N:{targetN:.2f}, E:{targetE:.2f}. Hold at N:{holdN:.2f}, E:{holdE:.2f}")

        # Cruise to hold point
        ok = cruise_to_point(holdN, holdE, holdD, arrive_tol=ARRIVE_TOL, rate_hz=CMD_RATE_HZ, timeout=NAV_TIMEOUT)
        if not ok:
            safe_stop_and_exit("Failed to reach hold point")

        # Lock and yaw-align toward target
        print("[HOLD] Stabilizing and yaw-aligning to target")
        yaw_to_face(targetN, targetE)
        # short stabilization hold
        for _ in range(8):
            send_ned_velocity(0,0,0); time.sleep(0.1)

        # Final check: if we have ample altitude to dive
        current_alt = -vehicle.location.local_frame.down
        print(f"[HOLD] altitude {current_alt:.2f} m")

        if current_alt <= 5.0:
            print("[HOLD] altitude too low to dive safely — aborting dive")
            safe_stop_and_exit("Low altitude")

        # Launch ballistic dive
        ok = ballistic_dive(targetN, targetE, targetD,
                            dive_speed=DIVE_SPEED,
                            dive_vz_max=DIVE_VZ_MAX,
                            min_alt=DIVE_MIN_ALT,
                            max_time=DIVE_MAX_TIME)
        if not ok:
            print("[MAIN] Dive aborted or timed out")

        # Post-dive: try to recover to LOITER or LAND
        print("[MAIN] Post-dive: commanding LOITER then LAND")
        vehicle.mode = VehicleMode("LOITER")
        time.sleep(1.0)
        vehicle.mode = VehicleMode("LAND")
        # wait for landing
        t_land_start = time.time()
        while vehicle.armed and (time.time()-t_land_start) < 60:
            print(" Alt:", round(vehicle.location.global_relative_frame.alt,3))
            time.sleep(1.0)

        print("Mission complete. Closing vehicle.")
        vehicle.close()
    except Exception as e:
        print("Exception:", e)
        safe_stop_and_exit("Exception")
