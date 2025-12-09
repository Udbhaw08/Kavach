from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import sys
import warnings

warnings.filterwarnings("ignore", category=UserWarning)

# ---------------------------------------------------------
# STEP 1: CONNECT TO THE VEHICLE
# ---------------------------------------------------------
# Change connection string if needed: (udp:127.0.0.1:14551 recommended)
print("\nConnecting to vehicle...")
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True, timeout=60)

print(f"Connected to: {vehicle.version}")
print(f"Autopilot Firmware: {vehicle.version}")
print(f"GPS: {vehicle.gps_0}")
print(f"Battery: {vehicle.battery}")
print(f"System status: {vehicle.system_status.state}")
print(f"Mode: {vehicle.mode.name}\n")

# ---------------------------------------------------------
# STEP 2: ARM AND TAKEOFF
# ---------------------------------------------------------
def arm_and_takeoff(aTargetAltitude):
        time.sleep(1)


#Arm and take of to altitude of 5 meters
arm_and_takeoff(5)


# ---------------------------------------------------------
# STEP 3: DEFINE DESTINATION WAYPOINT
# ---------------------------------------------------------
# Use a location near your current position (SITL default: 35.363261, 149.165230)
target_location = LocationGlobalRelative(35.364000, 149.166000, 20)

print(f"Flying to target location: {target_location.lat}, {target_location.lon} at 20m altitude...")
vehicle.simple_goto(target_location)

# Monitor flight for ~20 seconds
for i in range(20):
    current_location = vehicle.location.global_relative_frame
    dist = math.sqrt((target_location.lat - current_location.lat)**2 +
                     (target_location.lon - current_location.lon)**2) * 1.113195e5
    print(f" [{i+1:02d}] Current Location: {current_location.lat:.6f}, {current_location.lon:.6f}, Alt: {current_location.alt:.2f} | Distance to target: {dist:.2f} m")
    time.sleep(1)

# ---------------------------------------------------------
# STEP 4: LAND
# ---------------------------------------------------------
print("\nInitiating Landing...")
vehicle.mode = VehicleMode("LAND")

while True:
    altitude = vehicle.location.global_relative_frame.alt
    print(f" Altitude: {altitude:.2f} m")
    if altitude <= 0.5:
        print("Landed successfully.")
        break
    time.sleep(1)

# ---------------------------------------------------------
# STEP 5: CLEANUP
# ---------------------------------------------------------
print("\nMission complete. Closing vehicle object.")
vehicle.close()
sys.exit(0)

