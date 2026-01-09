# Mission Configuration Parameters

# Altitude and Velocity Targets
TAKEOFF_ALTITUDE = 40.0  # meters - entry altitude for attack
ASCENT_VELOCITY = 8.0     # m/s - rapid altitude gain
CRUISE_VELOCITY = 6.0     # m/s - forward velocity during transit
CRUISE_ALTITUDE = 40.0    # meters - maintain altitude during transit

# Phase Timing
CRUISE_DURATION = 7.0     # seconds - time to cruise forward and align

# Dive Parameters
DIVE_PITCH_ANGLE = 50.0   # degrees nose-down (80° stretch goal after validation)
DIVE_THRUST = 0.9         # 0-1 thrust during dive
TARGET_TERMINAL_VELOCITY = 10.0  # m/s target velocity

# PX4 Parameters to Override
PX4_PARAMS = {
    'MPC_TILTMAX_AIR': 60.0,      # Max tilt in AUTO/POSCTL (default 45°)
    'MPC_MAN_TILT_MAX': 60.0,     # Manual tilt limit
    'MC_PITCHRATE_MAX': 300.0,    # Max pitch rate (deg/s)
}

# Connection
SYSTEM_ADDRESS = "udp://:14540"  # MAVSDK connection string
