#!/usr/bin/env python3
"""
llm_ned_human_gate.py

- NED-only movement via SET_POSITION_TARGET_LOCAL_NED (MAV_FRAME_LOCAL_NED).
- Human-in-the-loop: LLM suggests a sequence of low-level commands (NED + actions).
  You must TYPE the exact confirmation string shown to actually execute them.
- Uses DroneKit + pymavlink message factory for precise MAVLink SET_POSITION_TARGET_LOCAL_NED calls.
- Uses Ollama AsyncClient (ollama package) for local LLM parsing (function-calling).
- Tested flow examples:
    -> "Take off to 8 meters, then fly east 20 meters."
    -> "Return to launch and land."
    -> "Fly to coordinates 18.5291, 73.8568 at 20 meters altitude."
    -> "Hold position for 20 seconds and then do RTL"

Requirements:
  pip install dronekit pymavlink ollama

Run:
  python3 llm_ned_human_gate.py
"""

import asyncio
import json
import math
import time
import re
from typing import Tuple, List, Dict, Any

# ---------------------------
# CONFIG
# ---------------------------
MAVLINK_CONN = "udp:127.0.0.1:14550"   # SITL connection (change to your connection)
OLLAMA_MODEL = "llama3.2"             # local model
MIN_ALT = 0.2
MAX_ALT = 120.0                       # adjust per your environment
COMMAND_TIMEOUT = 60                  # seconds for position acceptance loops
NED_POSITION_ACCEPTANCE = 1.5         # meters tolerance when checking reached position

# ---------------------------
# DRONEKIT & MAVLINK SETUP
# ---------------------------
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

class DroneController:
    def __init__(self, conn_str=MAVLINK_CONN):
        print(f"[DRONE] Connecting to {conn_str} ...")
        self.vehicle = connect(conn_str, wait_ready=True, heartbeat_timeout=60)
        print("[DRONE] Connected.")
        # Wait for a valid GPS/home fix if possible
        time.sleep(1)
        loc = self.vehicle.location.global_frame
        self.home = (loc.lat, loc.lon, getattr(loc, 'alt', 0.0))
        print("[DRONE] Home (lat,lon,alt):", self.home)
        # local_frame values become available after EKF/local position is up
        # we will attempt to wait for vehicle.location.local_frame if needed
        self._ensure_local_frame()

    def _ensure_local_frame(self, timeout=10):
        # Wait for local_frame to be available (vehicle.location.local_frame)
        t0 = time.time()
        while time.time() - t0 < timeout:
            lf = getattr(self.vehicle.location, 'local_frame', None)
            if lf is not None and lf.north is not None:
                return True
            time.sleep(0.5)
        # may still work with computed conversions; log a warning
        print("[DRONE] Warning: local_frame not available - conversions will use geographic approximations.")
        return False

    # ---------- helper: convert lat/lon delta to meters (approx)
    @staticmethod
    def latlon_to_north_east_meters(lat1, lon1, lat2, lon2) -> Tuple[float, float]:
        """
        Returns north_meters, east_meters from (lat1, lon1) -> (lat2, lon2)
        Approximate conversions (good for local distances).
        """
        # Earth radius approximations
        # 1 deg lat ~= 111319.5 m
        lat_rad = math.radians(lat1)
        m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat_rad) + 1.175 * math.cos(4 * lat_rad)
        m_per_deg_lon = 111412.84 * math.cos(lat_rad) - 93.5 * math.cos(3 * lat_rad)
        north = (lat2 - lat1) * m_per_deg_lat
        east = (lon2 - lon1) * m_per_deg_lon
        return north, east

    # ---------- build and send MAVLink SET_POSITION_TARGET_LOCAL_NED
    def send_position_target_local_ned(self, north, east, down, yaw=None, duration=0.0):
        """
        Send a position target in the LOCAL_NED frame.
        down: positive down (NED). To go to altitude A (meters above ground), set down = -A.
        type_mask will be set to use position only (ignore velocities/accelerations).
        """
        # type_mask: clear fields we want to use (0 = use), set ones we want to ignore (1)
        # Using position only => ignore vx,vy,vz,afx,afy,afz,yaw,yaw_rate
        # Bits: https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED
        MAV_TYPEMASK_POS_ONLY = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        # Use MAV_FRAME_LOCAL_NED = 1
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            MAV_TYPEMASK_POS_ONLY,
            north, east, down,   # x, y, z positions (north, east, down)
            0, 0, 0,             # vx, vy, vz
            0, 0, 0,             # afx, afy, afz
            0, 0                 # yaw, yaw_rate
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        if duration and duration > 0:
            # Re-send at 1Hz for the duration to increase chance of acceptance
            t0 = time.time()
            while time.time() - t0 < duration:
                time.sleep(1)
                self.vehicle.send_mavlink(msg)
                self.vehicle.flush()

    # ---------- utility: get current global and local approximations
    def get_global(self) -> Tuple[float, float, float]:
        lat = self.vehicle.location.global_frame.lat
        lon = self.vehicle.location.global_frame.lon
        alt = getattr(self.vehicle.location.global_relative_frame, 'alt', 0.0)
        return lat, lon, alt

    def get_local_ned_from_home(self) -> Tuple[float, float, float]:
        """
        Compute approximate current NED relative to home (north, east, down)
        using geographic approx. If vehicle.local_frame is available, prefer it.
        """
        lf = getattr(self.vehicle.location, 'local_frame', None)
        if lf is not None and getattr(lf, 'north', None) is not None:
            try:
                n = float(lf.north)
                e = float(lf.east)
                d = float(lf.down)
                return n, e, d
            except Exception:
                pass
        # fallback to approximate conversion from home lat/lon -> current lat/lon
        lat, lon, alt = self.get_global()
        h_lat, h_lon, h_alt = self.home
        north, east = self.latlon_to_north_east_meters(h_lat, h_lon, lat, lon)
        down = -(alt - h_alt)  # down positive = - (altitude above home)
        return north, east, down

    # ---------- high level actions
    def arm_and_takeoff(self, target_alt):
        target_alt = float(target_alt)
        if target_alt < MIN_ALT or target_alt > MAX_ALT:
            return {"ok": False, "error": "altitude_out_of_bounds"}
        print("[DRONE] Pre-arm checks")
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
        print("[DRONE] Arming motors")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        t0 = time.time()
        while not self.vehicle.armed and time.time() - t0 < 10:
            print(" Waiting for arming...")
            time.sleep(0.5)
        if not self.vehicle.armed:
            return {"ok": False, "error": "arm_failed"}
        print(f"[DRONE] Simple takeoff to {target_alt} m")
        self.vehicle.simple_takeoff(target_alt)
        # Wait until altitude reached
        t0 = time.time()
        while True:
            alt = getattr(self.vehicle.location.global_relative_frame, 'alt', 0.0)
            print(f" Current alt: {alt:.2f} m")
            if alt >= target_alt * 0.95:
                print("[DRONE] Target altitude reached.")
                return {"ok": True, "altitude": alt}
            if time.time() - t0 > COMMAND_TIMEOUT:
                return {"ok": False, "error": "takeoff_timeout", "alt": alt}
            time.sleep(1)

    def move_to_ned(self, target_north, target_east, target_down, wait=True, timeout=COMMAND_TIMEOUT):
        """
        Move to an absolute local NED coordinate (relative to home).
        target_down: positive-down (NED). For altitude A above home, use down = -A.
        """
        print(f"[DRONE] MoveTo NED target (n={target_north:.2f}, e={target_east:.2f}, d={target_down:.2f})")
        self.send_position_target_local_ned(target_north, target_east, target_down, duration=1.0)
        if not wait:
            return {"ok": True}
        # wait until within acceptance radius
        t0 = time.time()
        while time.time() - t0 < timeout:
            cur_n, cur_e, cur_d = self.get_local_ned_from_home()
            dist = math.sqrt((cur_n - target_north)**2 + (cur_e - target_east)**2 + (cur_d - target_down)**2)
            print(f"  Dist to target: {dist:.2f} m  (cur_n={cur_n:.2f}, cur_e={cur_e:.2f}, cur_d={cur_d:.2f})")
            if dist <= NED_POSITION_ACCEPTANCE:
                return {"ok": True, "dist": dist}
            time.sleep(1)
        return {"ok": False, "error": "move_timeout", "last_dist": dist}

    def goto_latlon_via_ned(self, lat, lon, alt):
        """
        Convert lat/lon/alt target to local NED relative to home and move using move_to_ned.
        alt: altitude above home reference (global_relative)
        """
        h_lat, h_lon, h_alt = self.home
        north, east = self.latlon_to_north_east_meters(h_lat, h_lon, lat, lon)
        down = -(alt - h_alt)  # NED positive down
        return self.move_to_ned(north, east, down)

    def hold_position(self, duration_seconds):
        # hold current NED position by re-sending current location as target for duration
        cur_n, cur_e, cur_d = self.get_local_ned_from_home()
        print(f"[DRONE] Holding position at (n={cur_n:.2f}, e={cur_e:.2f}, d={cur_d:.2f}) for {duration_seconds}s")
        t0 = time.time()
        while time.time() - t0 < duration_seconds:
            self.send_position_target_local_ned(cur_n, cur_e, cur_d, duration=0.8)
            time.sleep(0.8)
        return {"ok": True}

    def rtl_and_land(self):
        # Return to home NED = (0,0, -home_alt) and then land
        print("[DRONE] RTL → moving to home and landing")
        h_lat, h_lon, h_alt = self.home
        target_n, target_e, target_d = 0.0, 0.0, -h_alt
        res = self.move_to_ned(target_n, target_e, target_d)
        if not res.get("ok"):
            print("[DRONE] Warning: RTL move failed or timed out:", res)
        print("[DRONE] Switching to LAND mode")
        self.vehicle.mode = VehicleMode("LAND")
        return {"ok": True}

    def land(self):
        print("[DRONE] Land in place")
        self.vehicle.mode = VehicleMode("LAND")
        t0 = time.time()
        while True:
            if self.vehicle.location.global_relative_frame.alt <= 0.2:
                return {"ok": True}
            if time.time() - t0 > COMMAND_TIMEOUT:
                return {"ok": False, "error": "land_timeout"}
            time.sleep(1)

    def close(self):
        print("[DRONE] Closing connection...")
        self.vehicle.close()

# ---------------------------
# OLLAMA TOOLING
# ---------------------------
import ollama

TOOLS = [
    # high-level commands that LLM can produce
    {
        "type": "function",
        "function": {
            "name": "takeoff",
            "description": "Arm and takeoff to altitude (meters).",
            "parameters": {
                "type": "object",
                "properties": {
                    "altitude": {"type": "number"}
                },
                "required": ["altitude"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "move_ned",
            "description": "Move to an absolute local NED coordinate (meters). Use north,east,down (down positive).",
            "parameters": {
                "type": "object",
                "properties": {
                    "north": {"type": "number"},
                    "east": {"type": "number"},
                    "down": {"type": "number"}
                },
                "required": ["north", "east", "down"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "goto_latlon",
            "description": "Navigate to a global lat/lon at specific altitude (meters). Script will convert to NED and use NED control.",
            "parameters": {
                "type": "object",
                "properties": {
                    "lat": {"type": "number"},
                    "lon": {"type": "number"},
                    "altitude": {"type": "number"}
                },
                "required": ["lat", "lon", "altitude"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "hold",
            "description": "Hold current position for given seconds.",
            "parameters": {
                "type": "object",
                "properties": {
                    "seconds": {"type": "number"}
                },
                "required": ["seconds"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "rtl",
            "description": "Return to launch (home) and land.",
            "parameters": {"type": "object", "properties": {}}
        }
    },
    {
        "type": "function",
        "function": {
            "name": "land",
            "description": "Land in place now.",
            "parameters": {"type": "object", "properties": {}}
        }
    }
]

def build_system_prompt():
    return {
        "role": "system",
        "content":
        (
            "You are a STRICT COMMAND PARSER. You must output only JSON tool_calls (function-calling style).\n"
            "Available functions: takeoff, move_ned (abs local NED), goto_latlon, hold, rtl, land.\n"
            "All movement MUST be executed using local NED (SET_POSITION_TARGET_LOCAL_NED) by the controller.\n"
            "When converting lat/lon to NED, use home as the reference origin.\n"
            "If the user's instruction contains relative moves like 'fly east 20m' produce move_ned with the correct north/east offsets.\n"
            "Do not output natural language; output only the JSON function call structure.\n"
            "If input is ambiguous, ask for clarification."
        )
    }

async def ask_llm_for_toolcalls(user_text: str):
    client = ollama.AsyncClient()
    messages = [
        build_system_prompt(),
        {"role": "user", "content": user_text}
    ]
    response = await client.chat(
    model=OLLAMA_MODEL,
    messages=messages,
    tools=TOOLS,
    format="json"
)

    # response is a dict-like; we expect response["message"] with tool_calls
    print("\n[LLM RAW OUTPUT]\n", response, "\n")
    msg = response.get("message", {})
    if msg.get("tool_calls"):
        return msg["tool_calls"], None
    return None, msg.get("content", "")

# ---------------------------
# HELPERS FOR text -> numbers
# ---------------------------
word_to_num = {
    "zero":0,"one":1,"two":2,"three":3,"four":4,"five":5,"six":6,"seven":7,
    "eight":8,"nine":9,"ten":10,"twenty":20,"thirty":30
}

def extract_first_number(text: str):
    nums = re.findall(r"\d+\.?\d*", text)
    if nums:
        return float(nums[0])
    for w,v in word_to_num.items():
        if w in text.lower():
            return float(v)
    return None

# ---------------------------
# HUMAN-IN-THE-LOOP EXECUTION
# ---------------------------
def format_proposed_sequence(tool_calls: List[Dict[str, Any]]) -> Tuple[str, List[Dict[str, Any]]]:
    """
    Return a human-readable one-line confirmation string and the parsed sequence
    We'll format commands like:
      takeoff altitude=8; move_ned north=0 east=20 down=-8; hold seconds=20; rtl
    The exact string is required for execution.
    """
    parts = []
    seq = []
    for tc in tool_calls:
        fn = tc["function"]["name"]
        args = tc["function"].get("arguments", {})
        if fn == "takeoff":
            alt = float(args["altitude"])
            parts.append(f"takeoff altitude={alt}")
            seq.append({"cmd":"takeoff","altitude":alt})
        elif fn == "move_ned":
            n = float(args["north"])
            e = float(args["east"])
            d = float(args["down"])

            # ------------------------------
            # AUTO ALTITUDE SAFETY PATCH 🔥
            # ------------------------------
            # If user or LLM gives down = 0, assume drone should maintain its current altitude
            # Prevents unintended "ground-level" commands
            if d == 0:
                try:
                    # Ask drone controller for current NED altitude
                    cur_n, cur_e, cur_d = drone.get_local_ned_from_home()
                    d = cur_d   # keep same altitude
                except:
                    pass
            # ------------------------------

            parts.append(f"move_ned n={n:.2f} e={e:.2f} d={d:.2f}")
            seq.append({"cmd":"move_ned","north":n,"east":e,"down":d})

        elif fn == "goto_latlon":
            lat = float(args["lat"])
            lon = float(args["lon"])
            alt = float(args["altitude"])
            parts.append(f"goto_latlon lat={lat:.6f} lon={lon:.6f} alt={alt}")
            seq.append({"cmd":"goto_latlon","lat":lat,"lon":lon,"altitude":alt})
        elif fn == "hold":
            s = float(args["seconds"])
            parts.append(f"hold seconds={s}")
            seq.append({"cmd":"hold","seconds":s})
        elif fn == "rtl":
            parts.append("rtl")
            seq.append({"cmd":"rtl"})
        elif fn == "land":
            parts.append("land")
            seq.append({"cmd":"land"})
        else:
            parts.append(f"{fn} {json.dumps(args)}")
            seq.append({"cmd":fn,"args":args})
    one_line = " ; ".join(parts)
    return one_line, seq

async def main_loop():
    print("\n[SYSTEM] Starting Drone NED LLM Pipeline with HUMAN approval gate.")
    drone = DroneController()

    print("\nType natural commands like:")
    print("  take off to 8 meters then fly east 20 meters")
    print("  return to launch and land")
    print("  fly to coordinates 18.5291, 73.8568 at 20 meters altitude")
    print("  hold position for 20 seconds and then RTL")
    print("Type 'exit' to quit.\n")

    try:
        while True:
            user_text = input("Command> ").strip()
            if not user_text:
                continue
            if user_text.lower() in ("exit","quit"):
                break

            tool_calls, fallback = await ask_llm_for_toolcalls(user_text)

            if not tool_calls:
                print("[LLM] Did not return structured tool_calls. Fallback content:", fallback)
                # try to be helpful: try to extract simple patterns
                # but for safety require clarified input
                print("Please rephrase or provide explicit commands.")
                continue

            # Format the proposed low-level NED commands into a single confirmation string
            confirm_string, parsed_seq = format_proposed_sequence(tool_calls)
            print("\n[LLM → EXECUTING SEQUENCE]\n", confirm_string)
            parsed_seq = parsed_seq  # already parsed; no need to confirm

            # If confirmed, execute each parsed command in order
            for step in parsed_seq:
                cmd = step["cmd"]
                if cmd == "takeoff":
                    alt = step["altitude"]
                    print(f"[EXECUTE] takeoff → {alt} m")
                    res = drone.arm_and_takeoff(alt)
                    print("[RESULT]", res)
                    if not res.get("ok"):
                        print("[ERROR] takeoff failed or timed out. Aborting sequence.")
                        break
                elif cmd == "move_ned":
                    n = step["north"]
                    e = step["east"]
                    d = step["down"]
                    # move_to_ned expects absolute NED relative to home; the LLM toolmove_ned is defined as absolute local NED
                    print(f"[EXECUTE] move_ned → N:{n:.2f} E:{e:.2f} D:{d:.2f}")
                    res = drone.move_to_ned(n, e, d)
                    print("[RESULT]", res)
                    if not res.get("ok"):
                        print("[ERROR] move_ned failed or timed out. Aborting sequence.")
                        break
                elif cmd == "goto_latlon":
                    lat = step["lat"]
                    lon = step["lon"]
                    alt = step["altitude"]
                    # Validate altitude range before converting
                    if alt < MIN_ALT or alt > MAX_ALT:
                        print(f"[VALIDATOR] goto altitude {alt} out of bounds.")
                        break
                    print(f"[EXECUTE] goto_latlon → lat:{lat} lon:{lon} alt:{alt}")
                    res = drone.goto_latlon_via_ned(lat, lon, alt)
                    print("[RESULT]", res)
                    if not res.get("ok"):
                        print("[ERROR] goto_latlon failed or timed out. Aborting sequence.")
                        break
                elif cmd == "hold":
                    secs = step["seconds"]
                    print(f"[EXECUTE] hold → {secs}s")
                    res = drone.hold_position(secs)
                    print("[RESULT]", res)
                    # holding rarely fails; continue
                elif cmd == "rtl":
                    print("[EXECUTE] rtl")
                    res = drone.rtl_and_land()
                    print("[RESULT]", res)
                    # after rtl we break sequence as landed
                    break
                elif cmd == "land":
                    print("[EXECUTE] land")
                    res = drone.land()
                    print("[RESULT]", res)
                    break
                else:
                    print(f"[WARN] Unknown command {cmd}, skipping.")
            # end for sequence
            print("[SEQUENCE] Done.\n")

    except KeyboardInterrupt:
        print("\n[SHUTDOWN] KeyboardInterrupt received.")
    finally:
        drone.close()
        print("[SYSTEM] Shutdown complete.")

if __name__ == "__main__":
    asyncio.run(main_loop())
