#!/usr/bin/env python3

import asyncio
import json
import math
import time
import re
from typing import Tuple, List, Dict, Any

MAVLINK_CONN = "udp:127.0.0.1:14550"   # SITL connection (change to your connection)
OLLAMA_MODEL = "llama3.2"              # function-calling capable model
MIN_ALT = 0.2
MAX_ALT = 120.0                       # adjust per your environment
COMMAND_TIMEOUT = 60                  # seconds for position acceptance loops
NED_POSITION_ACCEPTANCE = 1.5         # meters tolerance when checking reached position

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

class DroneController:
    def __init__(self, conn_str=MAVLINK_CONN):
        print(f"[DRONE] Connecting to {conn_str} ...")
        self.vehicle = connect(conn_str, wait_ready=True, heartbeat_timeout=60)
        print("[DRONE] Connected.")
        time.sleep(1)
        # Use global_relative_frame.alt for home altitude reference
        loc = self.vehicle.location.global_frame
        h_alt = getattr(self.vehicle.location.global_relative_frame, 'alt', 0.0)
        self.home = (loc.lat, loc.lon, h_alt)
        print("[DRONE] Home (lat,lon,alt):", self.home)
        self._ensure_local_frame()

    def _ensure_local_frame(self, timeout=10):
        t0 = time.time()
        while time.time() - t0 < timeout:
            lf = getattr(self.vehicle.location, 'local_frame', None)
            if lf is not None and getattr(lf, 'north', None) is not None:
                return True
            time.sleep(0.5)
        print("[DRONE] Warning: local_frame not available - using geographic approximations.")
        return False

    @staticmethod
    def latlon_to_north_east_meters(lat1, lon1, lat2, lon2) -> Tuple[float, float]:
        lat_rad = math.radians(lat1)
        m_per_deg_lat = 111132.92 - 559.82 * math.cos(2 * lat_rad) + 1.175 * math.cos(4 * lat_rad)
        m_per_deg_lon = 111412.84 * math.cos(lat_rad) - 93.5 * math.cos(3 * lat_rad)
        north = (lat2 - lat1) * m_per_deg_lat
        east = (lon2 - lon1) * m_per_deg_lon
        return north, east

    def send_position_target_local_ned(self, north, east, down, yaw=None, duration=0.0):
        MAV_TYPEMASK_POS_ONLY = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                                 mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            MAV_TYPEMASK_POS_ONLY,
            float(north), float(east), float(down),
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
        if duration and duration > 0:
            t0 = time.time()
            while time.time() - t0 < duration:
                time.sleep(1)
                self.vehicle.send_mavlink(msg)
                self.vehicle.flush()

    def get_global(self) -> Tuple[float, float, float]:
        lat = self.vehicle.location.global_frame.lat
        lon = self.vehicle.location.global_frame.lon
        alt = getattr(self.vehicle.location.global_relative_frame, 'alt', 0.0)
        return lat, lon, alt

    def get_local_ned_from_home(self) -> Tuple[float, float, float]:
        lf = getattr(self.vehicle.location, 'local_frame', None)
        if lf is not None and getattr(lf, 'north', None) is not None:
            try:
                return float(lf.north), float(lf.east), float(lf.down)
            except Exception:
                pass
        lat, lon, alt = self.get_global()
        h_lat, h_lon, h_alt = self.home
        north, east = self.latlon_to_north_east_meters(h_lat, h_lon, lat, lon)
        down = -(alt - h_alt)
        return north, east, down

    def arm_and_takeoff(self, target_alt):
        target_alt = float(target_alt)
        if target_alt < MIN_ALT or target_alt > MAX_ALT:
            return {"ok": False, "error": "altitude_out_of_bounds"}
        print("[DRONE] Pre-arm checks")
        t0 = time.time()
        while not self.vehicle.is_armable:
            if time.time() - t0 > 20:
                return {"ok": False, "error": "not_armable"}
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
        print(f"[DRONE] MoveTo NED target (n={target_north:.2f}, e={target_east:.2f}, d={target_down:.2f})")
        self.send_position_target_local_ned(target_north, target_east, target_down, duration=1.0)
        if not wait:
            return {"ok": True}
        t0 = time.time()
        last_dist = None
        while time.time() - t0 < timeout:
            cur_n, cur_e, cur_d = self.get_local_ned_from_home()
            dist = math.sqrt((cur_n - target_north)**2 + (cur_e - target_east)**2 + (cur_d - target_down)**2)
            last_dist = dist
            print(f"  Dist to target: {dist:.2f} m  (cur_n={cur_n:.2f}, cur_e={cur_e:.2f}, cur_d={cur_d:.2f})")
            if dist <= NED_POSITION_ACCEPTANCE:
                return {"ok": True, "dist": dist}
            time.sleep(1)
        return {"ok": False, "error": "move_timeout", "last_dist": last_dist}

    def goto_latlon_via_ned(self, lat, lon, alt):
        h_lat, h_lon, h_alt = self.home
        north, east = self.latlon_to_north_east_meters(h_lat, h_lon, lat, lon)
        down = -(alt - h_alt)
        return self.move_to_ned(north, east, down)

    def hold_position(self, duration_seconds):
        cur_n, cur_e, cur_d = self.get_local_ned_from_home()
        print(f"[DRONE] Holding position at (n={cur_n:.2f}, e={cur_e:.2f}, d={cur_d:.2f}) for {duration_seconds}s")
        t0 = time.time()
        while time.time() - t0 < duration_seconds:
            self.send_position_target_local_ned(cur_n, cur_e, cur_d, duration=0.8)
            time.sleep(0.8)
        return {"ok": True}

    def rtl_and_land(self):
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
            if getattr(self.vehicle.location.global_relative_frame, 'alt', 0.0) <= 0.2:
                return {"ok": True}
            if time.time() - t0 > COMMAND_TIMEOUT:
                return {"ok": False, "error": "land_timeout"}
            time.sleep(1)

    def close(self):
        print("[DRONE] Closing connection...")
        self.vehicle.close()



# --------------------------- LLM INTERFACE SETUP


import ollama

TOOLS = [
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
            "description": "Move by a relative NED offset by default (meters). Use north,east,down (down positive). If you want absolute NED, include 'absolute': true.",
            "parameters": {
                "type": "object",
                "properties": {
                    "north": {"type": "number"},
                    "east": {"type": "number"},
                    "down": {"type": "number"},
                    "absolute": {"type": "boolean"}
                },
                "required": ["north", "east", "down"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "goto_latlon",
            "description": "Navigate to a global lat/lon at specific altitude (meters).",
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
            "You are a STRICT COMMAND PARSER. Output ONLY JSON tool_calls (function-calling style).\n"
            "Available functions: takeoff, move_ned, goto_latlon, hold, rtl, land.\n"
            "\nIMPORTANT RULES:\n"
            "1) If the user mentions TAKEOFF, ALTITUDE, HEIGHT, or 'meters', ALWAYS include a takeoff tool_call FIRST in the sequence.\n"
            "2) If the user gives relative horizontal moves (e.g. 'fly east 20m') and the drone is NOT airborne, include a takeoff step first.\n"
            "3) Preserve ALL actions in sequence — do not drop earlier steps.\n"
            "4) Use proper JSON types (numbers for numeric fields). Do not output plain text — only the function_call JSON.\n"
            "If input is ambiguous, ask a short clarifying question as a normal assistant message (but prefer to return structured tool_calls)."
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
    )
    print("\n[LLM RAW OUTPUT]\n", response, "\n")
    msg = response.get("message", {})
    if msg.get("tool_calls"):
        return msg["tool_calls"], None
    return None, msg.get("content", "")

# ---------------------------
# HELPERS
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
# FORMAT & AUTO-PATCH LOGIC
# ---------------------------
def format_proposed_sequence(tool_calls: List[Dict[str, Any]], drone: DroneController) -> Tuple[str, List[Dict[str, Any]]]:
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
            # parse numeric values safely (LLM sometimes returns strings)
            def _tof(x, default=0.0):
                try:
                    return float(x)
                except Exception:
                    return float(default)
            n = _tof(args.get("north", 0.0))
            e = _tof(args.get("east", 0.0))
            d = _tof(args.get("down", 0.0))
            absolute_flag = bool(args.get("absolute", False))

            # AUTO ALTITUDE SAFETY: if down is zero or None -> maintain current altitude
            if d == 0:
                try:
                    cur_n, cur_e, cur_d = drone.get_local_ned_from_home()
                    d = cur_d
                except Exception:
                    # fallback to -1.0m to be safe
                    d = -1.0

            # Convert semantics: default -> treat provided north/east/down as RELATIVE offsets
            # If the tool explicitly sets absolute=True then treat as absolute NED coordinates
            if absolute_flag:
                target_n = n
                target_e = e
                target_d = d
            else:
                # relative: target = current + delta
                try:
                    cur_n, cur_e, cur_d = drone.get_local_ned_from_home()
                    target_n = cur_n + n
                    target_e = cur_e + e
                    # for down: if LLM provided delta (e.g., down = -5 means go up 5) we treat as relative
                    # but most LLM outputs will have down=0 meaning "keep altitude" and we've already set d=cur_d
                    # If LLM intended a relative altitude change, they should provide a non-zero down.
                    # We'll interpret small values as relative deltas if absolute_flag is False and the magnitude is reasonable.
                    # To keep it simple: treat provided d as delta if absolute_flag False AND original arg 'down' was nonzero.
                    orig_down_arg = args.get("down", None)
                    if orig_down_arg is not None and float(orig_down_arg) != 0:
                        # treat provided d as delta
                        target_d = cur_d + d
                    else:
                        target_d = d
                except Exception:
                    target_n = n
                    target_e = e
                    target_d = d

            parts.append(f"move_ned n={target_n:.2f} e={target_e:.2f} d={target_d:.2f}")
            seq.append({"cmd":"move_ned","north":target_n,"east":target_e,"down":target_d})

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
    print("\n[SYSTEM] Starting Drone NED LLM Pipeline (auto-execute).")
    drone = DroneController()

    print("\nExamples you can type:")
    print("  take off to 8 meters then fly east 20 meters")
    print("  fly 3 meters north")
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
                print("[LLM] No structured tool_calls. Fallback:", fallback)
                print("Please rephrase or give a clearer command.")
                continue

            # Build parsed sequence (this will also auto-fix altitude and apply relative semantics)
            confirm_string, parsed_seq = format_proposed_sequence(tool_calls, drone)
            print("\n[LLM → PARSED SEQUENCE]\n", confirm_string)
            # Execute sequence immediately
            for step in parsed_seq:
                cmd = step["cmd"]
                if cmd == "takeoff":
                    alt = step["altitude"]
                    print(f"[EXECUTE] takeoff -> {alt} m")
                    res = drone.arm_and_takeoff(alt)
                    print("[RESULT]", res)
                    if not res.get("ok"):
                        print("[ERROR] takeoff failed. Aborting sequence.")
                        break
                elif cmd == "move_ned":
                    n = step["north"]
                    e = step["east"]
                    d = step["down"]

                    # --- MINIMAL SAFETY: ensure drone is airborne before horizontal move ---
                    cur_alt = getattr(drone.vehicle.location.global_relative_frame, "alt", None)
                    if cur_alt is None:
                        cur_alt = 0.0
                    # if drone below 1.0 m, force an auto-takeoff to safe height (2 m) before horizontal move
                    if cur_alt < 1.0:
                        print(f"[AUTO-SAFETY] Drone altitude {cur_alt:.2f} m — auto takeoff to 2.0 m before horizontal move.")
                        ok = drone.arm_and_takeoff(2.0)
                        print("[AUTO-SAFETY] takeoff result:", ok)
                        if not ok.get("ok"):
                            print("[ERROR] Auto-takeoff failed. Aborting move_ned.")
                            break

                    # execute move (move_to_ned expects absolute local NED)
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
                    if alt < MIN_ALT or alt > MAX_ALT:
                        print("[VALIDATOR] goto altitude out of bounds.")
                        break
                    print(f"[EXECUTE] goto_latlon -> lat:{lat} lon:{lon} alt:{alt}")
                    res = drone.goto_latlon_via_ned(lat, lon, alt)
                    print("[RESULT]", res)
                    if not res.get("ok"):
                        print("[ERROR] goto_latlon failed or timed out. Aborting sequence.")
                        break
                elif cmd == "hold":
                    secs = step["seconds"]
                    print(f"[EXECUTE] hold -> {secs}s")
                    res = drone.hold_position(secs)
                    print("[RESULT]", res)
                elif cmd == "rtl":
                    print("[EXECUTE] rtl")
                    res = drone.rtl_and_land()
                    print("[RESULT]", res)
                    # sequence ends on RTL/land
                    break
                elif cmd == "land":
                    print("[EXECUTE] land")
                    res = drone.land()
                    print("[RESULT]", res)
                    break
                else:
                    print(f"[WARN] Unknown command {cmd}, skipping.")
            print("[SEQUENCE] Done.\n")

    except KeyboardInterrupt:
        print("\n[SHUTDOWN] KeyboardInterrupt received.")
    finally:
        drone.close()
        print("[SYSTEM] Shutdown complete.")

if __name__ == "__main__":
    asyncio.run(main_loop())
