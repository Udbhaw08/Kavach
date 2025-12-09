#!/usr/bin/env python3
"""
llm_takeoff_full.py
FULL WORKING IMPLEMENTATION:
 - Connects to ArduPilot SITL using DroneKit
 - Uses local LLM (Ollama) with function-calling
 - Parses user commands like: "takeoff 5 meters"
 - LLM outputs correct tool_call JSON
 - Script validates altitude
 - Executes DroneKit simple_takeoff()
"""

import asyncio
import json
import re
import time

# ---------------------------
# CONFIG — CHANGE IF NEEDED
# ---------------------------

MAVLINK_CONN = "udp:127.0.0.1:14550"   # your SITL stream
OLLAMA_MODEL = "llama3.2"             # or qwen2.5, mistral etc.
MIN_ALT = 0.2
MAX_ALT = 15.0

# ---------------------------
# DRONEKIT SETUP
# ---------------------------
from dronekit import connect, VehicleMode, LocationGlobalRelative

class DroneController:
    def __init__(self):
        print(f"[DRONE] Connecting to {MAVLINK_CONN} ...")
        self.vehicle = connect(MAVLINK_CONN, wait_ready=True, heartbeat_timeout=60)
        print("[DRONE] Connected.")
        loc = self.vehicle.location.global_frame
        self.home = (loc.lat, loc.lon, getattr(loc, 'alt', 0.0))
        print(" Home:", self.home)

    def takeoff(self, altitude):
        altitude = float(altitude)
        print(f"[DRONE] Takeoff request → {altitude} meters")

        if altitude < MIN_ALT or altitude > MAX_ALT:
            return {"ok": False, "error": "altitude_out_of_bounds"}

        print("[DRONE] Changing mode → GUIDED")
        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)

        print("[DRONE] Arming motors...")
        self.vehicle.armed = True
        time.sleep(2)

        if not self.vehicle.armed:
            return {"ok": False, "error": "arm_failed"}

        print("[DRONE] Taking off...")
        self.vehicle.simple_takeoff(altitude)

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            print(f" Alt: {alt:.2f} m")
            if alt >= altitude * 0.95:
                print("[DRONE] Target altitude reached.")
                return {"ok": True, "altitude": alt}
            time.sleep(1)

    def close(self):
        print("[DRONE] Closing connection...")
        self.vehicle.close()

# ---------------------------
# OLLAMA FUNCTION CALLING
# ---------------------------
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
    }
]

def build_system_prompt():
    return {
        "role": "system",
        "content":
        (
            "You are a STRICT COMMAND PARSER.\n"
            "You only output JSON tool_calls.\n"
            "Function available: takeoff(altitude).\n"
            "Altitude must be numeric (meters).\n"
            "Never output text outside JSON.\n"
            "If unclear, ask for clarification."
        )
    }

async def ask_llm_for_toolcall(user_text: str):
    client = ollama.AsyncClient()

    messages = [
        build_system_prompt(),
        {"role": "user", "content": user_text}
    ]

    response = await client.chat(
        model=OLLAMA_MODEL,
        messages=messages,
        tools=TOOLS
    )

    print("\n[LLM RAW OUTPUT]\n", response, "\n")

    msg = response["message"]
    if msg.get("tool_calls"):
        return msg["tool_calls"], None

    return None, msg.get("content", "")

# ---------------------------
# HELPERS FOR NUMBER NORMALIZATION
# ---------------------------
word_to_number = {
    "zero":0,"one":1,"two":2,"three":3,"four":4,"five":5,
    "six":6,"seven":7,"eight":8,"nine":9,"ten":10
}

def extract_altitude(text: str):
    # check direct numbers
    nums = re.findall(r"\d+\.?\d*", text)
    if nums:
        return float(nums[0])

    # check word numbers: "five", "three"
    for w, v in word_to_number.items():
        if w in text.lower():
            return float(v)

    return None

# ---------------------------
# MAIN LOOP
# ---------------------------
async def main():
    print("\n[SYSTEM] Starting Drone LLM Pipeline...")
    drone = DroneController() 
    print("\nType commands such as:")
    print("  takeoff 3")
    print("  please take off to five meters")
    print("  go up 7m\n")

    while True:
        user_text = input("Command> ").strip()
        if user_text.lower() in ("exit", "quit"):
            break
        if not user_text:
            continue

        tool_calls, fallback = await ask_llm_for_toolcall(user_text)

        altitude = None

        if tool_calls:
            # LLM returned tool_call JSON
            fn = tool_calls[0]["function"]["name"]
            args = tool_calls[0]["function"]["arguments"]

            print("[LLM] Tool requested:", fn, args)

            if fn != "takeoff":
                print("[IGNORED] This demo only supports takeoff()")
                continue

            altitude = args.get("altitude")

        else:
            print("[LLM] Did not return tool_calls, fallback to content analysis.")
            altitude = extract_altitude(fallback or user_text)
            print(" Extracted altitude:", altitude)

        if altitude is None:
            print("[ERROR] Could not determine altitude.")
            continue

        altitude = float(altitude)

        # VALIDATE
        if altitude < MIN_ALT or altitude > MAX_ALT:
            print(f"[VALIDATOR] Altitude {altitude} out of safe range ({MIN_ALT}-{MAX_ALT})")
            continue

        # EXECUTE
        print(f"[EXECUTING] Taking off to {altitude}m ...")
        result = drone.takeoff(altitude)
        print("[RESULT]", result)

    drone.close()
    print("[SYSTEM] Shutdown complete.")

if __name__ == "__main__":
    asyncio.run(main())
