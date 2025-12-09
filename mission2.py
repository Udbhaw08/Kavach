#!/usr/bin/env python3
import time
import math
import json
import sys
import argparse
import re
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
# Try importing ollama, handle if missing
try:
    import ollama
except ImportError:
    print("Error: 'ollama' library not found. Please install it using 'pip install ollama'")
    sys.exit(1)

# -----------------------------------------------------------------------------
# CONFIGURATION
# -----------------------------------------------------------------------------
CONNECTION_STRING = 'udp:127.0.0.1:14550'
BAUD_RATE = 115200
LLM_MODEL = "llama3.2" 

# -----------------------------------------------------------------------------
# USER PROVIDED TOOLS & PROMPT
# -----------------------------------------------------------------------------
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
            "1) If the user mentions TAKEOFF, ALTITUDE, HEIGHT, or 'meters' (and no direction), ALWAYS include a takeoff tool_call FIRST.\n"
            "2) If the user gives relative horizontal moves (e.g. 'fly east 20m') and the drone is NOT airborne, include a takeoff step first.\n"
            "3) 'fly 5 meters' means TAKEOFF to 5m.\n"
            "4) Preserve ALL actions in sequence.\n"
            "5) Output only VALID JSON.\n"
            "6) If ambiguous, ask a clarifying question.\n"
            "7) Commands meaning RETURN HOME / RTH / GO BACK / COME BACK / RETURN TO BASE / GO TO LAUNCH must ALWAYS map to the rtl() tool.\n"
            "8) Never use goto_latlon for these. ONLY rtl().\n"
        )
    }


# -----------------------------------------------------------------------------
# UTILITIES
# -----------------------------------------------------------------------------
def latlon_to_ned(lat, lon, home_lat, home_lon):
    """
    Convert Lat/Lon to NED (North, East) in meters relative to Home.
    Approximation for small distances.
    """
    R = 6378137.0  # Radius of Earth
    dLat = math.radians(lat - home_lat)
    dLon = math.radians(lon - home_lon)
    lat1 = math.radians(home_lat)
    lat2 = math.radians(lat)

    n = dLat * R
    e = dLon * R * math.cos(lat1)
    d = 0 # Altitude handled separately usually
    return n, e, d

# -----------------------------------------------------------------------------
# DRONE CONTROLLER CLASS
# -----------------------------------------------------------------------------
class DroneController:
    def __init__(self, connection_string):
        print(f"Connecting to vehicle on: {connection_string}")
        self.vehicle = connect(connection_string, wait_ready=True)
        print("Connected!")
        
        self.home_location = None
        self._setup_home()

    def _setup_home(self):
        """
        Get home location. Wait for it if necessary.
        """
        print("Waiting for Home location...")
        while not self.vehicle.home_location:
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
            if not self.vehicle.home_location:
                print(" Waiting for home...")
                time.sleep(1)
        
        self.home_location = self.vehicle.home_location
        print(f"Home Location: {self.home_location.lat}, {self.home_location.lon}, {self.home_location.alt}")

    def get_current_ned(self):
        """
        Get current NED position relative to Home.
        Prefers vehicle.location.local_frame if available, else calculates from Global.
        Returns: (north, east, down) in meters (down positive)
        """
        try:
            lf = self.vehicle.location.local_frame
            # local_frame may exist but have None values; check explicitly
            if hasattr(lf, 'north') and lf.north is not None:
                ln = float(lf.north)
                le = float(lf.east)
                ld = float(lf.down)
                # print(f"DEBUG: Using local_frame NED: {ln}, {le}, {ld}")
                return ln, le, ld
        except Exception:
            # If any issue with local_frame, fallback to geo conversion
            pass

        # Fallback to geographic conversion (approximation)
        curr = self.vehicle.location.global_frame
        if curr is None or self.home_location is None:
            # Shouldn't happen after startup, but guard anyway
            return 0.0, 0.0, 0.0

        n, e, _ = latlon_to_ned(curr.lat, curr.lon, self.home_location.lat, self.home_location.lon)
        # local_frame.down is usually positive downward; global_relative_frame.alt is altitude above ground
        alt_rel = self.vehicle.location.global_relative_frame.alt or 0.0
        # down should be positive if below origin, so set down = -alt_rel if origin altitude=0
        return float(n), float(e), -float(alt_rel)


    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """
        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print(f"Taking off to {aTargetAltitude}m")
        self.vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing other commands
        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            print(f" Altitude: {current_alt:.1f}m")
            if current_alt >= aTargetAltitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def send_ned_position(self, north, east, down):
        """
        Move vehicle to a position based on NED coordinates.
        Uses MAV_FRAME_LOCAL_NED.
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111111000, # type_mask (only positions enabled)
            north, east, down,
            0, 0, 0, # x, y, z velocity in m/s  (not used)
            0, 0, 0, # x, y, z acceleration (not used)
            0, 0)    # yaw, yaw_rate (not used)
        self.vehicle.send_mavlink(msg)

    def move_ned(self, north, east, down, absolute=False):
        """
        Move to NED coordinates.
        If absolute=True, moves to that exact NED position relative to Home.
        If absolute=False (default), moves relative to CURRENT position.

        Safety: 
        - If down==0 (relative) or down==current_down (absolute), preserve current altitude.
        - If alt < 0.5m and horizontal move requested, auto-takeoff to 2m.
        """
        # Get current NED (down is positive)
        current_n, current_e, current_d = self.get_current_ned()
        current_alt = -current_d  # altitude = -down if down is positive downward

        if not absolute:
            target_n = current_n + north
            target_e = current_e + east
            target_d = current_d + down
            print(f"Relative Move requested: dN={north}, dE={east}, dD={down}")
        else:
            target_n = north
            target_e = east
            target_d = down
            print(f"Absolute Move requested: N={north}, E={east}, D={down}")

        # Safety Check 0: Check if armed
        if not self.vehicle.armed:
            print("Error: Vehicle is disarmed. Cannot execute move command.")
            print("Tip: Use 'takeoff' first to arm and launch.")
            return

        # Safety Check 2: Auto-takeoff if too low and moving horizontally
        horizontal_move = math.hypot(target_n - current_n, target_e - current_e)
        if current_alt < 0.5 and horizontal_move > 0.5:
            print("Safety: Altitude < 0.5m and horizontal move requested. Auto-taking off to 2m...")
            self.arm_and_takeoff(2)
            # *** REFRESH ALL CURRENT POSITIONS after takeoff! (this was the bug)
            current_n, current_e, current_d = self.get_current_ned()
            current_alt = -current_d

            # Recompute targets if relative (we must use the fresh current_n/current_e/current_d)
            if not absolute:
                target_n = current_n + north
                target_e = current_e + east
                target_d = current_d + down
            else:
                # absolute targets remain as requested
                pass

        # Safety Check 3: Prevent moving down into ground (simple guard)
        if target_d > 0 and current_alt < 0.5:
            print(f"Safety: Target Down={target_d} (below origin) and current alt < 0.5m. Aborting move.")
            return

        print(f"Executing move_ned: Target N={target_n:.1f}, E={target_e:.1f}, D={target_d:.1f}")

        self.send_ned_position(target_n, target_e, target_d)

        # Monitor progress
        while True:
            curr_n, curr_e, curr_d = self.get_current_ned()
            dist = math.sqrt((target_n - curr_n)**2 + (target_e - curr_e)**2 + (target_d - curr_d)**2)
            # Clean line update: ensure we always overwrite previous line
            print(f" Distance to target: {dist:.1f}m (Curr: {curr_n:.1f}, {curr_e:.1f}, {curr_d:.1f})", end='\r')
            if dist < 1.0: # 1m tolerance
                print("\nTarget reached.")
                break
            time.sleep(0.5)


    def goto_latlon(self, lat, lon, alt):
        """
        Convert Lat/Lon/Alt to NED and execute move.
        """
        if lat == 0 and lon == 0:
            print("Error: Lat/Lon 0,0 detected. Ignoring invalid coordinate.")
            return

        print(f"Converting Lat/Lon ({lat}, {lon}) to NED...")
        n, e, _ = latlon_to_ned(lat, lon, self.home_location.lat, self.home_location.lon)
        # Alt is usually relative altitude for commands, so down = -alt
        d = -alt
        print(f"Calculated NED: N={n:.1f}, E={e:.1f}, D={d:.1f}")
        # goto_latlon is always absolute
        self.move_ned(n, e, d, absolute=True)

    def hold(self, seconds):
        print(f"Holding position for {seconds} seconds...")
        n, e, d = self.get_current_ned()
        self.send_ned_position(n, e, d)
        for i in range(int(seconds)):
            print(f"Holding... {int(seconds)-i}s", end='\r')
            time.sleep(1)
        print("\nHold complete.")

    def rtl(self):
        print("Returning to Launch (RTL)...")
        self.vehicle.mode = VehicleMode("RTL")
        while self.vehicle.mode.name != "RTL":
            time.sleep(0.1)
        
        while self.vehicle.armed:
            print(f" Alt: {self.vehicle.location.global_relative_frame.alt:.1f}m", end='\r')
            time.sleep(1)
        print("\nRTL Complete. Disarmed.")

    def land(self):
        print("Landing...")
        self.vehicle.mode = VehicleMode("LAND")
        while self.vehicle.armed:
            print(f" Alt: {self.vehicle.location.global_relative_frame.alt:.1f}m", end='\r')
            time.sleep(1)
        print("\nLanded.")

# -----------------------------------------------------------------------------
# LLM INTEGRATION
# -----------------------------------------------------------------------------
def ask_llm_for_toolcalls(user_input):
    print(f"Thinking (LLM: {LLM_MODEL})...")
    try:
        response = ollama.chat(
            model=LLM_MODEL, 
            messages=[
                build_system_prompt(),
                {'role': 'user', 'content': user_input},
            ],
            tools=TOOLS
        )
        
        # Check if the model decided to call tools
        if 'tool_calls' in response['message']:
            return response['message']['tool_calls']
        else:
            print("LLM did not return any tool calls. Checking content for fallback...")
            content = response['message']['content']
            print(f"Content: {content}")
            
            # Fallback: Try to find JSON in the content
            # Look for something that looks like {"name": "...", "parameters": ...} or just the JSON object
            try:
                data = None
                # Priority 1: Try to find a JSON list
                match_list = re.search(r'\[.*\]', content, re.DOTALL)
                if match_list:
                    try:
                        json_str = match_list.group(0)
                        data = json.loads(json_str)
                    except:
                        pass # Failed to parse list, maybe try object
                
                # Priority 2: If no list or failed, try to find a JSON object
                if data is None:
                    match_obj = re.search(r'\{.*\}', content, re.DOTALL)
                    if match_obj:
                        try:
                            json_str = match_obj.group(0)
                            # Attempt to repair common JSON errors (like missing colon after "parameters")
                            # Regex to find "parameters" followed by { but missing :
                            # Look for "parameters" \s* \{
                            if '"parameters"' in json_str and '"parameters":' not in json_str:
                                print("Attempting to repair JSON: missing colon after parameters")
                                json_str = re.sub(r'("parameters")\s*(\{)', r'\1: \2', json_str)
                            
                            data = json.loads(json_str)
                        except:
                            pass

                if data is not None:
                    # Check if it looks like a tool call structure directly or a list of them
                    # The standard tool call format from Ollama/OpenAI is usually a list of objects with 'function' key
                    # But sometimes LLM just outputs the function call itself: {"name": "land", "parameters": {}}
                    
                    calls = []
                    
                    # If it's a list, iterate
                    if isinstance(data, list):
                        items = data
                    else:
                        items = [data]
                        
                    for item in items:
                        # Case 1: Standard tool call format: {'type': 'function', 'function': {'name': '...', 'arguments': ...}}
                        if 'function' in item:
                             # Create a simple object-like wrapper or just return the dict if the main loop handles it
                             # The main loop expects an object with .function.name or a dict
                             # Let's stick to dicts for simplicity and ensure main loop handles it
                             calls.append(item)
                        
                        # Case 2: Direct function call: {'name': 'land', 'parameters': {}} or {'name': 'land', 'arguments': {}}
                        elif 'name' in item:
                            # Construct a fake tool call structure
                            args = item.get('parameters') or item.get('arguments') or {}
                            calls.append({
                                'function': {
                                    'name': item['name'],
                                    'arguments': args
                                }
                            })
                            
                    if calls:
                        print(f"Fallback successful! Found {len(calls)} tool calls.")
                        return calls
            except Exception as parse_err:
                print(f"Fallback parsing failed: {parse_err}")

            return []

    except Exception as e:
        print(f"LLM Error: {e}")
        # Fallback for older ollama versions or errors
        return []

# -----------------------------------------------------------------------------
# MAIN LOOP
# -----------------------------------------------------------------------------
def main():
    # Connect
    drone = DroneController(CONNECTION_STRING)
    
    print("\n---------------------------------------------------")
    print(" Drone Control System Ready")
    print(" Type natural language commands (e.g., 'takeoff to 5m')")
    print("---------------------------------------------------\n")

    while True:
        try:
            user_text = input("CMD> ")
            if not user_text:
                continue
            if user_text.lower() in ['exit', 'quit']:
                break

            tool_calls = ask_llm_for_toolcalls(user_text)
            
            if not tool_calls:
                continue

            # print(f"Plan: {json.dumps(tool_calls, indent=2)}") 
            # tool_calls from ollama python lib are objects, not just dicts usually, 
            # but let's print them carefully
            print(f"Plan: {tool_calls}")
            
            for call in tool_calls:
                # Handle both object-like and dict-like access
                if isinstance(call, dict):
                    func = call.get('function', {})
                    name = func.get('name')
                    args = func.get('arguments', {})
                else:
                    # Assume it's the object returned by ollama lib
                    name = call.function.name
                    args = call.function.arguments

                print(f"Executing: {name} with {args}")
                
                if name == 'takeoff':
                    drone.arm_and_takeoff(float(args.get('altitude', 10)))
                elif name == 'move_ned':
                    drone.move_ned(
                        float(args.get('north', 0)), 
                        float(args.get('east', 0)), 
                        float(args.get('down', 0)),
                        bool(args.get('absolute', False))
                    )
                elif name == 'goto_latlon':
                    drone.goto_latlon(
                        float(args.get('lat')), 
                        float(args.get('lon')), 
                        float(args.get('altitude', 10))
                    )
                elif name == 'hold':
                    drone.hold(float(args.get('seconds', 5)))
                elif name == 'rtl':
                    drone.rtl()
                elif name == 'land':
                    drone.land()
                else:
                    print(f"Unknown tool: {name}")

        except KeyboardInterrupt:
            print("\nAborting...")
            break
        except Exception as e:
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()

    print("Exiting.")

if __name__ == "__main__":
    main()
