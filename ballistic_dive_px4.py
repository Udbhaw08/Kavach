#!/usr/bin/env python3
"""
PURE BALLISTIC DIVE SCRIPT - PX4 VERSION
- Cruise to a hold point
- Yaw lock toward target
- Pure ballistic dive (straight line, no path correction)
- Auto LAND after dive
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, OffboardError
import math

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


class BallisticDiver:
    def __init__(self):
        self.drone = System()
        
    async def connect(self):
        print("Connecting to drone...")
        await self.drone.connect(system_address="udp://:14540")
        
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("✓ Drone connected!")
                break
                
    async def wait_ready(self):
        print("Waiting for drone ready...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("✓ Drone ready!")
                break
                
    async def arm_and_takeoff(self, alt):
        print(f"Arming and taking off to {alt}m...")
        await self.drone.action.arm()
        await self.drone.action.set_takeoff_altitude(alt)
        await self.drone.action.takeoff()
        
        # Wait for altitude
        async for position in self.drone.telemetry.position():
            if position.relative_altitude_m >= alt * 0.95:
                print(f"✓ Reached {position.relative_altitude_m:.1f}m")
                break
                
        await asyncio.sleep(2)
        
    async def start_offboard(self):
        print("Starting offboard mode...")
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
        )
        await self.drone.offboard.start()
        print("✓ Offboard active!")
        await asyncio.sleep(1)
        
    async def get_position_ned(self):
        """Get current NED position"""
        async for pos in self.drone.telemetry.position_velocity_ned():
            return pos.position.north_m, pos.position.east_m, pos.position.down_m
            
    async def cruise_to_point(self, targetN, targetE, targetD, speed=CRUISE_SPEED):
        """Simple proportional velocity approach"""
        print(f"[CRUISE] → target N:{targetN:.2f}  E:{targetE:.2f}  D:{targetD:.2f}")
        
        rate_dt = 1.0 / CMD_RATE_HZ
        start_time = asyncio.get_event_loop().time()
        
        while True:
            currN, currE, currD = await self.get_position_ned()
            
            dN = targetN - currN
            dE = targetE - currE
            dD = targetD - currD
            dist = math.sqrt(dN*dN + dE*dE + dD*dD)
            
            print(f"[CRUISE] dist = {dist:.2f}m")
            
            if dist <= ARRIVE_TOL:
                print("[CRUISE] ✓ Arrived!")
                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, 0.0)
                )
                return True
                
            if asyncio.get_event_loop().time() - start_time > NAV_TIMEOUT:
                print("[CRUISE] ✗ Timeout!")
                return False
                
            # Proportional speed
            scale = CRUISE_KP * dist
            spd = min(speed, max(1.0, scale))
            
            vx = (dN / dist) * spd
            vy = (dE / dist) * spd
            vz = (dD / dist) * spd
            
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(vx, vy, vz, 0.0)
            )
            await asyncio.sleep(rate_dt)
            
    async def yaw_to_face(self, targetN, targetE):
        """Compute and set yaw to face target"""
        currN, currE, _ = await self.get_position_ned()
        
        dN = targetN - currN
        dE = targetE - currE
        
        yaw_rad = math.atan2(dE, dN)
        yaw_deg = math.degrees(yaw_rad)
        
        print(f"[YAW] Aligning to {yaw_deg:.1f}°")
        
        # Set yaw by sending velocity with yaw
        for _ in range(20):  # Hold yaw for 2 seconds
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg)
            )
            await asyncio.sleep(0.1)
            
        print(f"✓ Yaw locked at {yaw_deg:.1f}°")
        return yaw_rad
        
    async def ballistic_dive_pure(self, yaw_rad, dive_speed=DIVE_SPEED, 
                                   dive_vz=DIVE_VZ, min_alt=DIVE_MIN_ALT, 
                                   max_time=DIVE_MAX_TIME):
        """Pure ballistic dive with locked heading"""
        print("\n[PURE DIVE] Ballistic dive — NO corrections")
        print(f" Locked heading = {math.degrees(yaw_rad):.1f}°")
        
        # Heading vector
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        
        vx = cos_y * dive_speed
        vy = sin_y * dive_speed
        vz = dive_vz
        
        yaw_deg = math.degrees(yaw_rad)
        
        print(f" Command: vx={vx:.2f}  vy={vy:.2f}  vz={vz:.2f}")
        
        dt = 1.0 / CMD_RATE_HZ
        start_time = asyncio.get_event_loop().time()
        
        while True:
            _, _, currD = await self.get_position_ned()
            alt = -currD
            
            print(f"[DIVE] alt={alt:.2f}m")
            
            if alt <= min_alt:
                print("[DIVE] ✓ Reached minimum altitude — stopping!")
                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg)
                )
                return True
                
            if asyncio.get_event_loop().time() - start_time > max_time:
                print("[DIVE] ✗ TIMEOUT — stopping dive!")
                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg)
                )
                return False
                
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(vx, vy, vz, yaw_deg)
            )
            await asyncio.sleep(dt)
            
    async def land_and_disarm(self):
        print("\n[POST] Stopping offboard and landing...")
        try:
            await self.drone.offboard.stop()
        except:
            pass
            
        await self.drone.action.land()
        
        print("Landing...")
        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                print("✓ Landed!")
                break
                
        try:
            await self.drone.action.disarm()
            print("✓ Disarmed!")
        except:
            print("✓ Already disarmed!")
            
    async def run_mission(self):
        await self.connect()
        await self.wait_ready()
        await self.arm_and_takeoff(TAKEOFF_ALT)
        
        # Get starting position
        await asyncio.sleep(1)
        startN, startE, startD = await self.get_position_ned()
        print(f"Start position: N={startN:.2f}, E={startE:.2f}, D={startD:.2f}")
        
        # DEFINE TARGET (adjust as needed)
        targetN = startN + 40
        targetE = startE + 80
        targetD = startD
        
        print(f"\n[TARGET] N:{targetN:.2f}  E:{targetE:.2f}")
        
        # Calculate hold point
        vecN = targetN - startN
        vecE = targetE - startE
        L = math.sqrt(vecN*vecN + vecE*vecE)
        
        holdN = targetN - (vecN / L) * HOLD_OFFSET
        holdE = targetE - (vecE / L) * HOLD_OFFSET
        holdD = startD
        
        print(f"[HOLD POINT] N:{holdN:.2f}  E:{holdE:.2f}")
        
        # Start offboard
        await self.start_offboard()
        
        # Cruise to hold point
        ok = await self.cruise_to_point(holdN, holdE, holdD)
        if not ok:
            print("✗ Failed to reach hold point!")
            await self.land_and_disarm()
            return
            
        # Yaw lock toward target
        yaw_rad = await self.yaw_to_face(targetN, targetE)
        
        # PURE BALLISTIC DIVE
        dive_ok = await self.ballistic_dive_pure(yaw_rad)
        
        if dive_ok:
            print("\n✓ DIVE COMPLETE!")
        else:
            print("\n✗ Dive ended with timeout")
            
        # Land
        await self.land_and_disarm()
        print("\n✓ MISSION COMPLETE!")


async def main():
    diver = BallisticDiver()
    await diver.run_mission()


if __name__ == "__main__":
    asyncio.run(main())
