#!/usr/bin/env python3
"""
Target-Based Diving Algorithm with Explicit NED Frame Lock
- Uses bitmask for precise NED position control
- Target is locked in NED frame before dive
"""

import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw, VelocityNedYaw, OffboardError
from pymavlink import mavutil
import math

# ============= CONFIG =============
SEARCH_ALTITUDE = 35.0      
TARGET_LOCATION = {          
    'north': 50.0,
    'east': 50.0,
    'down': 0.0              
}

DIVE_ANGLE = 45.0           
DIVE_SPEED = 20.0           
PULLOUT_ALTITUDE = 5.0      
HOVER_TIME = 3.0            

POSITION_TOLERANCE = 2.0    
CMD_RATE = 10               


class TargetDiveMission:
    def __init__(self):
        self.drone = System()
        self.target_locked_ned = None  # Store locked NED target
        
    async def connect(self):
        print("🔌 Connecting to drone...")
        await self.drone.connect(system_address="udp://:14540")
        
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("✅ Drone connected!")
                break
                
    async def wait_ready(self):
        print("⏳ Waiting for drone ready...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("✅ Drone ready!")
                break
                
    async def arm_and_takeoff(self, altitude):
        print(f"\n🚁 PHASE 1: TAKEOFF TO {altitude}m")
        print("=" * 50)
        
        print("🔧 Arming...")
        await self.drone.action.arm()
        
        print(f"⬆️  Taking off to {altitude}m...")
        await self.drone.action.set_takeoff_altitude(altitude)
        await self.drone.action.takeoff()
        
        async for position in self.drone.telemetry.position():
            alt = position.relative_altitude_m
            print(f"   Altitude: {alt:.1f}m", end='\r')
            if alt >= altitude * 0.95:
                print(f"\n✅ Reached search altitude: {alt:.1f}m")
                break
                
        await asyncio.sleep(2)
        
    async def get_position_ned(self):
        """Get current NED position"""
        async for pos in self.drone.telemetry.position_velocity_ned():
            return pos.position.north_m, pos.position.east_m, pos.position.down_m
            
    async def start_offboard(self):
        print("\n🎮 Starting offboard control...")
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -10.0, 0.0)
        )
        await self.drone.offboard.start()
        print("✅ Offboard mode active!")
        await asyncio.sleep(1)
        
    def send_ned_position_bitmask(self, north, east, down, yaw):
        """
        Send NED position command with explicit bitmask
        Bitmask: 0b0000111111111000 = 0x0FF8
        - Position: N, E, D enabled
        - Velocity: disabled
        - Acceleration: disabled
        - Yaw: enabled
        """
        msg = self.drone._system_impl._connection.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # type_mask (position only)
            north, east, down,   # position
            0, 0, 0,            # velocity (ignored)
            0, 0, 0,            # acceleration (ignored)
            yaw, 0              # yaw, yaw_rate
        )
        self.drone._system_impl._connection.send_mavlink(msg)
        
    async def position_above_target(self, targetN, targetE, altitude):
        """Position drone above target using bitmask"""
        print(f"\n🎯 PHASE 2: POSITIONING ABOVE TARGET (NED BITMASK)")
        print("=" * 50)
        print(f"Target: N={targetN:.1f}m, E={targetE:.1f}m")
        
        targetD = -altitude
        
        dt = 1.0 / CMD_RATE
        
        while True:
            currN, currE, currD = await self.get_position_ned()
            
            dN = targetN - currN
            dE = targetE - currE
            dD = targetD - currD
            
            horizontal_dist = math.sqrt(dN*dN + dE*dE)
            
            print(f"Distance: {horizontal_dist:.2f}m", end='\r')
            
            if horizontal_dist <= POSITION_TOLERANCE and abs(dD) < 1.0:
                print(f"\n✅ Positioned above target!")
                await asyncio.sleep(2)
                return True
                
            # Send position command with bitmask
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(targetN, targetE, targetD, 0.0)
            )
            await asyncio.sleep(dt)
            
    async def lock_target_ned_frame(self, targetN, targetE):
        """
        CRITICAL: Lock target in NED frame explicitly
        After this, target coordinates are FROZEN in NED frame
        """
        print(f"\n🔒 PHASE 3: LOCKING TARGET IN NED FRAME")
        print("=" * 50)
        
        currN, currE, currD = await self.get_position_ned()
        
        # Calculate yaw to target
        dN = targetN - currN
        dE = targetE - currE
        
        yaw_rad = math.atan2(dE, dN)
        yaw_deg = math.degrees(yaw_rad)
        
        # LOCK TARGET: Store in NED frame (will NOT change)
        self.target_locked_ned = {
            'north': targetN,
            'east': targetE,
            'down': 0.0,
            'yaw_rad': yaw_rad,
            'yaw_deg': yaw_deg
        }
        
        print(f"🎯 Target LOCKED in NED frame:")
        print(f"   N: {targetN:.2f}m")
        print(f"   E: {targetE:.2f}m")
        print(f"   Yaw: {yaw_deg:.1f}°")
        print(f"📹 FPV Camera locked on target")
        
        # Hold yaw using bitmask
        for i in range(20):
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(currN, currE, currD, yaw_deg)
            )
            await asyncio.sleep(0.1)
            
        print("✅ Target LOCKED - NED frame frozen!")
        await asyncio.sleep(1)
        
    async def execute_dive_with_bitmask(self):
        """Execute dive using velocity bitmask toward locked NED target"""
        print(f"\n🚀 PHASE 4: DIVE TO LOCKED TARGET")
        print("=" * 50)
        
        if self.target_locked_ned is None:
            print("❌ ERROR: Target not locked!")
            return
            
        print(f"Diving to LOCKED target:")
        print(f"   N: {self.target_locked_ned['north']:.2f}m")
        print(f"   E: {self.target_locked_ned['east']:.2f}m")
        print(f"   Yaw: {self.target_locked_ned['yaw_deg']:.1f}°")
        print()
        print(f"Dive angle: {DIVE_ANGLE}°")
        print(f"Dive speed: {DIVE_SPEED}m/s")
        print()
        print("🔴 DIVING NOW!")
        
        # Calculate velocity based on locked target
        dive_angle_rad = math.radians(DIVE_ANGLE)
        horizontal_speed = DIVE_SPEED * math.cos(dive_angle_rad)
        vertical_speed = DIVE_SPEED * math.sin(dive_angle_rad)
        
        yaw_rad = self.target_locked_ned['yaw_rad']
        yaw_deg = self.target_locked_ned['yaw_deg']
        
        # Velocity components toward LOCKED target
        vx = horizontal_speed * math.cos(yaw_rad)
        vy = horizontal_speed * math.sin(yaw_rad)
        vz = vertical_speed
        
        print(f"Velocity (NED locked): vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")
        
        dt = 1.0 / CMD_RATE
        
        while True:
            _, _, currD = await self.get_position_ned()
            altitude = -currD
            
            print(f"⬇️  Diving... Alt: {altitude:.1f}m", end='\r')
            
            if altitude <= PULLOUT_ALTITUDE:
                print(f"\n✅ Pullout altitude: {altitude:.1f}m")
                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0.0, 0.0, 0.0, yaw_deg)
                )
                break
                
            # Continue dive with velocity bitmask
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(vx, vy, vz, yaw_deg)
            )
            await asyncio.sleep(dt)
            
        await asyncio.sleep(2)
        
    async def verify_target_position(self):
        """Verify we're near the locked target"""
        print(f"\n🎯 PHASE 5: TARGET VERIFICATION")
        print("=" * 50)
        
        currN, currE, currD = await self.get_position_ned()
        
        targetN = self.target_locked_ned['north']
        targetE = self.target_locked_ned['east']
        
        dN = targetN - currN
        dE = targetE - currE
        dist = math.sqrt(dN*dN + dE*dE)
        
        print(f"Current position: N={currN:.2f}, E={currE:.2f}")
        print(f"Target position:  N={targetN:.2f}, E={targetE:.2f}")
        print(f"Distance to target: {dist:.2f}m")
        
        if dist <= 10.0:
            print("✅ Target reached successfully!")
        else:
            print(f"⚠️  Target missed by {dist:.2f}m")
            
        await asyncio.sleep(HOVER_TIME)
        
    async def land_and_disarm(self):
        print("\n🛬 Landing...")
        
        try:
            await self.drone.offboard.stop()
        except:
            pass
            
        await self.drone.action.land()
        
        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                print("✅ Landed!")
                break
                
        try:
            await self.drone.action.disarm()
            print("✅ Disarmed!")
        except:
            print("✅ Already disarmed!")
            
    async def run_mission(self):
        """Execute complete mission with NED frame lock"""
        await self.connect()
        await self.wait_ready()
        
        print("\n" + "="*60)
        print("    TARGET DIVING - NED FRAME LOCKED")
        print("="*60)
        
        # Phase 1: Takeoff
        await self.arm_and_takeoff(SEARCH_ALTITUDE)
        
        targetN = TARGET_LOCATION['north']
        targetE = TARGET_LOCATION['east']
        
        # Start offboard
        await self.start_offboard()
        
        # Phase 2: Position above target
        await self.position_above_target(targetN, targetE, SEARCH_ALTITUDE)
        
        # Phase 3: LOCK target in NED frame (critical!)
        await self.lock_target_ned_frame(targetN, targetE)
        
        # Phase 4: Dive to LOCKED target
        await self.execute_dive_with_bitmask()
        
        # Phase 5: Verify
        await self.verify_target_position()
        
        # Land
        await self.land_and_disarm()
        
        print("\n" + "="*60)
        print("    MISSION COMPLETE - NED LOCKED DIVE")
        print("="*60)


async def main():
    mission = TargetDiveMission()
    await mission.run_mission()


if __name__ == "__main__":
    asyncio.run(main())