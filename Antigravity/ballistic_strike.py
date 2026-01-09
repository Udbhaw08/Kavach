#!/usr/bin/env python3
"""
Autonomous Ballistic Strike Drone
5-Phase Mission Profile with Attitude-Based Terminal Dive
"""

import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw, Attitude
from config import *


class BallisticStrikeDrone:
    def __init__(self):
        self.drone = System()
        self.param_backup = {}

    # ================= CONNECT =================
    async def connect(self):
        print(f"[CONNECT] Connecting to drone at {SYSTEM_ADDRESS}...")
        await self.drone.connect(system_address=SYSTEM_ADDRESS)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("[CONNECT] Drone connected!")
                break

    # ================= PARAM BACKUP =================
    async def backup_parameters(self):
        print("[BACKUP] Saving original parameters...")
        for p in PX4_PARAMS:
            try:
                val = await self.drone.param.get_param_float(p)
                self.param_backup[p] = ("float", val)
            except Exception:
                try:
                    val = await self.drone.param.get_param_int(p)
                    self.param_backup[p] = ("int", val)
                except Exception:
                    print(f"[BACKUP] Could not read {p}")

    async def configure_parameters(self):
        print("[CONFIG] Applying aggressive parameters...")
        for p, v in PX4_PARAMS.items():
            try:
                await self.drone.param.set_param_float(p, float(v))
                await asyncio.sleep(0.1)
            except Exception:
                print(f"[CONFIG] Failed to set {p}")

    async def restore_parameters(self):
        print("[RESTORE] Restoring parameters...")
        for p, (t, v) in self.param_backup.items():
            try:
                if t == "float":
                    await self.drone.param.set_param_float(p, v)
                else:
                    await self.drone.param.set_param_int(p, v)
            except Exception:
                print(f"[RESTORE] Failed {p}")

    # ================= ARM / OFFBOARD =================
    async def wait_for_armed(self):
        print("[ARM] Waiting for arm...")
        async for armed in self.drone.telemetry.armed():
            if armed:
                print("[ARM] Armed")
                return

    async def enable_offboard(self):
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0, 0, 0, 0)
        )
        await self.drone.offboard.start()
        print("[OFFBOARD] Enabled")

    # ================= PHASES =================
    async def phase1_ascent(self):
        async for pos in self.drone.telemetry.position():
            alt = abs(pos.relative_altitude_m)
            if alt < TAKEOFF_ALTITUDE - 1:
                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0, 0, -ASCENT_VELOCITY, 0)
                )
            else:
                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(0, 0, 0, 0)
                )
                await asyncio.sleep(2)
                break

    async def phase2_transit(self):
        start = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - start < CRUISE_DURATION:
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(CRUISE_VELOCITY, 0, 0, 0)
            )
            await asyncio.sleep(0.5)

    async def phase3_commit(self):
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0, 0, 0, 0)
        )
        await asyncio.sleep(1)
        print("[PHASE 3] COMMIT")

    # ================= BALLISTIC MATH =================
    def compute_ballistic_thrust(self, pitch_deg,
                                 hover_thrust=0.5,
                                 k=0.7,
                                 max_thrust=0.6):
        theta = abs(math.radians(pitch_deg))
        thrust = k * hover_thrust / math.cos(theta)
        return min(thrust, max_thrust)

    # ================= PHASE 4 =================
    async def phase4_ballistic_dive(self):
        pitch = -DIVE_PITCH_ANGLE

        await self.drone.offboard.set_attitude(
            Attitude(0, pitch, 0, 0.35)
        )
        await asyncio.sleep(0.8)

        async for pos in self.drone.telemetry.position():
            alt = abs(pos.relative_altitude_m)
            if alt <= 5:
                break

            thrust = self.compute_ballistic_thrust(pitch)

            await self.drone.offboard.set_attitude(
                Attitude(0, pitch, 0, thrust)
            )
            await asyncio.sleep(0.1)

    # ================= PHASE 5 =================
    async def phase5_terminal(self):
        pitch = -DIVE_PITCH_ANGLE
        thrust = self.compute_ballistic_thrust(pitch)

        async for pos in self.drone.telemetry.position():
            if abs(pos.relative_altitude_m) <= 0.5:
                print("[IMPACT]")
                break

            await self.drone.offboard.set_attitude(
                Attitude(0, pitch, 0, thrust)
            )
            await asyncio.sleep(0.1)

    # ================= EXECUTION =================
    async def execute_mission(self):
        try:
            await self.connect()
            await self.backup_parameters()
            await self.configure_parameters()
            await self.wait_for_armed()
            await self.enable_offboard()

            await self.phase1_ascent()
            await self.phase2_transit()
            await self.phase3_commit()
            await self.phase4_ballistic_dive()
            await self.phase5_terminal()
        finally:
            await self.restore_parameters()


async def main():
    drone = BallisticStrikeDrone()
    await drone.execute_mission()


if __name__ == "__main__":
    asyncio.run(main())
