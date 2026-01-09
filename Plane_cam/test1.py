import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan


async def main():
    # -----------------------------
    # CONNECT TO PX4
    # -----------------------------
    drone = System()
    # match PX4 mavlink onboard port (from your logs)
    await drone.connect(system_address="udp://127.0.0.1:14580")

    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✔ Drone connected")
            break

    # -----------------------------
    # BUILD MISSION
    # -----------------------------
    mission_items = []

    # 1️⃣ Takeoff + climb to 50 m
    mission_items.append(
        MissionItem(
            latitude_deg=0.0,
            longitude_deg=0.0,
            relative_altitude_m=50.0,       # climb altitude
            speed_m_s=15.0,
            is_fly_through=True,
            gimbal_pitch_deg=0.0,
            gimbal_yaw_deg=0.0,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0,
            camera_photo_interval_s=0,
            acceptance_radius_m=10.0,
            yaw_deg=float("nan"),
            camera_photo_distance_m=0,
            vehicle_action=MissionItem.VehicleAction.NONE
        )
    )

    # 2️⃣ Controlled descent segment (SAFE descent)
    mission_items.append(
        MissionItem(
            latitude_deg=0.001,              # ~110 m north
            longitude_deg=0.0,
            relative_altitude_m=30.0,        # descend to 30m
            speed_m_s=18.0,
            is_fly_through=True,
            gimbal_pitch_deg=0.0,
            gimbal_yaw_deg=0.0,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0,
            camera_photo_interval_s=0,
            acceptance_radius_m=15.0,
            yaw_deg=float("nan"),
            camera_photo_distance_m=0,
            vehicle_action=MissionItem.VehicleAction.NONE
        )
    )

    # 3️⃣ Proper FIXED-WING landing waypoint
    mission_items.append(
        MissionItem(
            latitude_deg=0.003,              # farther ahead = safe approach
            longitude_deg=0.0,
            relative_altitude_m=0.0,         # landing alt MUST be 0
            speed_m_s=15.0,
            is_fly_through=False,
            gimbal_pitch_deg=0.0,
            gimbal_yaw_deg=0.0,
            camera_action=MissionItem.CameraAction.NONE,
            loiter_time_s=0,
            camera_photo_interval_s=0,
            acceptance_radius_m=20.0,
            yaw_deg=float("nan"),
            camera_photo_distance_m=0,
            vehicle_action=MissionItem.VehicleAction.LAND
        )
    )

    mission_plan = MissionPlan(mission_items)

    # -----------------------------
    # UPLOAD MISSION
    # -----------------------------
    await drone.mission.clear_mission()
    await drone.mission.set_return_to_launch_after_mission(False)

    print("⬆ Uploading mission…")
    await drone.mission.upload_mission(mission_plan)

    # -----------------------------
    # ARM + START
    # -----------------------------
    print("⚙ Arming…")
    await drone.action.arm()

    print("🚀 Starting mission…")
    await drone.mission.start_mission()

    # -----------------------------
    # MONITOR PROGRESS
    # -----------------------------
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: {mission_progress.current} / {mission_progress.total}")
        if mission_progress.current == mission_progress.total:
            print("✔ Mission finished")
            break


if __name__ == "__main__":
    asyncio.run(main())
