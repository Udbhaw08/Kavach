#!/bin/bash
# Launch VTOL with large world directly
# Usage: ./launch_vtol_large_world.sh

cd /home/udbhaw/PX4-Autopilot

# Set the world file (just the name, no .world extension)
export PX4_SITL_WORLD=long_range_test

echo "Launching VTOL with long_range_test world..."
echo "World: Brown ground (5000m x 5000m) + Blue sky"
echo "Targets: Red box at 1500m, Yellow at 500m, Orange at 1000m"
echo ""

# Launch simulation with the world
make px4_sitl gazebo-classic_vtol_downward_depth_camera

