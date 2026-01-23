#!/bin/bash
# Simple and reliable VTOL launcher
# Just launches normally - world will show up as empty.world but that's fine for now

cd /home/udbhaw/PX4-Autopilot

# Kill existing instances
pkill -9 px4 2>/dev/null
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null
sleep 2

echo "🚀 Launching VTOL simulation..."
echo ""

# Launch with default configuration (this always works)
make px4_sitl gazebo-classic_vtol_downward_depth_camera

echo ""
echo "✅ Done"
