#!/bin/bash
# Test script to verify PX4 Typhoon H480 launches with default world

echo "=========================================="
echo "PX4 Default World Restoration - Test"
echo "=========================================="
echo ""

# Source the updated .bashrc
echo "1. Sourcing updated .bashrc..."
source ~/.bashrc
echo "   ✓ Done"
echo ""

# Check the GAZEBO_MODEL_PATH
echo "2. Checking GAZEBO_MODEL_PATH..."
echo "   Current GAZEBO_MODEL_PATH:"
echo "   $GAZEBO_MODEL_PATH"
echo ""

# Check if PX4 models are accessible
echo "3. Verifying PX4 models are accessible..."
if [ -d "/home/udbhaw/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models" ]; then
    echo "   ✓ PX4 models directory exists"
else
    echo "   ✗ PX4 models directory NOT found"
fi

if [ -d "/home/udbhaw/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/sun" ]; then
    echo "   ✓ sun model exists"
else
    echo "   ✗ sun model NOT found"
fi

if [ -d "/home/udbhaw/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/ground_plane" ]; then
    echo "   ✓ ground_plane model exists"
else
    echo "   ✗ ground_plane model NOT found"
fi

if [ -d "/home/udbhaw/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/asphalt_plane" ]; then
    echo "   ✓ asphalt_plane model exists"
else
    echo "   ✗ asphalt_plane model NOT found"
fi

echo ""
echo "=========================================="
echo "Ready to test PX4!"
echo "=========================================="
echo ""
echo "To launch PX4 Typhoon H480 with the default world, run:"
echo ""
echo "  cd /home/udbhaw/PX4-Autopilot"
echo "  make px4_sitl gazebo-classic_typhoon_h480"
echo ""
echo "You should now see:"
echo "  - Ground plane (asphalt)"
echo "  - Proper lighting (sun)"
echo "  - Typhoon H480 drone"
echo ""
echo "Instead of just the drone in a gray void."
echo "=========================================="
