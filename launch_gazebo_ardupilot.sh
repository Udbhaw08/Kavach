#!/bin/bash
# Manually launch Gazebo (Classic) with ArduPilot plugin and Iris world

echo "--------------------------------------------------------"
echo "Launching Gazebo for ArduPilot..."
echo "--------------------------------------------------------"

# 1. Setup Paths (Same as .bashrc, just to be sure)
export GAZEBO_MODEL_PATH=$HOME/ArduPilot_Project/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=/usr/local/lib/x86_64-linux-gnu/gazebo-11/plugins:$HOME/ArduPilot_Project/ardupilot_gazebo/build:$GAZEBO_PLUGIN_PATH

# 2. Check if gazebo is installed
if ! command -v gazebo &> /dev/null; then
    echo "Error: gazebo command not found. Is Gazebo installed?"
    exit 1
fi

# 3. Launch World
WORLD_FILE="$HOME/ArduPilot_Project/ardupilot_gazebo/worlds/iris_arducopter_runway.world"

if [ ! -f "$WORLD_FILE" ]; then
    echo "Warning: World file not found at $WORLD_FILE"
    echo "Launching empty world..."
    gazebo --verbose
else
    echo "Launching world: $WORLD_FILE"
    gazebo --verbose "$WORLD_FILE"
fi
