#!/bin/bash

echo "🔄 Resetting ArduPilot SITL, MAVProxy and Gazebo..."

# Kill ArduPilot SITL
echo "❌ Killing ArduCopter / SITL..."
pkill -9 arducopter 2>/dev/null
pkill -9 ArduCopter 2>/dev/null
pkill -9 sim_vehicle.py 2>/dev/null

# Kill MAVProxy
echo "❌ Killing MAVProxy..."
pkill -9 mavproxy.py 2>/dev/null

# Kill Gazebo components
echo "❌ Killing Gazebo processes (gazebo, gzclient, gzserver)..."
pkill -9 gazebo 2>/dev/null
pkill -9 gzserver 2>/dev/null
pkill -9 gzclient 2>/dev/null

# Free locked ports by killing processes using them
PORTS=("5760" "5762" "11345" "14550" "14551" "9002" "9003")

echo "🔌 Freeing blocked ports..."
for PORT in "${PORTS[@]}"; do
    PID=$(sudo lsof -t -i :$PORT 2>/dev/null)
    if [ ! -z "$PID" ]; then
        echo "  → Killing PID $PID on port $PORT"
        sudo kill -9 $PID
    fi
done

# Remove Gazebo lock files
echo "🧹 Removing Gazebo lock files..."
rm -f ~/.gazebo/master.lock
rm -f ~/.gazebo/ogre.log
rm -rf /tmp/gazebo-*
rm -rf /tmp/gazebo/*
rm -rf /tmp/gazebo-11345/* 2>/dev/null

echo "✅ Done! All ports cleared and processes stopped."
echo "➡️ You can now start Gazebo and SITL cleanly."
