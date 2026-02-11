#!/bin/bash
# Script to spawn a target box in Gazebo at specific coordinates
# Usage: ./spawn_target.sh x y z
# Defaults: x=0, y=60, z=45 (Matches the mission script default target)

X=${1:-0}   # East (NED Y)
Y=${2:-125}  # North (NED X)
Z=${3:-115}  # Up (NED -Z)

MODEL_NAME="target_box"
SDF_PATH="$(pwd)/models/target_box.sdf"

# Create a simple red box SDF if it doesn't exist
mkdir -p models
if [ ! -f "$SDF_PATH" ]; then
    echo "Creating simple box model at $SDF_PATH..."
    cat <<EOF > "$SDF_PATH"
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="$MODEL_NAME">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Red</name>
          </script>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
EOF
fi

echo "Spawning target box at X=$X, Y=$Y, Z=$Z (ENU Frame)"
gz model --spawn-file="$SDF_PATH" --model-name="$MODEL_NAME" -x "$X" -y "$Y" -z "$Z"

if [ $? -eq 0 ]; then
    echo "Target spawned successfully!"
else
    echo "Failed to spawn target. Make sure Gazebo is running."
fi
