#!/bin/bash
# Fix GAZEBO_MODEL_PATH in .bashrc to properly append paths instead of overriding

# Create backup
echo "Creating backup of .bashrc..."
cp ~/.bashrc ~/.bashrc.backup_px4_fix

# Fix the GAZEBO_MODEL_PATH line
echo "Fixing GAZEBO_MODEL_PATH..."
sed -i 's|^export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models$|export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ardupilot_gazebo/models:~/Kavach/models|g' ~/.bashrc

# Add comment explaining the fix
sed -i '/^export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:~\/ardupilot_gazebo\/models:~\/Kavach\/models$/i # Append to GAZEBO_MODEL_PATH instead of overriding (fixed for PX4 compatibility)' ~/.bashrc

echo "✓ Fixed! The line now appends to GAZEBO_MODEL_PATH instead of overriding it."
echo ""
echo "Original line:"
echo "  export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models"
echo ""
echo "New line:"
echo "  export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:~/ardupilot_gazebo/models:~/Kavach/models"
echo ""
echo "To apply changes, run: source ~/.bashrc"
