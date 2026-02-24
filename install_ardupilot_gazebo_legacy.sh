#!/bin/bash
# Install ArduPilot Gazebo Plugin (Legacy / Gazebo Classic)
# Based on khancyr/ardupilot_gazebo

echo "--------------------------------------------------------"
echo "Installing ArduPilot Gazebo Plugin (Classic)"
echo "--------------------------------------------------------"

# 1. Install Dependencies
# Check if Gazebo is already installed (likely yes from PX4)
if dpkg -l | grep -q gazebo11; then
    echo "[+] Gazebo 11 detected. Installing development headers and plugin dependencies..."
else
    echo "[+] Gazebo 11 not found. Installing..."
fi
sudo apt-get install -y libgazebo-dev gazebo11 libopencv-dev

# 2. Clone Plugin Repo
cd $HOME/ArduPilot_Project
if [ -d "ardupilot_gazebo" ]; then
    echo "[!] ardupilot_gazebo already exists. Pulling latest..."
    cd ardupilot_gazebo
    git pull
else
    git clone https://github.com/khancyr/ardupilot_gazebo.git
    cd ardupilot_gazebo
fi

# 3. Build Plugin
mkdir -p build
cd build
cmake ..
make -j4
sudo make install

echo "--------------------------------------------------------"
echo "Plugin Installation Complete!"
echo "Make sure to add the models to GAZEBO_MODEL_PATH:"
echo 'export GAZEBO_MODEL_PATH=$HOME/ArduPilot_Project/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH' >> ~/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=/usr/local/lib/x86_64-linux-gnu/gazebo-11/plugins:$GAZEBO_PLUGIN_PATH' >> ~/.bashrc
echo "--------------------------------------------------------"
