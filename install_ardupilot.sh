#!/bin/bash

# Configuration
INSTALL_DIR="$HOME/ArduPilot_Project"
REPO_URL="https://github.com/ArduPilot/ardupilot.git"
BRANCH="Copter-4.5" # Stable branch, or master for latest features

echo "--------------------------------------------------------"
echo "Starting ArduPilot Installation Script"
echo "Make sure you have sudo privileges."
echo "--------------------------------------------------------"

# 1. Create Directory
echo "[+] Creating workspace at $INSTALL_DIR"
mkdir -p "$INSTALL_DIR"
cd "$INSTALL_DIR"

# 2. Clone Repository
if [ -d "ardupilot" ]; then
    echo "[!] Directory 'ardupilot' already exists. Pulling latest..."
    cd ardupilot
    git pull
else
    echo "[+] Cloning ArduPilot repository..."
    git clone --recursive "$REPO_URL" ardupilot
    cd ardupilot
fi

# 3. Checkout Branch (Optional)
# echo "[+] Checking out branch $BRANCH..."
# git checkout "$BRANCH"
# git submodule update --init --recursive

# 4. Install Dependencies
echo "[+] Installing prerequisites (Requires SUDO)..."
echo "    Running: Tools/environment_install/install-prereqs-ubuntu.sh -y"
Tools/environment_install/install-prereqs-ubuntu.sh -y

# 5. Reload Profile
echo "[+] Reloading .profile to update PATH..."
. ~/.profile

echo "--------------------------------------------------------"
echo "Installation Complete!"
echo "To test installation, open a NEW terminal and run:"
echo "  cd $INSTALL_DIR/ardupilot/ArduCopter"
echo "  sim_vehicle.py -v ArduCopter --console --map"
echo "--------------------------------------------------------"
