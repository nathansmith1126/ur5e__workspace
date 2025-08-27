#!/bin/bash
# Usage:
#   source setup_project.sh            # without venv (default)
#   source setup_project.sh open_venv  # with venv

# Source ROS
source /opt/ros/noetic/setup.bash

# Check if first argument is "open_venv"
if [ "$1" = "open_venv" ]; then
    if [ -d ~/ur5e_ws/.venv ]; then
        echo "[setup_project] Activating virtual environment..."
        source ~/ur5e_ws/.venv/bin/activate
    else
        echo "[setup_project] Requested venv activation, but .venv not found!"
    fi
else
    echo "[setup_project] Skipping virtual environment activation."
fi

# Source workspace (catkin_make_isolated)
source ~/ur5e_ws/install_isolated/setup.bash

