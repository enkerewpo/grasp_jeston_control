#!/bin/bash
cd graspnet-baseline

# export OPEN3D_HEADLESS=1

# Source ROS2 Humble setup
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "[*] Sourced ROS2 Humble setup"
fi

# Add ROS2 Humble Python packages to PYTHONPATH for conda environment
for PYTHON_SITE in /opt/ros/humble/lib/python*/site-packages; do
    if [ -d "$PYTHON_SITE" ]; then
        export PYTHONPATH="$PYTHON_SITE:$PYTHONPATH"
        echo "[*] Added ROS2 Python packages: $PYTHON_SITE"
    fi
done

# Also add workspace if it exists
if [ -d "$HOME/ros2_ws/install" ]; then
    for PYTHON_SITE in $HOME/ros2_ws/install/lib/python*/site-packages; do
        if [ -d "$PYTHON_SITE" ]; then
            export PYTHONPATH="$PYTHONPATH:$PYTHON_SITE"
            echo "[*] Added workspace Python packages: $PYTHON_SITE"
        fi
    done
fi

# Create output directory if it doesn't exist
OUTPUT_DIR="../output/visualization"
mkdir -p "$OUTPUT_DIR"

# Suppress ROS-related warnings
export PYTHONWARNINGS="ignore::UserWarning"

CUDA_VISIBLE_DEVICES=0 \
    python -W ignore demo.py --checkpoint_path ../weights/checkpoint-kn.tar