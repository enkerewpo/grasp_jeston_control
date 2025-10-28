#!/bin/bash
cd graspnet-baseline

# to fix the error, manually do this once for conda environment to use system libstdc++.so.6
# mv /opt/miniconda3/envs/graspnet/lib/libstdc++.so.6 /opt/miniconda3/envs/graspnet/lib/libstdc++.so.6.bak

# ln -sf /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /opt/miniconda3/envs/graspnet/lib/libstdc++.so.6 (optional)

# export OPEN3D_HEADLESS=1

export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

export PYTHONPATH=/opt/ros/humble/lib/python3.10/site-packages:$PYTHONPATH

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

# Check ROS2 topics before running
echo "[*] Checking ROS2 topics..."
python << 'EOF'
import rclpy
from rclpy.node import Node

# Initialize rclpy
rclpy.init()

# Create a node
node = Node('test_node')

try:
    # Get topics
    topics_and_types = node.get_topic_names_and_types()
    
    print(f'[*] Found {len(topics_and_types)} ROS2 topics')
    
    # Show relevant camera topics
    camera_topics = [t[0] for t in topics_and_types if '/camera' in t[0]]
    if camera_topics:
        print('[*] Camera topics found:')
        for topic in camera_topics:
            print(f'    {topic}')
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
except Exception as e:
    print(f'Error: {e}')
    node.destroy_node()
    rclpy.shutdown()
EOF

echo ""
echo "[*] Running GraspNet ROS2 demo..."
echo "[*] Make sure ROS2 topics are publishing: /camera/color/image_raw, /camera/depth/image_raw, /camera/color/camera_info"
echo ""
echo "[*] If stuck, press Ctrl+C to interrupt"
echo "[*] The program will timeout after 30 seconds if no data received"
echo ""


# export OPEN3D_HEADLESS=1

# Run with timeout
# Add --no_collision to disable collision detection for debugging
CUDA_VISIBLE_DEVICES=0 \
    timeout 60 python -W ignore demo_ros2.py \
    --checkpoint_path ../weights/checkpoint-kn.tar \
    --process_once \
    --no_collision

echo ""
echo "[*] Done"