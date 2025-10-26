#!/bin/bash
cd graspnet-baseline

# Set headless mode for Docker container
export OPEN3D_HEADLESS=1

CUDA_VISIBLE_DEVICES=0 \
    python demo.py --checkpoint_path ../weights/checkpoint-kn.tar --headless