#!/bin/bash
cd graspnet-baseline

CUDA_VISIBLE_DEVICES=0 \
    python demo.py --checkpoint_path ../weights/checkpoint-kn.tar