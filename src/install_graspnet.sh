#!/usr/bin/env bash
# Install GraspNet following official instructions
# https://github.com/graspnet/graspnet-baseline

set -e

cd /root/src/graspnet-baseline

echo "[1/4] Installing requirements..."
pip3 install -r requirements.txt

echo "[2/4] Installing graspnetAPI..."
cd /root/src/graspnetAPI
pip3 install .

echo "[3/4] Compiling and installing pointnet2..."
cd /root/src/graspnet-baseline/pointnet2
python3 setup.py install

echo "[4/4] Compiling and installing knn..."
cd /root/src/graspnet-baseline/knn
python3 setup.py install

echo ""
echo "✓ Installation complete!"
echo "Test GPU: python3 -c 'import torch; print(torch.cuda.is_available())'"

