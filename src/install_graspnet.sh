#!/usr/bin/env bash
# Install GraspNet following official instructions
# https://github.com/graspnet/graspnet-baseline

set -e

# Activate conda environment
source /opt/miniconda3/bin/activate graspnet

cd /root/src/graspnet-baseline

echo "[*] Checking environment..."
echo "Python version: $(python --version)"

echo "[*] Installing Intel libraries (fixes iJIT_NotifyEvent error)..."
pip install intel-openmp

echo "[*] Installing PyTorch with CUDA 12.4 (matching system CUDA version)..."
# Uninstall existing PyTorch first
conda uninstall -y pytorch torchvision torchaudio --force-remove || true
# Install CUDA 12.4 PyTorch (compatible with system CUDA 12.4)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124

echo "[*] Verifying PyTorch installation..."
echo "PyTorch version: $(python -c 'import torch; print(torch.__version__)')"
echo "CUDA available: $(python -c 'import torch; print(torch.cuda.is_available())')"
echo "CUDA version: $(python -c 'import torch; print(torch.version.cuda)')"

echo "[*] Installing graspnet-baseline requirements..."
pip install -r requirements.txt

echo "[*] Installing missing dependencies..."
# Try to install autolab packages, but don't fail if they're not available
pip install autolab-core autolab-perception grasp_nms || echo "Warning: Some optional packages may not be available"

echo "[*] Installing graspnetAPI..."
cd /root/src/graspnetAPI
# Fix sklearn dependency in setup.py before installing
sed -i "s/'sklearn'/'scikit-learn'/g" setup.py || true
# Remove problematic dependencies from setup.py temporarily
sed -i "/'autolab_core',/d" setup.py || true
sed -i "/'autolab-perception',/d" setup.py || true
sed -i "/'grasp_nms'/d" setup.py || true
pip install .

echo "[*] Compiling and installing pointnet2 and knn..."
cd /root/src/graspnet-baseline/pointnet2
python setup.py install

cd /root/src/graspnet-baseline/knn
python setup.py install

echo "[*] Cleaning conda cache..."
conda clean -ya

echo ""
echo "✓ Installation complete!"
echo ""
echo "Testing installations..."
echo "Test GPU: python -c 'import torch; print(f\"CUDA available: {torch.cuda.is_available()}\")'"
echo "Test pointnet2: python -c 'import pointnet2' && echo 'pointnet2 OK'"
echo "Test knn: python -c 'import knn_pytorch' && echo 'knn OK'"

