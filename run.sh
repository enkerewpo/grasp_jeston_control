#!/usr/bin/env bash
set -e

if [ -f .env ]; then
    echo "[*] Loading environment variables from .env..."
    export $(cat .env | grep -v '^#' | xargs)
fi

IMAGE_NAME=tailscale_ros
CONTAINER_NAME=tailscale_ros_dev

if [ "$1" == "-b" ]; then
    echo "[*] Building Docker image..."
    docker build -t $IMAGE_NAME .
fi

if ! docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
    echo "[*] Building Docker image..."
    docker build -t $IMAGE_NAME .
fi

if [ "$1" == "-d" ]; then
    echo "[*] Deleting Docker container..."
    docker rm -f $CONTAINER_NAME
fi

if [ "$1" == "-c" ]; then
    echo "[*] To clear Tailscale state, run: sudo rm -rf ./lib/tailscale"
    echo "[*] Then run ./run.sh again"
    exit 0
fi

GPU_ARGS=""
if command -v nvidia-smi &> /dev/null; then
    echo "[*] GPU detected, enabling GPU support..."
    GPU_ARGS="--gpus all --runtime nvidia"
fi

# Check if user wants to use host network
# Use host network so container has full network access
NETWORK_ARGS="--net host"
# No SYSCTL_ARGS needed for host network (not allowed in host network namespace)

# Stop host tailscaled if running (to avoid TUN device conflicts)
if pgrep tailscaled > /dev/null; then
    echo "[*] Stopping host tailscaled to avoid TUN device conflicts..."
    sudo systemctl stop tailscaled 2>/dev/null || sudo pkill tailscaled || true
    sleep 2
fi

docker run -it --rm \
  --name $CONTAINER_NAME \
  --hostname docker-ub \
  --cap-add NET_ADMIN \
  --cap-add NET_RAW \
  --cap-add SYS_MODULE \
  $NETWORK_ARGS \
  $GPU_ARGS \
  -v ./shared:/root/shared \
  -v ./src:/root/src \
  -v ./tailscale:/var/lib/tailscale \
  -v /dev/net/tun:/dev/net/tun \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$XDG_RUNTIME_DIR:$XDG_RUNTIME_DIR" \
  -v "/dev/dri:/dev/dri" \
  -e TS_AUTHKEY="${DOCKERKEY_PERM}" \
  -e TS_ROUTES="10.0.0.0/8" \
  -e TS_USERSPACE=0 \
  -e TS_STATE_DIR=/var/lib/tailscale \
  -e TS_HOSTNAME=docker1 \
  -e DISPLAY=${DISPLAY:-:0} \
  -e QT_X11_NO_MITSHM=1 \
  -e QT_QPA_PLATFORM=xcb \
  -e NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all} \
  -e NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:-all} \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  $IMAGE_NAME \
  bash -c '
    mkdir -p /tmp/runtime-root
    chmod 700 /tmp/runtime-root
    export XDG_RUNTIME_DIR=/tmp/runtime-root
    
    # Set up OSMesa for headless OpenGL rendering
    export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/osmesa:${LD_LIBRARY_PATH}
    export OSMESA_LIB=/usr/lib/x86_64-linux-gnu/libOSMesa.so
    
    echo "[*] Starting tailscaled..."
    # Kill any existing tailscaled process in container
    killall tailscaled 2>/dev/null || true
    sleep 1
    
    # Check if tailscale0 exists and remove it
    if ip link show tailscale0 &>/dev/null; then
        echo "[*] Removing existing tailscale0 interface..."
        ip link set tailscale0 down 2>/dev/null || true
        ip link delete tailscale0 2>/dev/null || true
        sleep 1
    fi
    
    nohup tailscaled >/var/log/tailscaled.log 2>&1 &
    sleep 2
    
    echo "[*] Checking tailscaled status..."
    tailscale status || true
    
    echo "[*] Checking auth key..."
    if [ -z "$TS_AUTHKEY" ]; then
      echo "[!] Error: TS_AUTHKEY is empty"
      echo "[!] Please set DOCKERKEY_PERM environment variable"
      exit 1
    else
      echo "[*] Auth key is set (length: ${#TS_AUTHKEY})"
    fi
    
    echo "[*] Testing network connectivity..."
    echo "[*] Checking internet connectivity..."
    if ping -c 1 -W 2 8.8.8.8 >/dev/null 2>&1; then
      echo "[*] Network connectivity OK"
    else
      echo "[!] Warning: Network connectivity test failed"
      echo "[*] Showing network interfaces:"
      ip addr show || ifconfig
    fi
    
    echo "[*] Checking connectivity to Tailscale control plane..."
    if curl -s -m 5 https://login.tailscale.com >/dev/null 2>&1; then
      echo "[*] Can reach Tailscale login server"
    else
      echo "[!] Cannot reach Tailscale login server"
      echo "[*] Showing routing table:"
      ip route show || route -n
    fi
    
    echo "[*] Bringing up Tailscale..."
    echo "[*] This may take a moment (connecting to Tailscale servers)..."
    
    # Try with extended timeout and verbose output
    tailscale up --authkey "$TS_AUTHKEY" --hostname "$TS_HOSTNAME" --accept-routes 2>&1 || {
      echo "[!] Failed to bring up Tailscale"
      echo "[*] ========== Debugging Info =========="
      echo "[*] Recent Tailscale logs:"
      tail -50 /var/log/tailscaled.log
      echo ""
      echo "[*] Network status:"
      ip addr show
      echo ""
      echo "[*] Routing table:"
      ip route show
      echo "[*] ====================================="
      exit 1
    }
    
    echo "[*] Tailscale connected."
    echo "[*] Activating conda environment..."
    source /opt/miniconda3/bin/activate graspnet
    echo "[*] Conda environment activated: graspnet"
    exec bash
  '