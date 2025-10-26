#!/usr/bin/env bash
set -e

IMAGE_NAME=tailscale_ros
CONTAINER_NAME=tailscale_ros_dev

# if args have -b, build the image
if [ "$1" == "-b" ]; then
    echo "[*] Building Docker image..."
    docker build -t $IMAGE_NAME .
fi

# if the image does not exist, build it
if ! docker image inspect $IMAGE_NAME >/dev/null 2>&1; then
    echo "[*] Building Docker image..."
    docker build -t $IMAGE_NAME .
fi

# if args have -d, delete the container
if [ "$1" == "-d" ]; then
    echo "[*] Deleting Docker container..."
    docker rm -f $CONTAINER_NAME
fi

docker run -it --rm \
  --name $CONTAINER_NAME \
  --hostname docker-ub \
  --cap-add NET_ADMIN \
  --cap-add NET_RAW \
  -v ./lib:/var/lib \
  -v ./shared:/root/shared \
  -v /dev/net/tun:/dev/net/tun \
  -e TS_AUTHKEY="${DOCKERKEY_PERM}" \
  -e TS_ROUTES="10.0.0.0/8" \
  -e TS_USERSPACE=0 \
  -e TS_STATE_DIR=/var/lib/tailscale \
  -e TS_HOSTNAME=docker1 \
  $IMAGE_NAME \
  bash -c '
    echo "[*] Starting tailscaled..."
    nohup tailscaled >/var/log/tailscaled.log 2>&1 &
    sleep 2
    echo "[*] Bringing up Tailscale..."
    tailscale up --authkey "$TS_AUTHKEY" --hostname "$TS_HOSTNAME" --accept-routes
    echo "[*] Tailscale connected."
    exec bash
  '