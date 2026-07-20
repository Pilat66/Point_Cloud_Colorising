#!/bin/bash
# Build and enter a development container for this package.
#
#   ./run_dev.sh [/path/to/bag/dir ...]
#
# The catkin workspace is derived from this script's location rather than
# hardcoded, so the container works wherever the repo is checked out. Any
# directories given as arguments are mounted read-only at /root/data0, data1...
# which is how you get bags into the container.
set -eu

# Docker/ -> package root -> src/ -> catkin workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$(cd "$PKG_DIR/../.." && pwd)"

PROJECT_NAME="point_cloud_projection"
IMAGE_NAME="${PROJECT_NAME}:noetic-ros"

if [ ! -d "$WS_DIR/src" ]; then
    echo "Could not find a catkin workspace above $PKG_DIR (looked at $WS_DIR)." >&2
    echo "Expected layout: <ws>/src/${PROJECT_NAME}/Docker/run_dev.sh" >&2
    exit 1
fi

echo "workspace: $WS_DIR"
docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile" "$SCRIPT_DIR"

# Mount any data directories passed on the command line.
DATA_MOUNTS=()
i=0
for d in "$@"; do
    if [ -d "$d" ]; then
        DATA_MOUNTS+=(-v "$(cd "$d" && pwd):/root/data$i:ro")
        echo "data:      $d -> /root/data$i"
        i=$((i + 1))
    else
        echo "skipping non-existent data dir: $d" >&2
    fi
done

# GPU flags only if the nvidia runtime is actually present, so this still runs
# on a machine without it.
GPU_FLAGS=()
if docker info 2>/dev/null | grep -qi nvidia; then
    GPU_FLAGS=(--runtime=nvidia --gpus all
               --env NVIDIA_VISIBLE_DEVICES=all
               --env NVIDIA_DRIVER_CAPABILITIES=all)
    echo "gpu:       nvidia runtime detected"
else
    echo "gpu:       no nvidia runtime, running without GPU"
fi

# RViz needs access to the host X server.
xhost +local:root >/dev/null 2>&1 || true

docker run --rm -it \
    --name "$PROJECT_NAME" \
    -e DISPLAY="$DISPLAY" \
    -v "$HOME/.Xauthority:/root/.Xauthority:rw" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --network host \
    -v "$WS_DIR:/root/catkin_ws" \
    "${DATA_MOUNTS[@]}" \
    "${GPU_FLAGS[@]}" \
    --privileged \
    --cap-add sys_ptrace \
    "$IMAGE_NAME" /bin/bash

xhost -local:root >/dev/null 2>&1 || true
