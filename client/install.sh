#!/bin/bash

# Define variables
IMAGE_NAME="ros2_humble_custom"
CONTAINER_NAME="ros2_humble_dev"

# Step 2: Build Docker image
docker build -t "$IMAGE_NAME" .
if [ $? -ne 0 ]; then
  echo "[✗] Failed to build Docker image."
  exit 1
fi
echo "[✓] Docker image built: $IMAGE_NAME"

