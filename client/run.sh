#!/bin/bash

# Define variables
IMAGE_NAME="ros2_humble_custom"
SERVER_IP="73.15.180.243"

# Generate random DRONE_ID
DRONE_ID=$(tr -dc A-Za-z0-9 </dev/urandom | head -c 6)

# Container name tied to DRONE_ID
CONTAINER_NAME="ros2_humble_dev_${DRONE_ID}"

# Step 3: Run the container
docker run -it --rm \
  --name "$CONTAINER_NAME" \
  -e DRONE_ID="$DRONE_ID" \
  -e SERVER_IP="$SERVER_IP" \
  -v $(pwd)/ros_ws:/ros_ws \
  "$IMAGE_NAME"

