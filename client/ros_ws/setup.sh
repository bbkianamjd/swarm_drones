cd /ros_ws
# rm -rf build install log
colcon build --symlink-install

source install/setup.bash


# Run both nodes in background and capture their PIDs
ros2 run drone_swarm gateway_client &
PID1=$!
ros2 run drone_swarm position_planner &
PID2=$!

# Define a cleanup function
cleanup() {
  echo "Caught SIGINT, terminating nodes..."
  kill $PID1 $PID2
  wait
  exit 0
}

# Trap Ctrl+C (SIGINT)
trap cleanup SIGINT

# Wait for both to exit
wait
