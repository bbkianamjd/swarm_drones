# swarm_drones
One day, I decided to create some swarm drone software.

# 🚁 Drone Swarm System — ROS 2 + FastAPI

This project implements a minimal swarm coordination system using:

- ✅ **ROS 2 (Humble)** on the client side for inter-drone messaging
- ✅ **FastAPI** on the server side to track swarm state and respond with updated positions

---

## 📐 Architecture Overview
[ROS2 Node: position_planner] ---> HTTP POST ---> [FastAPI Server] ^ | | v [ROS2 Node: gateway_client] <--- HTTP Response --- DroneResponse | v Publishes: /swarm_position (drone_msgs::msg::DroneResponse)


---

## 🤖 Client-Side (ROS 2) Overview

### ✅ `position_planner` (dummy_publisher)

- Publishes `drone_msgs::msg::DroneRequest` to topic `/drone_position`
- Uses an env variable `DRONE_ID` to identify itself
- Subscribes to `/swarm_position` and caches all other drone positions

### ✅ `gateway_client`

- Subscribes to `/drone_position`
- Sends the message to the FastAPI server (`POST /drone_position`)
- Receives `DroneResponse` (swarm list), publishes it to `/swarm_position`

### ✅ Custom Messages

#### `DroneRequest.msg`

```msg
int32 sequence_number
string drone_id
float32 positionx
float32 positiony
float32 positionz
string timestamp
```

# Quickstart (Dev)
 
## Client Side
 cd client
 ./install.sh  # Installs the Docker
 
 ./run.sh # Runs the docker
 Note: server_ip is hard-coded
 Port is hard-coded
 
### Inside the docker
#### Run both nodes in parallel
./ros_ws/install/setup.bash # ros2 run drone_swarm gateway_client &  ros2 run drone_swarm
   
## Server Side
### Start the server (Outside the docker)
 cd server
 python3 server.py

## Testing
 ros2 topic echo /swarm_position
 Notes:
 - Port is hard-coded 2020. Ensure your network forwards this port
 - ip address is hard-coded in run.sh

