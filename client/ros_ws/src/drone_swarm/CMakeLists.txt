cmake_minimum_required(VERSION 3.8)
project(drone_swarm)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drone_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include headers
include_directories(
  include
)

# Executable: Dummy Publisher (aka position_planner)
add_executable(position_planner src/dummy_publisher.cpp)
ament_target_dependencies(position_planner rclcpp drone_msgs)

# Executable: Gateway Client
add_executable(gateway_client
  src/gateway/gateway_client.cpp
  src/gateway/swarm_client_http.cpp
)
ament_target_dependencies(gateway_client rclcpp drone_msgs)

target_link_libraries(gateway_client
  CURL::libcurl
  nlohmann_json::nlohmann_json
)

# Install all binaries
install(TARGETS
  position_planner
  gateway_client
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files (optional if you plan to use them elsewhere)
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()

