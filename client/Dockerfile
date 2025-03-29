FROM ros:humble-ros-base

# Set environment variables
ENV ROS_DOMAIN_ID=0
ENV ROS_NAMESPACE=my_bot

# Create the ROS workspace structure
WORKDIR /ros_ws
RUN mkdir -p src

# Install system dependencies
RUN apt update && apt install -y \
    libcurl4-openssl-dev \
    nlohmann-json3-dev

# Copy your source code into the container
COPY ros_ws/src ./src

# Source base ROS 2 env and build workspace
RUN . /opt/ros/humble/setup.sh 
#&& colcon build --symlink-install

# Append sourcing to .bashrc (for interactive use)
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "[ -f /ros_ws/install/setup.bash ] && source /ros_ws/install/setup.bash" >> ~/.bashrc

# Default startup command (optional — replace with entrypoint or custom script if needed)
CMD ["bash"]
#CMD ["/ros_ws/setup.sh"]

