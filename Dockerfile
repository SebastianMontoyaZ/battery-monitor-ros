# First stage: Build the ROS 2 workspace
FROM robotnik/ros:humble-builder-0.6.2 as builder

# Copy the workspace's source code into the Docker image
WORKDIR /ros2_ws
COPY src/ src/

# Install git
USER root
RUN apt-get update && apt-get install -y git && rm -rf /var/lib/apt/lists/*

# Download robotnik_msgs
WORKDIR /ros2_ws/src
RUN git clone https://github.com/RobotnikAutomation/robotnik_msgs.git -b ros2-devel

WORKDIR /ros2_ws

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build --install-base /ros2_ws/install

# Second stage: Copy the built artifacts into a smaller image
FROM robotnik/ros:humble-base-0.6.2

# Copy the install folder from the builder stage
WORKDIR /ros2_ws
COPY --from=builder /ros2_ws/install/ /ros2_ws/install/

# Install Kubernetes Python client
USER root
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip3 install kubernetes && \
    pip3 install prometheus_client && \
    rm -rf /var/lib/apt/lists/*

# Source the setup file and set the environment variables
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /ros2_ws/install/setup.bash && exec ros2 run battery_monitor battery_monitor"]