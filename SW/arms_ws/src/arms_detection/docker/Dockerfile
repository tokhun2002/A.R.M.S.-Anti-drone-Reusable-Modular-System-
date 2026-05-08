# ============================================================
# A.R.M.S. Detection Container
# Base: Ultralytics Jetson Jetpack6 (Ubuntu 22.04 + CUDA + ultralytics)
# Adds: ROS2 Humble + arms_msgs + detection node
#
# Build context: arms_ws/src/
#   docker-compose build (from arms_detection/docker/)
# ============================================================
FROM ultralytics/ultralytics:latest-jetson-jetpack6

# ---- ROS2 Humble repository ----
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) \
        signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rclpy \
        ros-humble-sensor-msgs \
        ros-humble-rosidl-default-generators \
        ros-humble-rosidl-default-runtime \
        python3-colcon-common-extensions \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ---- Build arms_msgs inside the container ----
# Build context is arms_ws/src/, so arms_msgs/ is available at COPY time.
WORKDIR /ros_ws/src
COPY arms_msgs/ arms_msgs/

WORKDIR /ros_ws
RUN bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select arms_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release"

# ---- Copy detection node ----
COPY arms_detection/docker/arms_detection_node.py /ros_ws/arms_detection_node.py

# ---- Entry point ----
CMD ["bash", "-c", \
    "source /opt/ros/humble/setup.bash && \
     source /ros_ws/install/setup.bash && \
     python3 /ros_ws/arms_detection_node.py"]
