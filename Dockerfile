FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-c"]

# System packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Python dependencies used in this project
RUN pip3 install --no-cache-dir \
    flask \
    numpy \
    scipy

# Create workspace root
WORKDIR /root/ros2_study/workspace

# Copy source code into container
COPY ros2_ws/ /root/ros2_study/workspace/

# Initialize rosdep if possible
RUN rosdep init 2>/dev/null || true && rosdep update || true

# Install ROS package dependencies from source tree
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Build workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

# Convenience: auto-source ROS and workspace in interactive shell
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_study/workspace/install/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_study/workspace

CMD ["/bin/bash"]
