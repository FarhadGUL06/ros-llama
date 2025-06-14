FROM ros:noetic

# Install system dependencies, including cmake, blas, and others
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    libblas-dev \
    python3-pip \
    python3-catkin-tools \
    git \
    ros-noetic-rospy \
    ros-noetic-std-msgs \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install llama-cpp-python with your environment variables
ENV CMAKE_ARGS="-DLLAMA_CLBLAST=on"
ENV FORCE_CMAKE=1

RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install --no-cache-dir llama-cpp-python

# Create ROS workspace folder structure
RUN mkdir -p /root/catkin_ws/src

WORKDIR /root/catkin_ws

# Build workspace (initially empty)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

CMD ["/bin/bash"]
