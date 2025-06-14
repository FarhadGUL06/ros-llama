#!/bin/bash
set -e

LOCAL_LLAMA_PYTHON="$(pwd)/llama-python"
PACKAGE_NAME="llama_ros"
NODE_SCRIPTS=("ros-llama.py" "publisher.py" "subscriber.py")
CONTAINER_NAME="ros_llama_container"

# Make sure ./llama-python/catkin_ws/src exists
if [ ! -d "$LOCAL_LLAMA_PYTHON/catkin_ws/src" ]; then
  echo "Creating catkin workspace directory structure..."
  mkdir -p "$LOCAL_LLAMA_PYTHON/catkin_ws/src"
fi

echo "Building Docker image ros-llama..."
sudo docker build -t ros-llama .

echo "Starting Docker container, mounting llama-python as /root..."
sudo docker run -it --rm \
  --name "$CONTAINER_NAME" \
  -v "$LOCAL_LLAMA_PYTHON:/root" \
  --workdir /root/catkin_ws \
  ros-llama \
  /bin/bash -c "
    set -e
    source /opt/ros/noetic/setup.bash
    
    # Setup catkin workspace and package
    mkdir -p src
    cd src
    if [ ! -d $PACKAGE_NAME ]; then
      catkin_create_pkg $PACKAGE_NAME rospy std_msgs
    fi

    mkdir -p $PACKAGE_NAME/scripts
    cp /root/scripts/*.py $PACKAGE_NAME/scripts/
    chmod +x $PACKAGE_NAME/scripts/*.py

    cd ..
    catkin_make
    source devel/setup.bash

    # Launch nodes (just an example: run ros-llama.py here)
    roscore
  "
