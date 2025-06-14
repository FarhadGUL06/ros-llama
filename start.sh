#!/bin/bash
set -e

LOCAL_LLAMA_PYTHON="$(pwd)/llama-python"
PACKAGE_NAME="llama_ros"
NODE_SCRIPT="ros-llama.py"

echo "Building Docker image ros-llama..."
sudo docker build -t ros-llama .

echo "Starting Docker container, mounting llama-python as /root..."
sudo docker run -it --rm \
  -v "$LOCAL_LLAMA_PYTHON:/root" \
  --workdir /root/catkin_ws \
  ros-llama \
  /bin/bash -c "
    set -e

    source /opt/ros/noetic/setup.bash

    # Create catkin workspace and src if not exist
    mkdir -p src
    cd src

    # Create ROS package if not exists
    if [ ! -d $PACKAGE_NAME ]; then
      catkin_create_pkg $PACKAGE_NAME rospy std_msgs
    fi

    # Copy node script into package scripts folder
    mkdir -p $PACKAGE_NAME/scripts
    cp /root/$NODE_SCRIPT $PACKAGE_NAME/scripts/
    chmod +x $PACKAGE_NAME/scripts/$NODE_SCRIPT

    cd ..

    echo 'Building catkin workspace...'
    catkin_make

    # Install the node script into install space (optional but good)
    source devel/setup.bash
    catkin_make install

    echo 'Setup complete. Node script is ready in the workspace.'

    exec /bin/bash
  "
