#!/bin/bash
set -e

CONTAINER_NAME="ros_llama_container"
PACKAGE_NAME="llama_ros"
BASE_CMD="source /opt/ros/noetic/setup.bash && source /root/catkin_ws/devel/setup.bash"

# Check if container is running
if ! sudo docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  echo "Error: Docker container '$CONTAINER_NAME' is not running."
  exit 1
fi

open_node_terminal() {
  local node_script="$1"
  local window_title="$2"
  
  gnome-terminal --title="$window_title" -- bash -c "
    sudo docker exec -it $CONTAINER_NAME bash -c '
      $BASE_CMD &&
      rosrun $PACKAGE_NAME $node_script
    ';
    echo \"\nProcess ended for $node_script. You can now interact with this terminal.\";
    exec bash
  "
}

echo "Opening terminals and running nodes..."

open_node_terminal "ros-llama.py" "ros-llama.py node"
open_node_terminal "publisher.py" "publisher.py node"
open_node_terminal "subscriber.py" "subscriber.py node"
