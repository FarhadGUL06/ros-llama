# Introduction

Llama 3.2 Q8 instead of Gemma3 (inference time issue)

ROS 1 project where I have one node that runs llama 3.2 model to parse text and transform it to commands like ['pick', 'glass', 'place', 'table']. I

```
sudo docker stop $(sudo docker ps -aq)
sudo docker rm $(sudo docker ps -aq)
sudo docker rmi $(sudo docker images -q)

```

To build the image
```
sudo docker build -t ros-llama .



sudo docker run -it --rm \
  -v /home/farhad/ros-llama/llama-python:/root \
  --workdir /root/catkin_ws \
  ros-llama


```