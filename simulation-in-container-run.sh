#!/bin/bash
docker build -t summit_xl_sim .
xhost + local:root
echo "roslaunch summit_xl_sim_bringup summit_xl_complete.launch"
if docker container ls -a | sed '1d' | awk '{print $2}' | grep -q ^summit_xl_sim$; then
    docker container rm --force summit_xl_sim
fi
docker run --gpus all --rm \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --device=/dev/dri \
    --group-add video \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env ROS_MASTER_URI="http://localhost:11311" \
    --env GAZEBO_MASTER_URI="http://localhost:11345" \
    --env NVIDIA_VISIBLE_DEVICES=0 \
    --name summit_xl_sim \
    summit_xl_sim:latest \
    bash -c "source /opt/ros/melodic/setup.bash &&
        source /home/ros/catkin_ws/devel/setup.bash &&
        roslaunch summit_xl_sim_bringup summit_xl_complete.launch"
xhost - local:root
