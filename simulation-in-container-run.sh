#!/bin/bash
#
# Description:   Summit XL simulation on docker
#                bring-up script
#
# Company:       Robotnik Automation S.L.L.
# Creation Year: 2021
# Author:        Guillem Gari  <ggari@robotnik.es>

function print_error() {
    return 0
}

function print_info() {
    return 0
}

function print_success() {
    return 0
}

function tool_check() {
    return 0
}

function tools_check() {
    return 0
}

function build_image() {
    return 0
}

function check_docker_instance_already_running() {
    return 0
}

function delete_running_docker_instance() {
    return 0
}

function allow_screen() {
    return 0
}

function disable_screen() {
    return 0
}

function run_simulation() {
    return 0
}

function simulation_flow() {
    if ! build_image; then
        return 1
    fi
    if ! allow_screen; then
        return 1
    fi
    if check_docker_instance_already_running; then
        if ! delete_running_docker_instance; then
            return 1
        fi
    fi
    if ! run_simulation; then
        return 1
    fi
    return 0
}

function simulation_main() {
    if ! simulation_flow; then
        return 1
    fi
    if ! disable_screen; then
        return 1
    fi
    return 0
}

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
