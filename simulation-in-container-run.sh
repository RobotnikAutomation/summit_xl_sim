#!/bin/bash
#
# Description:   Summit XL simulation on docker
#                bring-up script
#
# Company:       Robotnik Automation S.L.L.
# Creation Year: 2021
# Author:        Guillem Gari  <ggari@robotnik.es>

function load_file() {
    local file="${1}"
    if ! test -r "${file}"; then
        echo "File not present: ${file} : Aborting" 2>&1
        return 1
    fi
    if ! source "${file}"; then
        echo "Could not load: ${file} : Aborting" 2>&1
        return 1
    fi
    return 0
}

function load_files() {
    previous_exec_path="$PWD"
    host_source_path="$(dirname "$(readlink -f "${0}")")"
    data_file="${0%.sh}"
    data_file="${data_file}.data"
    data_file="${host_source_path}/${data_file}"
    if ! load_file "${data_file}"; then
        return 1
    fi
    func_file="${0%.sh}"
    func_file="${func_file}.func"
    func_file="${host_source_path}/${func_file}"
    if ! load_file "${func_file}"; then
        return 1
    fi
    cd "$host_source_path"
    return 0
}

# VARIABLES
###########

#Colour
red_colour='\033[0;31m'
green_colour='\033[0;32m'
light_purple_colour='\033[1;35m'
err_colour="${red_colour}"
nfo_colour="${light_purple_colour}"
suc_colour="${green_colour}"
no_colour='\033[0m'

#Success String
suc_str_tool_check_success='All required tools are available'

#Info Strings
nfo_str_tool_checking='Checking tools'

#Error Strings
err_str_required_tool_not_found='Required tool not found: ${tool}'

tool_list=(\
    docker \
    readlink \
    dirname \
    xhost \
    nvidia_smi \
)


# FUNCTIONS
###########

function print_error() {
    local message="${1}"
    eval "echo -e "'"'"${err_colour}ERROR]${no_colour}:   ${message}"'"'" 2>&1"
}

function print_info() {
    local message="${1}"
    eval "echo -e "'"'"${nfo_colour}[INFO]${no_colour}:    ${message}"'"'""
}

function print_success() {
    local message="${1}"
    eval "echo -e "'"'"${suc_colour}[SUCCESS]${no_colour}: ${message}"'"'""
}

function set_path() {
    host_source_path="$(dirname "$(readlink -f "${0}")")"
    build_path="${host_source_path}"
    return 0
}

function tool_check() {
    local binary="${1}"
    if [[ -z "${binary}" ]];then
        return 1
    fi
    eval "${tool_check_cmd}"
    return $?
}

function tools_check() {
    local tools=("${@}")
    print_info "${nfo_str_tool_checking}"
    for tool in "${tools[@]}"; do
        if ! tool_check "${tool}"; then
            print_error "${err_str_required_tool_not_found}"
            return 1
        fi
    done
    print_success "${suc_str_tool_check_success}"
    return 0
}

function build_image() {
    docker build -t summit_xl_sim .
    return $?
}

function check_docker_instance_already_running() {
    docker container ls -a | sed '1d' | awk '{print $2}' | grep -q ^summit_xl_sim$
    return $?
}

function delete_running_docker_instance() {
    docker container rm --force summit_xl_sim
    return $?
}

function allow_screen() {
    xhost + local:root
    return $?
}

function disable_screen() {
    xhost - local:root
    return $?
}

function run_simulation() {
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
    return $?
}

function exec_environment_check() {
    return 0
}

function simulation_flow() {
    if ! exec_environment_check; then
        return 1
    fi
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
    local return_value=1
    if ! load_files; then
        cd "${previous_exec_path}"
        return 1
    fi
    simulation_flow
    return_value=$?
    if ! disable_screen; then
        return_value=1
    fi
    cd "${previous_exec_path}"
    return "${return_value}"
}

simulation_main "$@"
exit $?
