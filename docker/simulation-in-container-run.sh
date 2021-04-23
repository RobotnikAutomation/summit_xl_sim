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
    # if ! source "${file}"; then
    #     echo "Could not load: ${file} : Aborting" 2>&1
    #     return 1
    # fi
    return 0
}

function load_files() {
    previous_exec_path="$PWD"
    executable="$(basename "${0}")"
    host_source_path="$(dirname "$(readlink -f "${0}")")"
    data_file="${executable%.sh}"
    data_file="${data_file}.data"
    data_file="${host_source_path}/${data_file}"
    if ! load_file "${data_file}"; then
        return 1
    fi
    func_file="${executable%.sh}"
    func_file="${func_file}.func"
    func_file="${host_source_path}/${func_file}"
    if ! load_file "${func_file}"; then
        return 1
    fi
    cd "$host_source_path"
    return 0
}

function exec_environment_check() {
    if ! select_simulation; then
        return 1
    fi
    if ! tools_check "${tool_list[@]}"; then
        return 1
    fi
    if ! check_if_there_is_display; then
        return 1
    fi
    if ! get_nvidia_cuda; then
        return 1
    fi
    if ! test_nvidia_docker; then
        return 1
    fi
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

function parse_arguments() {
    local key=""
    while [[ $# -gt 0 ]]; do
        key="${1}"
        case "${key}" in
            --robot|-r)
                selected_robot="${2}"
                shift
                ;;
            --launch|-l)
                selected_launch_file="${2}"
                shift
                ;;
            --package|-p)
                selected_package="${2}"
                shift
                ;;
            --ros-port|-u)
                ros_master_port="${2}"
                ros_master_uri="http://localhost:${ros_master_port}"
                shift
                ;;
            --gazebo-port|-g)
                gazebo_master_port="${2}"
                gazebo_master_uri="http://localhost:${gazebo_master_port}"
                shift
                ;;
            --help|-h)
                help
                exit 0
                ;;
            *)
                echo "Unkwon Command"
                help
                exit 1
                ;;

        esac
        shift
    done
    return 0
}

function help() {
cat << EOF
ROBOTNIK AUTOMATION S.L.L. 2021

Simulation of ${robot_name} using docker

Usage:
${0} [OPTIONS]

Optional arguments:
 --robot -r ROBOT       Select robot to simulate
                        Valid robots:
                            ${!ros_bringup_package_array[@]}
                        default: ${default_robot}

 --launch -l            Select launch file
                        default: ${ros_launch_file_array[${default_robot}]}

 --package -p           Select ros package
                        default: ${ros_bringup_package_array[${default_robot}]}

 --ros-port -u PORT     Host ros port
                        default: ${ros_master_port}

 --gazebo-port -g PORT  Host ros port
                        default: ${gazebo_master_port}

 -h, --help             Shows this help

EOF
    return 0
}

function simulation_main() {
    local return_value=1
    if ! parse_arguments "${@}"; then
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

if ! load_files; then
    cd "${previous_exec_path}"
    exit 1
fi

if ! source "${data_file}"; then
    echo "Could not load: ${data_file} : Aborting" 2>&1
    exit 1
fi

if ! source "${func_file}"; then
    echo "Could not load: ${func_file} : Aborting" 2>&1
    exit 1
fi

simulation_main "$@"
exit $?
