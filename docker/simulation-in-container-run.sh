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

function exec_environment_check() {
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
