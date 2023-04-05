# Copyright (c) 2023, Robotnik Automation S.L.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

from robotnik_common.launch import RewrittenYaml

def read_params(ld : launch.LaunchDescription):
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    controllers_file = launch.substitutions.LaunchConfiguration('controllers_file')
    robot_id = launch.substitutions.LaunchConfiguration('robot_id')

    # Declare the launch options
    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        description='Use simulation (Gazebo) clock if true',
        choices=['true', 'false'],
        default_value='true')
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='controllers_file',
        description='ROS 2 controller file.',
        default_value=[get_package_share_directory('summit_xl_gazebo'), '/config/controller.yml'])
    )

    ld.add_action(launch.actions.DeclareLaunchArgument(
        name='robot_id',
        description='Robot ID used to create the robot namespace',
        default_value='robot')
    )

    # Parse the launch options
    return {
        'use_sim_time': use_sim_time,
        'robot_description_path': os.path.join(get_package_share_directory('summit_xl_description'), 'robots', 'summit_xl.urdf.xacro'),
        'robot_id': robot_id,
        'controllers_file': controllers_file,
    }

def generate_launch_description():

    ld = launch.LaunchDescription()

    params = read_params(ld)

    config_file_rewritten = RewrittenYaml(
        source_file=params['controllers_file'],
        param_rewrites={},
        root_key=[params['robot_id'],],
        convert_types=True,
    )

    robot_description_content = launch.substitutions.Command(
        [
            launch.substitutions.PathJoinSubstitution(
                [launch.substitutions.FindExecutable(name="xacro")]),
            " ",
            params['robot_description_path'],
            " robot_id:=", params['robot_id'],
            " robot_ns:=", params['robot_id'],
            " config_controllers:=", config_file_rewritten,
        ]
    )

    # Create parameter
    robot_description_param = launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': params['use_sim_time'],
            'robot_description': robot_description_param,
            'publish_frequency': 100.0,
            'frame_prefix': [params['robot_id'], '/'],
        }],
    )

    ld.add_action(robot_state_publisher)

    return ld
