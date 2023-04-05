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
import launch
import launch_ros
import os

from ament_index_python.packages import get_package_share_directory

#from robotnik_common.launch import RewrittenYaml

# Environment variables
#  USE_SIM_TIME: Use simulation (Gazebo) clock if true
#  NAMESPACE: Namespace of the node stack.
#  ROBOT_ID: Frame id of the robot. (e.g. vectornav_link).
#  WORLD: World to load.

def read_params(ld : launch.LaunchDescription):
  use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
  robot_id = launch.substitutions.LaunchConfiguration('robot_id')
  namespace = launch.substitutions.LaunchConfiguration('namespace')
  pos_x = launch.substitutions.LaunchConfiguration('pos_x')
  pos_y = launch.substitutions.LaunchConfiguration('pos_y')

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='use_sim_time',
    description='Use simulation (Gazebo) clock if true',
    choices=['true', 'false'],
    default_value='true',
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='robot_id',
    description='Id of the robot',
    default_value='robot',
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='namespace',
    description='Namespace of the node stack',
    default_value=robot_id,
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='pos_x',
    description='X position of the robot',
    default_value='0.0',
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='pos_y',
    description='Y position of the robot',
    default_value='0.0',
  ))

  ret = {
    'use_sim_time': use_sim_time,
    'namespace': namespace,
    'robot_id': robot_id,
    'pos_x': pos_x,
    'pos_y': pos_y,
  }

  return ret


from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  ld = launch.LaunchDescription()
  summit_xl_gazebo = get_package_share_directory('summit_xl_gazebo')

  params = read_params(ld)

  namespace = launch_ros.actions.PushRosNamespace(namespace=params['namespace'])
  robot_state_publisher = launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(summit_xl_gazebo, 'launch', 'description.launch.py')
    ),
    launch_arguments={
      'use_sim_time': params['use_sim_time'],
      'robot_id': params['robot_id'],
    }.items(),
  )
  spawner = launch_ros.actions.Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
      '-entity', params['robot_id'],
      '-topic', 'robot_description',
      '-x', params['pos_x'],
      '-y', params['pos_y'],
      '-z', '0.10',
    ],
    output='screen',
  )
  base_controller = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    arguments=["robotnik_base_control", "--controller-manager", ["/", params['namespace'], "/controller_manager"]],
  )
  joint_broadcaster = launch_ros.actions.Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster", "--controller-manager", ["/", params['namespace'], "/controller_manager"]],
  )

  ld.add_action(launch.actions.GroupAction(actions=[
    namespace,
    robot_state_publisher,
    spawner,
    base_controller,
    joint_broadcaster,
  ]))

  return ld
