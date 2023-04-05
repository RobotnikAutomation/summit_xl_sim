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
  environment = launch.substitutions.LaunchConfiguration('environment')
  use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
  robot_id = launch.substitutions.LaunchConfiguration('robot_id')
  namespace = launch.substitutions.LaunchConfiguration('namespace')
  world_name = launch.substitutions.LaunchConfiguration('world_name')
  world = launch.substitutions.LaunchConfiguration('world')

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='environment',
    description='Read parameters from environment variables',
    choices=['true', 'false'],
    default_value='true',
  ))

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
    name='world_name',
    description='Name of the world to load',
    default_value='empty',
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='world',
    description='World to load',
    default_value=[launch_ros.substitutions.FindPackageShare('summit_xl_gazebo'), '/worlds/', world_name, '.world']
  ))

  ret = {}

  if environment == 'false':
    ret = {
      'use_sim_time': use_sim_time,
      'namespace': namespace,
      'robot_id': robot_id,
      'world': world,
    }
  else:
    if 'USE_SIM_TIME' in os.environ:
      ret['use_sim_time'] = os.environ['USE_SIM_TIME']
    else:
      ret['use_sim_time'] = use_sim_time
    if 'NAMESPACE' in os.environ:
      ret['namespace'] = os.environ['NAMESPACE']
    elif 'ROBOT_ID' in os.environ:
      ret['namespace'] = os.environ['ROBOT_ID']
    else:
      ret['namespace'] = namespace
    if 'ROBOT_ID' in os.environ:
      ret['robot_id'] = os.environ['ROBOT_ID']
    else:
      ret['robot_id'] = robot_id
    if 'WORLD' in os.environ:
      ret['world'] = os.environ['WORLD']
    elif 'WORLD_NAME' in os.environ:
      ret['world'] = [launch_ros.substitutions.FindPackageShare('summit_xl_gazebo'), '/worlds/', os.environ['WORLD_NAME'], '.world']
    else:
      ret['world'] = world

  return ret


from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  ld = launch.LaunchDescription()
  summit_xl_gazebo = get_package_share_directory('summit_xl_gazebo')
  gazebo_ros = get_package_share_directory('gazebo_ros')

  params = read_params(ld)

  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')
    ),
    launch_arguments={
      'verbose': 'false',
      'world': params['world'],
      'paused': 'false',
      'init': 'true',
      'factory': 'true',
      'force_system': 'true',
      'params_file': os.path.join(summit_xl_gazebo, 'config','gazebo.yml'),
    }.items(),
  ))

  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')
    ),
    launch_arguments={
      'verbose': 'false',
      }.items(),
  ))

  ld.add_action(launch.actions.IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(summit_xl_gazebo, 'launch', 'spawn.launch.py')
    ),
    launch_arguments={
      'use_sim_time': params['use_sim_time'],
      'robot_id': params['robot_id'],
      'namespace': params['namespace'],
      'pos_x': '1.0',
      'pos_y': '1.0',
      }.items(),
  ))

  return ld