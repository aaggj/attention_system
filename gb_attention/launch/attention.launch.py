# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('gb_attention')
    config_dir = os.path.join(bringup_dir, 'params')
    config_file = os.path.join(config_dir, 'attention_config.yaml')

    namespace = LaunchConfiguration('namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_USE_STDOUT', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    attention_manager_cmd = Node(
        package='gb_attention',
        executable='attention_server',
        output='screen',
        namespace=namespace,
        parameters=[
          config_file
        ])

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    ld.add_action(attention_manager_cmd)

    return ld
