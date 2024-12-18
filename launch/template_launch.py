# Copyright (c) 2018 Intel Corporation
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
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
	
    load_nodes = GroupAction(
        actions=[
            Node(
                package='lidarfilter_node',
                executable='lidarfilter_node',
                name='lidarfilter_node',
                output='screen',
                parameters =[os.path.join(get_package_share_directory('lidarfilter_node'),'params', 'config.yaml')],
                )
        ]
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(load_nodes)

    return ld
