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

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from nav2_common.launch import RewrittenYaml
#from ament_index_python import get_package_share_directory
import os

def generate_launch_description():

    num_robots = 1 # TODO: add parameter

    # Nav2 launch
    nav2_launchs = []
    for i in range(num_robots):
        nav2_launchs.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("multi_robot_simulation"),
                                "launch", "mvsim_nav2_bringup.launch.py")),
                launch_arguments={
                    "use_sim_time": "false",
                    "use_namespace": "true",
                    "namespace": "r"+str(i),
                }.items(),
            ))
    
    # Frontiers exploration nodes
    frontiers_nodes = []
    for i in range(num_robots):
        frontiers_nodes.append(Node(
            package='simple_frontiers_exploration',
            executable='explore',
            name='simple_frontiers_exploration',
            output='screen',
            namespace="r"+str(i),
            parameters=[
            ]
        ))

    # Create the launch description and populate
    ld = LaunchDescription()
    for nav2 in nav2_launchs:
        ld.add_entity(nav2)
    for frontiers_node in frontiers_nodes:
        ld.add_entity(frontiers_node)

    return ld
