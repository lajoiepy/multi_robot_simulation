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
from launch.actions import TimerAction, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, PushLaunchConfigurations, PopLaunchConfigurations, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from nav2_common.launch import RewrittenYaml
#from ament_index_python import get_package_share_directory
import os

def generate_launch_description():

    num_robots = 5 # TODO: add parameter
    delay_between_robots_launch = 6.0 # TODO: add parameter

    # Fix tf2 frames
    tf2_static_nodes = []
    for i in range(num_robots):
        tf2_static_nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=['--frame-id', 'r' + str(i) + '/base_link', '--child-frame-id', 'r' + str(i) + '/laser1', '--x', '0.15', '--z', '0.25']
        ))

    # Nav2 launch
    pkg_dir = get_package_share_directory("multi_robot_simulation")
    nav2_launchs = []
    for i in range(num_robots):
        nav2_config_file = os.path.join(
            pkg_dir, 'config', 'nav2', 'default_multi', 'r' + str(i) + '.yaml')
        nav2_launchs.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("multi_robot_simulation"),
                                "launch", "mvsim_nav2_bringup.launch.py")),
                launch_arguments={
                    "use_namespace": "true",
                    "namespace": "r"+str(i),
                    "params_file": nav2_config_file,
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
    delay_s = 0.0
    # for tf2_static_node in tf2_static_nodes:
    #     ld.add_entity(PushLaunchConfigurations())
    #     ld.add_entity(tf2_static_node)
    #     ld.add_entity(PopLaunchConfigurations())
    for nav2 in nav2_launchs:
        ld.add_entity(PushLaunchConfigurations())
        ld.add_entity(TimerAction(period=delay_s, actions=[nav2]))
        ld.add_entity(PopLaunchConfigurations())
        delay_s += delay_between_robots_launch
    for frontiers_node in frontiers_nodes:
        ld.add_entity(PushLaunchConfigurations())
        ld.add_entity(TimerAction(period=delay_s, actions=[frontiers_node]))
        ld.add_entity(PopLaunchConfigurations())
        delay_s += delay_between_robots_launch + 1.0

    return ld
