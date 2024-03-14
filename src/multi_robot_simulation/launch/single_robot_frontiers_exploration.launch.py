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
#from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    # Nav2 launch
    nav2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("multi_robot_simulation"),
                             "launch", "mvsim_nav2_bringup.launch.py")),
            launch_arguments={
            }.items(),
        )

    # Frontiers exploration nodes
    frontiers_node = Node(
        package='simple_frontiers_exploration',
        executable='explore',
        name='simple_frontiers_exploration',
        output='screen',
        parameters=[]
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_entity(nav2)
    ld.add_entity(frontiers_node)

    return ld
