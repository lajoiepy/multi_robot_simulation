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
from launch.actions import TimerAction, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, PushLaunchConfigurations, PopLaunchConfigurations
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution
from nav2_common.launch import RewrittenYaml
#from ament_index_python import get_package_share_directory
import os

def generate_launch_description():

    num_robots = 10 # TODO: add parameter
    delay_between_robots_launch = 10.0 # TODO: add parameter

    # Swarm-SLAM launch
    cslam_processes = []
    for i in range(num_robots):
        cslam_config_file = 'jackal_lidar.yaml'
        cslam_processes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("multi_robot_simulation"),
                                "launch", "swarm_slam.launch.py")),
                launch_arguments={
                    "namespace": "/r"+str(i),
                    "robot_id": str(i),
                    "max_nb_robots": str(num_robots),
                    "config_file": cslam_config_file,
                }.items(),
            ))

    # Create the launch description and populate
    ld = LaunchDescription()
    delay_s = 0.0
    for cslam in cslam_processes:
        ld.add_entity(PushLaunchConfigurations())
        ld.add_entity(TimerAction(period=delay_s, actions=[cslam]))
        ld.add_entity(PopLaunchConfigurations())
        delay_s += delay_between_robots_launch

    return ld
