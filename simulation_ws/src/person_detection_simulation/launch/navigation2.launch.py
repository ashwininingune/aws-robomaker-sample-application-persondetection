# Copyright 2019 Open Source Robotics Foundation, Inc.
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
# limitations under the License

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_path = os.path.join(
        # get_package_share_directory('aws_robomaker_small_house_world'),   
        # # '/home', 'user',  'longnameofthepath_longnameofthepath_longnameofthepath_longnameofthepath_longnameo',    
        #     'maps', 'turtlebot3_waffle_pi',
            # "/home/user/",
            os.environ['HOME'],
            'map.yaml')
    map_dir = LaunchConfiguration(
        'map',
        default=map_path)
    #node_parameters = [use_sim_time, map_dir]
    navigation2_dir = get_package_share_directory('turtlebot3_navigation2')
    navigation2_launch_dir = os.path.join(navigation2_dir, 'launch')
   
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(navigation2_launch_dir, 'navigation2.launch.py')),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time}.items(),)
    ])

