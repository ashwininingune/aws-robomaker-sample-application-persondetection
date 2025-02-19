# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import yaml
import shutil
import launch
import launch_ros.actions
from launch.substitutions import EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
CONFIG = "routes"

def generate_launch_description():

    gui = launch.actions.DeclareLaunchArgument(
             'gui',
             default_value='false',
             description='Argument for GUI Display',
        )
    default_config = os.path.join(get_package_share_directory('person_detection_simulation'),
    'routes', 'route.yaml')

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
        CONFIG,
        default_value=default_config
        )
    ])
    node_parameters = [launch.substitutions.LaunchConfiguration(CONFIG)]
    rl_agent = launch_ros.actions.Node(
             package='aws_robomaker_simulation_common', 
             node_executable='route_manager', 
             output='screen',
             node_name='route_manager', name='route_manager',
             parameters=[{
                # Route file is passed as "<package_name>.<relative path in install space>" due to limitations on string parameter size.
                'route_file': '.'.join(['person_detection_simulation', os.path.join('routes', 'route.yaml')])
            }]
            )

    ld.add_action(rl_agent)

    return ld

if __name__ == '__main__':
    generate_launch_description()
