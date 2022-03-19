# MIT License
#
# Copyright (c) 2021 David Cutting, Avery Girven
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ground_point_model_threshold = LaunchConfiguration('ground_point_model_threshold', default='0.1')
    publish_debug_cloud = LaunchConfiguration('publish_debug_cloud', default='false')

    pcpl = Node(
        package='pcpl',
        executable='pcpl',
        name='pcpl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'ground_point_model_threshold': ground_point_model_threshold,
            'debug_cloud': publish_debug_cloud
        }])

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='false',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('ground_point_model_threshold',
                              default_value='0.1',
                              description='Distance a point may be from the ground plane to be considered a ground point.'),
        DeclareLaunchArgument('publish_debug_cloud',
                              default_value='false',
                              description='Publish all pointclouds for visualization (faster without)'),

        # Nodes
        pcpl,
    ])
