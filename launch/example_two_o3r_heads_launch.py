# Copyright 2023 Sameer Tuteja
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_ifm3d_ros2 = get_package_share_directory('ifm3d_ros2')
    uncompressed = LaunchConfiguration('uncompressed', default='False')

    return LaunchDescription([

        DeclareLaunchArgument('uncompressed',
                              default_value='false',
                              description='Set to true if uncompressed image'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ifm3d_ros2, 'launch', 'camera.launch.py'),
            ),
            launch_arguments={
                'camera_name': 'left_camera_2d',
                'parameter_file_package': 'o3r_color_pcl_ros2',
                'parameter_file_directory': 'config',
                'parameter_file_name': 'two_o3r_heads.yaml',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ifm3d_ros2, 'launch', 'camera.launch.py'),
            ),
            launch_arguments={
                'camera_name': 'left_camera_3d',
                'parameter_file_package': 'o3r_color_pcl_ros2',
                'parameter_file_directory': 'config',
                'parameter_file_name': 'two_o3r_heads.yaml',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ifm3d_ros2, 'launch', 'camera.launch.py'),
            ),
            launch_arguments={
                'camera_name': 'right_camera_2d',
                'parameter_file_package': 'o3r_color_pcl_ros2',
                'parameter_file_directory': 'config',
                'parameter_file_name': 'two_o3r_heads.yaml',
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ifm3d_ros2, 'launch', 'camera.launch.py'),
            ),
            launch_arguments={
                'camera_name': 'right_camera_3d',
                'parameter_file_package': 'o3r_color_pcl_ros2',
                'parameter_file_directory': 'config',
                'parameter_file_name': 'two_o3r_heads.yaml',
            }.items()
        ),
        Node(
            package='o3r_color_pcl_ros2',
            executable='color_pcl_pub',
            name='color_pcl_pub_left_compressed',
            condition=UnlessCondition(LaunchConfiguration('uncompressed')),
            output='both',
            remappings=[

                # input

                ('/camera_2d/rgb_compressed', '/ifm3d/left_camera_2d/rgb'),
                ('/camera_2d/RGB_INFO', '/ifm3d/left_camera_2d/RGB_INFO'),
                ('/camera_3d/distance', '/ifm3d/left_camera_3d/distance'),
                ('/camera_3d/extrinsics', '/ifm3d/left_camera_3d/extrinsics'),
                ('/camera_3d/INTRINSIC_CALIB', \
                    '/ifm3d/left_camera_3d/INTRINSIC_CALIB'),

                # output

                ('/colored_pcl', '/pcl/left_camera_cloud'),
            ],
            parameters=[{'uncompressed': uncompressed}],
        ),
        Node(
            package='o3r_color_pcl_ros2',
            executable='color_pcl_pub',
            name='color_pcl_pub_left_uncompressed',
            condition=IfCondition(LaunchConfiguration('uncompressed')),
            output='both',
            remappings=[

                # input

                ('/camera_2d/rgb_uncompressed', \
                    '/ifm3d/left_camera_2d/rgb_uncompressed'),
                ('/camera_2d/RGB_INFO', '/ifm3d/left_camera_2d/RGB_INFO'),
                ('/camera_3d/distance', '/ifm3d/left_camera_3d/distance'),
                ('/camera_3d/extrinsics', '/ifm3d/left_camera_3d/extrinsics'),
                ('/camera_3d/INTRINSIC_CALIB', \
                    '/ifm3d/left_camera_3d/INTRINSIC_CALIB'),

                # output

                ('/colored_pcl', '/pcl/left_camera_cloud'),
            ],
            parameters=[{'uncompressed': uncompressed}],
        ),
        Node(
            package='o3r_color_pcl_ros2',
            executable='color_pcl_pub',
            name='color_pcl_pub_right_compressed',
            condition=UnlessCondition(LaunchConfiguration('uncompressed')),
            output='both',
            remappings=[

                # input

                ('/camera_2d/rgb_compressed', '/ifm3d/right_camera_2d/rgb'),
                ('/camera_2d/RGB_INFO', '/ifm3d/right_camera_2d/RGB_INFO'),
                ('/camera_3d/distance', '/ifm3d/right_camera_3d/distance'),
                ('/camera_3d/extrinsics', '/ifm3d/right_camera_3d/extrinsics'),
                ('/camera_3d/INTRINSIC_CALIB', \
                    '/ifm3d/right_camera_3d/INTRINSIC_CALIB'),

                # output

                ('/colored_pcl', '/pcl/right_camera_cloud'),
            ],
            parameters=[{'uncompressed': uncompressed}],
        ),
        Node(
            package='o3r_color_pcl_ros2',
            executable='color_pcl_pub',
            name='color_pcl_pub_right_uncompressed',
            condition=IfCondition(LaunchConfiguration('uncompressed')),
            output='both',
            remappings=[

                # input

                ('/camera_2d/rgb_uncompressed', \
                    '/ifm3d/right_camera_2d/rgb_uncompressed'),
                ('/camera_2d/RGB_INFO', '/ifm3d/right_camera_2d/RGB_INFO'),
                ('/camera_3d/distance', '/ifm3d/right_camera_3d/distance'),
                ('/camera_3d/extrinsics', '/ifm3d/right_camera_3d/extrinsics'),
                ('/camera_3d/INTRINSIC_CALIB', \
                    '/ifm3d/right_camera_3d/INTRINSIC_CALIB'),

                # output

                ('/colored_pcl', '/pcl/right_camera_cloud'),
            ],
            parameters=[{'uncompressed': uncompressed}],
        ),

    ])
