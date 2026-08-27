"""
Launch file for the full aruco_localisation pipeline.

Plan step 15: starts detector, tf_publisher, and optitrack_transform
together (and provides a hook to add a camera driver node), all sharing
the config/params.yaml parameter file.

Usage:
    ros2 launch aruco_localisation aruco_localisation.launch.py
    ros2 launch aruco_localisation aruco_localisation.launch.py enable_optitrack:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('aruco_localisation')
    default_params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the aruco_localisation parameters YAML file.',
    )
    enable_optitrack_arg = DeclareLaunchArgument(
        'enable_optitrack',
        default_value='true',
        description='Whether to start the optitrack_transform node.',
    )

    params_file = LaunchConfiguration('params_file')

    detector_node = Node(
        package='aruco_localisation',
        executable='detector',
        name='detector',
        output='screen',
        parameters=[params_file],
    )

    tf_publisher_node = Node(
        package='aruco_localisation',
        executable='tf_publisher',
        name='tf_publisher',
        output='screen',
        parameters=[params_file],
    )

    # IfCondition lets `enable_optitrack:=false` skip OptiTrack entirely,
    # e.g. for a minimal camera-only test (plan step 21).
    optitrack_transform_node = Node(
        package='aruco_localisation',
        executable='optitrack_transform',
        name='optitrack_transform',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('enable_optitrack')),
    )

    return LaunchDescription([
        params_file_arg,
        enable_optitrack_arg,
        detector_node,
        tf_publisher_node,
        optitrack_transform_node,
        # NOTE: add your camera driver / image source node here, e.g. a
        # usb_cam or v4l2_camera node publishing on the configured
        # image_topic, so the pipeline has an actual image source.
    ])
