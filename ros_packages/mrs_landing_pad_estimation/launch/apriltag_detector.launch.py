#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = 'mrs_landing_pad_estimation'
    this_pkg_path = get_package_share_directory(pkg_name)

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')
    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', 'uav1'),
        description='The UAV name used for namespacing.',
    ))

    # #} end of uav_name

    # #{ standalone

    standalone = LaunchConfiguration('standalone')
    ld.add_action(DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start standalone or load into an existing container.'
    ))

    # #} end of standalone

    # #{ container_name

    container_name = LaunchConfiguration('container_name')
    ld.add_action(DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    ))

    # #} end of container_name

    # #{ camera_node

    camera_node = LaunchConfiguration('camera_node')
    ld.add_action(DeclareLaunchArgument(
        'camera_node',
        default_value='bluefox_optflow',
        description='Name of the camera node'
    ))

    # #} end of camera_node

    # #{ image_topic

    image_topic = LaunchConfiguration('image_topic')
    ld.add_action(DeclareLaunchArgument(
        'image_topic',
        default_value='image_raw',
        description='Image topic name'
    ))

    # #} end of image_topic

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value='',
        description='Path to custom configuration file. The path can be absolute (starting with "/") or relative to the current working directory.',
    ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
        condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
        else_value=custom_config
    )

    # #} end of custom_config

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', 'false'),
        description='Should the node subscribe to sim time?',
    ))

    # #} end of use_sim_time

    apriltag_node = ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagNode',
        namespace=uav_name,
        name='apriltag_detector',
        parameters=[
            {'config': this_pkg_path + '/config/apriltag_recursive.yaml'},
            {'custom_config': custom_config},
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('image_rect', [camera_node, '/', image_topic]),
            ('camera_info', [camera_node, '/camera_info']),
            ('tag_detections', 'apriltag_detector/tag_detections'),
            ('tag_detections_image', 'apriltag_detector/tag_detections_image'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    ld.add_action(LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[apriltag_node],
        condition=UnlessCondition(standalone)
    ))

    ld.add_action(ComposableNodeContainer(
        namespace=uav_name,
        name='apriltag_detector_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        composable_node_descriptions=[apriltag_node],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(standalone)
    ))

    return ld
