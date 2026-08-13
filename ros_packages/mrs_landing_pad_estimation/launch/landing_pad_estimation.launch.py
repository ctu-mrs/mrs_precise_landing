#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # #{ apriltag_config

    apriltag_config_raw = LaunchConfiguration('apriltag_config')
    ld.add_action(DeclareLaunchArgument(
        'apriltag_config',
        default_value='',
        description='Custom apriltag configuration file.'
    ))

    # #} end of apriltag_config

    # #{ estimator_config

    estimator_config_raw = LaunchConfiguration('estimator_config')
    ld.add_action(DeclareLaunchArgument(
        'estimator_config',
        default_value='',
        description='Custom estimator configuration file.'
    ))

    estimator_config = IfElseSubstitution(
        condition=PythonExpression(['"', estimator_config_raw, '" != "" and ', 'not "', estimator_config_raw, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), estimator_config_raw]),
        else_value=estimator_config_raw
    )

    # #} end of estimator_config

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', 'false'),
        description='Should the node subscribe to sim time?',
    ))

    # #} end of use_sim_time

    # Include apriltag_detector launch file
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(this_pkg_path, 'launch', 'apriltag_detector.launch.py')
        ),
        launch_arguments={
            'uav_name': uav_name,
            'camera_node': camera_node,
            'image_topic': image_topic,
            'custom_config': apriltag_config_raw,
            'use_sim_time': use_sim_time,
            'standalone': standalone,
            'container_name': container_name,
        }.items()
    ))

    # Landing pad estimation node
    estimation_node = ComposableNode(
        package=pkg_name,
        plugin='mrs_landing_pad_estimation::LandingPadEstimation',
        namespace=uav_name,
        name='landing_pad_estimation',
        parameters=[
            {'uav_name': uav_name},
            {'use_sim_time': use_sim_time},
            {'config': this_pkg_path + '/config/landing_pad_estimation.yaml'},
            {'custom_config': estimator_config},
        ],
        remappings=[
            ('tag_detections_in', 'apriltag_detector/tag_detections'),
            ('estimated_pose_out', 'landing_pad_estimation/pose_estimate'),
            ('measurement_pose_out', 'landing_pad_estimation/pose_measurement'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    ld.add_action(LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[estimation_node],
        condition=UnlessCondition(standalone)
    ))

    ld.add_action(ComposableNodeContainer(
        namespace=uav_name,
        name='landing_pad_estimation_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        output='screen',
        composable_node_descriptions=[estimation_node],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(standalone)
    ))

    return ld
