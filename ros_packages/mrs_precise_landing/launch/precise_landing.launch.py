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

    pkg_name = 'mrs_precise_landing'
    this_pkg_path = get_package_share_directory(pkg_name)
    estimation_pkg_path = get_package_share_directory('mrs_landing_pad_estimation')

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

    # #{ controller_config

    controller_config_raw = LaunchConfiguration('controller_config')
    ld.add_action(DeclareLaunchArgument(
        'controller_config',
        default_value='',
        description='Custom controller configuration file.'
    ))

    controller_config = IfElseSubstitution(
        condition=PythonExpression(['"', controller_config_raw, '" != "" and ', 'not "', controller_config_raw, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), controller_config_raw]),
        else_value=controller_config_raw
    )

    # #} end of controller_config

    # #{ estimator_config

    estimator_config_raw = LaunchConfiguration('estimator_config')
    ld.add_action(DeclareLaunchArgument(
        'estimator_config',
        default_value='',
        description='Custom estimator configuration file.'
    ))

    # #} end of estimator_config

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', 'false'),
        description='Should the node subscribe to sim time?',
    ))

    # #} end of use_sim_time

    # Include landing_pad_estimation launch file
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(estimation_pkg_path, 'launch', 'landing_pad_estimation.launch.py')
        ),
        launch_arguments={
            'uav_name': uav_name,
            'camera_node': camera_node,
            'image_topic': image_topic,
            'apriltag_config': apriltag_config_raw,
            'estimator_config': estimator_config_raw,
            'use_sim_time': use_sim_time,
            'standalone': standalone,
            'container_name': container_name,
        }.items()
    ))

    # Precise landing node
    precise_landing_node = ComposableNode(
        package=pkg_name,
        plugin='mrs_precise_landing::PreciseLanding',
        namespace=uav_name,
        name='precise_landing',
        parameters=[
            {'uav_name': uav_name},
            {'use_sim_time': use_sim_time},
            {'config': PathJoinSubstitution([this_pkg_path, 'config', 'precise_landing.yaml'])},
            {'custom_config': controller_config},
        ],
        remappings=[
            # Subscribers
            ('landing_pad_in', 'landing_pad_estimation/pose_estimate'),
            ('tracker_cmd_in', 'control_manager/tracker_cmd'),
            ('uav_state_in', 'estimation_manager/uav_state'),
            ('mass_estimate_in', 'control_manager/mass_estimate'),
            ('ctrl_diag_in', 'control_manager/diagnostics'),
            # Service callers
            ('switch_controller_out', 'control_manager/switch_controller'),
            ('switch_tracker_out', 'control_manager/switch_tracker'),
            ('arming_out', 'control_manager/arm'),
            ('set_min_z_out', 'control_manager/set_min_z'),
            ('enable_min_height_check_out', 'uav_manager/enable_min_height_check'),
            ('path_out', 'trajectory_generation/path'),
            # Advertised services
            ('land_in', 'precise_landing/land'),
            ('abort_in', 'precise_landing/abort'),
            # Publishers
            ('trajectory_reference_out', 'control_manager/trajectory_reference'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    ld.add_action(LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[precise_landing_node],
        condition=UnlessCondition(standalone)
    ))

    ld.add_action(ComposableNodeContainer(
        namespace=uav_name,
        name='precise_landing_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[precise_landing_node],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(standalone)
    ))

    return ld
