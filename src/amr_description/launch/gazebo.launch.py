#!/usr/bin/env python3
"""
Launch Gazebo Fortress (Ignition) simulation cho AMR Ackermann.

Thứ tự khởi động:
  1. robot_state_publisher (publish robot_description + TF tĩnh)
  2. ign gazebo  (simulator)
  3. spawn entity (đưa robot vào world)
  4. joint_state_broadcaster + ackermann_steering_controller
  5. ros_gz_bridge (bridge /clock, /scan, v.v.)
"""
import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_description')

    # ── Tham số launch ────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='World file cho Ignition Gazebo')

    gz_headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Chạy Gazebo không GUI (true/false)')

    # ── Robot description từ xacro ────────────────────────────────
    urdf_file = os.path.join(pkg_share, 'urdf', 'amr.urdf.xacro')
    robot_description_xml = xacro.process_file(urdf_file).toxml()

    # ── Nodes ─────────────────────────────────────────────────────

    # 1. robot_state_publisher — pub /robot_description + TF tĩnh
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_xml,
            'use_sim_time': True,
        }],
    )

    # 2. Ignition Gazebo server + GUI
    #    Dùng LaunchConfiguration để nhận world argument
    gazebo = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r',
            LaunchConfiguration('world'),
        ],
        output='screen',
    )

    # 3. Spawn robot từ topic /robot_description
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_amr',
        arguments=[
            '-name', 'amr_ackermann',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',       # spawn hơi cao hơn mặt đất để tránh overlap
        ],
        output='screen',
    )

    # 4a. Spawn joint_state_broadcaster (cần spawner từ ros2_controllers)
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_jsb',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    # 4b. Spawn ackermann_steering_controller — sau khi jsb đã load xong
    spawn_ackermann = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_ackermann',
        arguments=['ackermann_steering_controller'],
        output='screen',
    )

    # Chờ jsb load xong rồi mới spawn ackermann
    load_ackermann_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jsb,
            on_exit=[spawn_ackermann],
        )
    )

    # 5. ros_gz_bridge — bridge /clock trước, các topic khác sau
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen',
    )

    # Bridge chờ spawn robot xong để tránh TF race condition
    delayed_bridge = TimerAction(
        period=3.0,   # giây — đủ thời gian Gazebo load world
        actions=[gz_bridge],
    )

    return LaunchDescription([
        world_arg,
        gz_headless_arg,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        spawn_jsb,
        load_ackermann_after_jsb,
        delayed_bridge,
    ])
