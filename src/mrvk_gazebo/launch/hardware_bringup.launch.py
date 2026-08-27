import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('mrvk_gazebo')
    pkg_desc = get_package_share_directory('mrvk_description')

    default_model = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')
    default_rviz = os.path.join(pkg_gazebo, 'config', 'rviz', 'config.rviz')
    default_p2l_params = os.path.join(
        pkg_gazebo, 'config', 'laserscan', 'pointcloud_to_laserscan.yaml')

    model = LaunchConfiguration('model')
    use_rviz = LaunchConfiguration('rviz')

    robot_description = {
        'robot_description': Command(['xacro ', model]),
        'use_sim_time': False,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
    )

    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[default_p2l_params, {'use_sim_time': False}],
        remappings=[
            ('cloud_in', '/cloud_registered_body'),
            ('scan', '/scan'),
        ],
    )

    # PLACEHOLDER: livox_ros_driver not yet vendored
    livox_driver = Node(
        package='livox_ros_driver',
        executable='livox_ros_driver_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # PLACEHOLDER: fast_lio not yet vendored
    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # PLACEHOLDER: ublox_gps not yet vendored
    ublox_gps = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # PLACEHOLDER: stm32_serial_bridge location unknown
    stm32_serial_bridge = Node(
        package='stm32_serial_bridge',
        executable='stm32_serial_bridge_node',
        name='stm32_serial_bridge',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument('rviz', default_value='true'),
        robot_state_publisher,
        joint_state_publisher,
        rviz,
        pointcloud_to_laserscan,
        livox_driver,
        fast_lio,
        ublox_gps,
        stm32_serial_bridge,
    ])
