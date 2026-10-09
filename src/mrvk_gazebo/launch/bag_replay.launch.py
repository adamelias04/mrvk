import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('mrvk_gazebo')
    pkg_desc = get_package_share_directory('mrvk_description')

    default_model = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')
    default_rviz = os.path.join(pkg_gazebo, 'config', 'rviz', 'bag_replay.rviz')
    default_predictor_params = os.path.join(
        pkg_gazebo, 'config', 'predictor', 'dynamic_obstacle_predictor.yaml')
    default_ground_seg_params = os.path.join(
        pkg_gazebo, 'config', 'ground_segmentation', 'ground_segmentation_params.yaml')

    bag_path = LaunchConfiguration('bag_path')
    qos_overrides_path = LaunchConfiguration('qos_overrides_path')
    model = LaunchConfiguration('model')
    planner = LaunchConfiguration('planner')
    use_rviz = LaunchConfiguration('rviz')
    use_predict = LaunchConfiguration('predict')

    robot_description = {
        'robot_description': Command(['xacro ', model]),
        'use_sim_time': True,
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )


    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, 'launch', 'nav2.launch.py')),
        launch_arguments={'planner': planner}.items(),
    )

    # Not conditional on `predict`: the Nav2 obstacle layer now takes its
    # obstacles from this cloud too, not just the predictor.
    ground_segmentation = Node(
        package='ground_segmentation_ros2', executable='ground_segmentation_ros2_node',
        name='ground_segmentation', output='screen',
        parameters=[default_ground_seg_params, {'use_sim_time': True}],
        remappings=[
            ('/ground_segmentation/input_pointcloud', '/cloud_registered_body'),
        ],
    )

    dynamic_obstacle_predictor = Node(
        package='mrvk_gazebo', executable='dynamic_obstacle_predictor',
        name='dynamic_obstacle_predictor', output='screen',
        condition=IfCondition(use_predict),
        parameters=[default_predictor_params, {'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock',
            '--disable-keyboard-controls',
            '--qos-profile-overrides-path', qos_overrides_path,
        ],
        output='screen',
    )

    return LaunchDescription([
        # Replay is entirely local, so pin the middleware instead of
        # inheriting whatever the shell was last pointed at. A shell left
        # configured for the robot (RMW_IMPLEMENTATION=rmw_zenoh_cpp with
        # ZENOH_CONFIG_OVERRIDE naming the robot's router) makes every node
        # block in rmw init waiting for a router that is not on this network:
        # all processes start, nothing is ever printed, and the launch just
        # hangs. Override `rmw` if you really do want to replay over zenoh --
        # then a router has to be running and ZENOH_CONFIG_OVERRIDE set.
        DeclareLaunchArgument('rmw', default_value='rmw_fastrtps_cpp',
                              description='RMW for the replay processes'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw')),
        SetEnvironmentVariable('ZENOH_CONFIG_OVERRIDE', ''),

        DeclareLaunchArgument('bag_path', default_value='/home/adam/ws/rosbags/pivo0001'),
        DeclareLaunchArgument('qos_overrides_path',
                              default_value=PathJoinSubstitution([bag_path, 'qos_overrides.yaml'])),
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument('planner', default_value='teb',
                              description='Local planner: dwb | teb | tr'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('predict', default_value='true',
                              description='Run dynamic_obstacle_predictor'),
        robot_state_publisher,
        nav2,
        ground_segmentation,
        dynamic_obstacle_predictor,
        rviz,
        bag_play,
    ])
