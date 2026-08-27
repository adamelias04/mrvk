import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mrvk_gazebo')

    predictor_params = os.path.join(pkg, 'config', 'predictor', 'dynamic_obstacle_predictor.yaml')
    ground_seg_params = os.path.join(pkg, 'config', 'ground_segmentation', 'ground_segmentation_params.yaml')

    planner = LaunchConfiguration('planner')
    odom_topic = LaunchConfiguration('odom_topic')
    safety_filter = LaunchConfiguration('safety_filter')

    cmd_topic = PythonExpression(
        ["'/cmd_vel_raw' if '", safety_filter, "' == 'true' else '/shoddy/cmd_vel'"])

    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'hardware_bringup.launch.py')),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'nav2.launch.py')),
        launch_arguments={'planner': planner, 'cmd_vel_out': cmd_topic}.items(),
    )

    # Real-hardware sensor pipeline for the predictor: raw Livox cloud ->
    # ground_segmentation (ground/obstacle split) -> predictor consumes
    # obstacle_points directly. Mirrors bag_replay.launch.py.
    ground_segmentation = Node(
        package='ground_segmentation_ros2', executable='ground_segmentation_ros2_node',
        name='ground_segmentation', output='screen',
        condition=IfCondition(safety_filter),
        parameters=[ground_seg_params, {'use_sim_time': False}],
        remappings=[
            ('/ground_segmentation/input_pointcloud', '/cloud_registered_body'),
        ],
    )

    dynamic_obstacle_predictor = Node(
        package='mrvk_gazebo', executable='dynamic_obstacle_predictor',
        name='dynamic_obstacle_predictor', output='screen',
        condition=IfCondition(safety_filter),
        parameters=[predictor_params, {'use_sim_time': False}],
    )

    robot_motion_predictor = Node(
        package='mrvk_gazebo', executable='robot_motion_predictor',
        name='robot_motion_predictor', output='screen',
        condition=IfCondition(safety_filter),
        parameters=[{
            'use_sim_time': False,
            'cmd_vel_in_topic': '/cmd_vel_raw',
            'cmd_vel_out_topic': '/shoddy/cmd_vel',
            'odom_topic': odom_topic,
            'local_costmap_topic': '/local_costmap/costmap',
            'prediction_path_topic': '/robot_motion_prediction',
            'prediction_marker_topic': '/robot_motion_prediction_marker',
            'prediction_time': 4.0,
            'viz_rate': 20.0,
            'marker_line_width': 0.05,
            'marker_z': 0.10,
            'prediction_dt': 0.1,
            'slow_horizon': 1.0,
            'stop_horizon': 0.4,
            'reverse_horizon': 0.2,
            'lethal_threshold': 85,
            'reverse_velocity': -0.15,
            'reverse_clear_probe_distance': 0.45,
            'publish_prediction_path': True,
            'obstacle_path_topic': '/predicted_obstacle_path',
            'obstacle_prediction_dt': 0.1,
            'temporal_safety_distance': 0.48,
            'temporal_path_max_age': 1.0,
            'temporal_prediction_time': 4.0,
            'temporal_match_window': 2.0,
            'escape_horizon': 1.5,
            'escape_linear_velocity': 1.5,
            'escape_angular_velocity': 0.6,
            'headon_reverse_velocity': -1.1,
            'headon_swerve_angular': 1.0,
            'headon_forward_velocity': 1.0,
            'headon_horizon': 1.2,
            'headon_side_lock_time': 2.0,
            'headon_side_lock_bonus': 0.25,
            'headon_hard_lock': True,
            'headon_min_feasible_steps': 2,
            'cmd_latch_time': 1.9,
        }],
    )

    path_recorder = Node(
        package='mrvk_gazebo', executable='path_recorder',
        name='path_recorder', output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_topic': odom_topic,
            'path_topic': '/traveled_path',
            'frame_id': 'odom',
            'min_distance': 0.05,
            'max_points': 0,
            'publish_rate': 5.0,
        }],
    )

    rqt_steering = Node(
        package='rqt_robot_steering', executable='rqt_robot_steering',
        name='rqt_robot_steering', output='screen',
        parameters=[{'default_topic': cmd_topic}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('planner', default_value='teb',
                              description='Local planner: dwb | teb | tr'),
        DeclareLaunchArgument('safety_filter', default_value='true',
                              description='Run dynamic obstacle predictor + motion '
                                          'safety filter (false = planner drives '
                                          'the robot directly)'),
        DeclareLaunchArgument('odom_topic', default_value='/odom',
                              description='FAST-LIO odometry topic (real hardware; '
                                          'sim used /shoddy/odom instead)'),
        hardware,
        nav2,
        ground_segmentation,
        dynamic_obstacle_predictor,
        robot_motion_predictor,
        path_recorder,
        rqt_steering,
    ])
