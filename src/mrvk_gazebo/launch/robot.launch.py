# ROS2 top-level launch (port of robot.launch). Brings up Gazebo + the robot,
# slam_toolbox, Nav2 (selectable planner), the dynamic obstacle predictor, the
# robot motion safety filter, path recorder and rqt_robot_steering.
#
# Usage:
#   ros2 launch mrvk_gazebo robot.launch.py
#   ros2 launch mrvk_gazebo robot.launch.py planner:=teb world_file:=<...>.world
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

    # Default empty.world = dynamicky svet s oscilujucim boxom (pouziva sa so
    # safety_filter:=true, co je tiez default). Pre staticku navigaciu spusti
    # s world_file:=<...>/test_static.world  safety_filter:=false.
    default_world = os.path.join(pkg, 'worlds', 'empty.world')
    slam_params = os.path.join(pkg, 'config', 'slam', 'slam_toolbox.yaml')

    world_file = LaunchConfiguration('world_file')
    planner = LaunchConfiguration('planner')
    laser_topic = LaunchConfiguration('laser_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    safety_filter = LaunchConfiguration('safety_filter')

    # S filtrom: planner -> /cmd_vel_raw -> robot_motion_predictor -> robot.
    # Bez filtra (safety_filter:=false): planner ide priamo na robot.
    cmd_topic = PythonExpression(
        ["'/cmd_vel_raw' if '", safety_filter, "' == 'true' else '/shoddy/cmd_vel'"])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world_file': world_file,
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'use_nvidia': LaunchConfiguration('use_nvidia'),
            'init_x': LaunchConfiguration('init_x'),
            'init_y': LaunchConfiguration('init_y'),
            'init_z': LaunchConfiguration('init_z'),
            'init_yaw': LaunchConfiguration('init_yaw'),
        }.items(),
    )

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'nav2.launch.py')),
        launch_arguments={'planner': planner, 'cmd_vel_out': cmd_topic}.items(),
    )

    dynamic_obstacle_predictor = Node(
        package='mrvk_gazebo', executable='dynamic_obstacle_predictor',
        name='dynamic_obstacle_predictor', output='screen',
        condition=IfCondition(safety_filter),
        parameters=[{
            'use_sim_time': True,
            'scan_topic': laser_topic,
            'output_frame': 'odom',
            'prediction_time': 4.0,
            'prediction_dt': 0.1,
            'alpha_gain': 0.65,
            'beta_gain': 0.20,
            'max_speed': 2.0,
            'track_timeout': 1.2,
            'lost_track_prediction_time': 1.6,
            'prediction_hold_time': 0.8,
            'prediction_hold_speed_epsilon': 0.08,
            'min_dynamic_speed': 0.3,
            'min_dynamic_persist': 0.35,
            'point_spacing': 0.05,
            'corridor_radius': 0.18,
            'object_half_extent': 0.18,
            'z_height': 0.10,
            'scan_range_min_use': 0.10,
            'scan_range_max_use': 8.00,
            'segment_jump_distance': 0.30,
            'segment_min_points': 2,
            'segment_max_points': 120,
            'segment_min_width': 0.03,
            'segment_max_width': 2.00,
            'use_angle_roi': False,
            'angle_min_roi': -1.57,
            'angle_max_roi': 1.57,
            'use_range_roi': False,
            'range_min_roi': 0.20,
            'range_max_roi': 8.00,
            'initial_expected_angle': 0.0,
            'initial_expected_range': 2.5,
            'max_association_distance': 2.0,
            'publish_debug_segment': True,
        }],
    )

    robot_motion_predictor = Node(
        package='mrvk_gazebo', executable='robot_motion_predictor',
        name='robot_motion_predictor', output='screen',
        condition=IfCondition(safety_filter),
        parameters=[{
            'use_sim_time': True,
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
            'use_sim_time': True,
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
        DeclareLaunchArgument('world_file', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_nvidia', default_value='true',
                              description='Render Gazebo/RViz on NVIDIA GPU (PRIME offload)'),
        DeclareLaunchArgument('planner', default_value='teb',
                              description='Local planner: dwb | teb | tr'),
        # Default true — dynamicka navigacia: bezi dynamic_obstacle_predictor +
        # robot_motion_predictor (bezpecnostny filter). Pre cistu staticku
        # navigaciu spusti s safety_filter:=false (planner riadi robota priamo).
        DeclareLaunchArgument('safety_filter', default_value='true',
                              description='Run dynamic obstacle predictor + motion '
                                          'safety filter (false = planner drives '
                                          'the robot directly; useful for static '
                                          'benchmark worlds)'),
        DeclareLaunchArgument('laser_topic', default_value='/scan'),
        DeclareLaunchArgument('odom_topic', default_value='/shoddy/odom'),
        DeclareLaunchArgument('init_x', default_value='3.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_z', default_value='0.3'),
        DeclareLaunchArgument('init_yaw', default_value='3.14'),
        gazebo,
        slam,
        nav2,
        dynamic_obstacle_predictor,
        robot_motion_predictor,
        path_recorder,
        rqt_steering,
    ])
