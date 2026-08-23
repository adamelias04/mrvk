# Nav2 bringup for MRVK. Starts controller/planner/behavior/bt_navigator/
# waypoint_follower with the selected planner param file and a lifecycle
# manager. The controller output is remapped cmd_vel -> /cmd_vel_raw so the
# robot_motion_predictor safety filter can post-process it into /shoddy/cmd_vel.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('mrvk_gazebo')
    planner = LaunchConfiguration('planner').perform(context)
    # Kam ide vystup plannera: /cmd_vel_raw (cez bezpecnostny filter, default)
    # alebo priamo /shoddy/cmd_vel ked filter nebezi (safety_filter:=false).
    cmd_vel_out = LaunchConfiguration('cmd_vel_out').perform(context)

    planner_files = {
        'dwb': 'nav2_params_dwb.yaml',
        'teb': 'nav2_params_teb.yaml',
        'tr':  'nav2_params_tr.yaml',
    }
    params_file = os.path.join(pkg, 'config', 'nav2',
                               planner_files.get(planner, 'nav2_params_dwb.yaml'))

    use_sim_time = {'use_sim_time': True}

    controller = Node(
        package='nav2_controller', executable='controller_server', output='screen',
        parameters=[params_file, use_sim_time],
        remappings=[('cmd_vel', cmd_vel_out)],
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', parameters=[params_file, use_sim_time],
    )
    behavior = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server',
        output='screen', parameters=[params_file, use_sim_time],
        # Recovery spravanie (spin/backup) musi ist tou istou cestou ako
        # planner, inak by jeho cmd_vel nikto neodoberal.
        remappings=[('cmd_vel', cmd_vel_out)],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen', parameters=[params_file, use_sim_time],
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen',
        parameters=[params_file, use_sim_time],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
        }],
    )

    return [controller, planner_server, behavior, bt_navigator,
            waypoint_follower, lifecycle_manager]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='teb',
                              description='Local planner: dwb | teb | tr'),
        DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel_raw',
                              description='Controller/behavior cmd_vel output topic'),
        OpaqueFunction(function=launch_setup),
    ])
