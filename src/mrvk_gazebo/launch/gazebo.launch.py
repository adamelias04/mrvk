# ROS2 port of gazebo.launch: starts Gazebo Classic with a world, spawns the
# "shoddy" robot from xacro, and runs robot_state_publisher + RViz.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, AppendEnvironmentVariable,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('mrvk_gazebo')
    pkg_desc = get_package_share_directory('mrvk_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    default_model = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')
    default_world = os.path.join(pkg_gazebo, 'worlds', 'test_static.world')
    default_rviz = os.path.join(pkg_gazebo, 'config', 'rviz', 'config.rviz')

    # Gazebo Classic system resources (shaders/materials/models/plugins).
    # Without /usr/share/gazebo-11 on GAZEBO_RESOURCE_PATH the GUI (gzclient)
    # fails to initialize the OGRE scene and crashes. Normally set by sourcing
    # /usr/share/gazebo/setup.sh; we ensure it here so the launch is turnkey.
    gz_prefix = '/usr/share/gazebo-11'
    gz_plugins = '/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'

    model = LaunchConfiguration('model')
    world_file = LaunchConfiguration('world_file')
    gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')
    use_nvidia = LaunchConfiguration('use_nvidia')
    init_x = LaunchConfiguration('init_x')
    init_y = LaunchConfiguration('init_y')
    init_z = LaunchConfiguration('init_z')
    init_yaw = LaunchConfiguration('init_yaw')

    robot_description = {
        'robot_description': Command(['xacro ', model]),
        'use_sim_time': True,
    }

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
    )

    # GUI pluginy (point-and-click "2D Nav Goal" + X/Y/Yaw panel) sa nacitavaju
    # cez --gui-client-plugin, takze funguju pre kazdy svet (ako v ROS1).
    # gzclient spustame priamo (nie cez gzclient.launch.py): jeho
    # extra_gazebo_args sa neseka na tokeny, takze viacero flagov by doslo
    # ako jeden argv retazec a pluginy by sa nenacitali. Nutna je aj forma
    # s '=' — s oddelenym tokenom ('--gui-client-plugin NAME') gzclient
    # instancuje plugin pred vytvorenim QApplication a spadne.
    gzclient = ExecuteProcess(
        cmd=[
            'gzclient',
            '--gui-client-plugin=libgazebo_goal_tool.so',
            '--gui-client-plugin=libgazebo_nav_goal_gui.so',
        ],
        output='screen',
        condition=IfCondition(gui),
    )

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
        parameters=[{'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robot',
            '-x', init_x, '-y', init_y, '-z', init_z, '-Y', init_yaw,
        ],
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

    return LaunchDescription([
        AppendEnvironmentVariable('GAZEBO_RESOURCE_PATH', gz_prefix),
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.join(gz_prefix, 'models')),
        AppendEnvironmentVariable('GAZEBO_PLUGIN_PATH', gz_plugins),
        # PRIME render offload: render Gazebo + RViz on the discrete NVIDIA GPU
        # instead of the Intel iGPU (faster; avoids the Mesa/Intel RViz Map
        # GLSL shader-link warning). Set use_nvidia:=false on non-NVIDIA hosts.
        DeclareLaunchArgument('use_nvidia', default_value='true',
                              description='Render on the NVIDIA GPU via PRIME offload'),
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1',
                               condition=IfCondition(use_nvidia)),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia',
                               condition=IfCondition(use_nvidia)),
        DeclareLaunchArgument('model', default_value=default_model),
        DeclareLaunchArgument('world_file', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('init_x', default_value='3.0'),
        DeclareLaunchArgument('init_y', default_value='0.0'),
        DeclareLaunchArgument('init_z', default_value='0.3'),
        DeclareLaunchArgument('init_yaw', default_value='3.14'),
        gzserver,
        gzclient,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        rviz,
    ])
