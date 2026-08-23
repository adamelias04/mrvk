# MRVK — ROS2 (Humble) port

ROS2 port of the "shoddy" mobile-robot simulation with dynamic-obstacle
prediction and a motion-safety filter. Original project was ROS1 (catkin,
move_base, gmapping, Gazebo Classic). This port targets **ROS2 Humble**,
**Gazebo Classic** (gazebo_ros_pkgs), **Nav2** and **slam_toolbox**.

## Packages

| Package | What it is |
|---|---|
| `mrvk_description` | URDF/xacro robot model + Gazebo (ROS2) sensor/drive plugins |
| `mrvk_gazebo_plugins` | Gazebo Classic plugins: oscillating-box dynamic obstacle, path-trail visual |
| `mrvk_gazebo` | C++ nodes, Nav2 costmap layer, Nav2/SLAM config, worlds, launch, maps, metrics |
| `detected_path_layer` | Nav2 costmap layer marking non-walkable camera cells |
| `path_detection` | `DetectedPath` msg + camera path detector (rclpy) |
| `imu_quat_to_eul` | `EulRot` msg + quaternion→Euler node (rclpy) |
| `simple_filter` | naive IMU+odom complementary filter (rclpy) |
| `gazebo_nav_goal_gui` | Gazebo Classic GUI plugin: publish 2D goal to `/goal_pose` |

## Key ROS1 → ROS2 mapping

- `move_base` → **Nav2** (controller/planner/behavior/bt_navigator + lifecycle manager)
- `gmapping` → **slam_toolbox** (async)
- `dwa_local_planner` → **DWB** (`nav2_params_dwb.yaml`)
- `teb_local_planner` → **teb_local_planner ROS2** (`nav2_params_teb.yaml`, built from source)
- `base_local_planner/TrajectoryPlannerROS` → **DWB (trajectory-rollout tuning)** (`nav2_params_tr.yaml`)
- costmap layers → **nav2_costmap_2d::Layer** API
- `roscpp`/`rospy` → `rclcpp`/`rclpy`; `catkin` → `ament_cmake`/`ament_python`
- gazebo skid-steer/imu/gpu_laser/camera plugins → gazebo_ros ROS2 equivalents

Navigation runs in the `odom` frame (rolling-window costmaps, no static map),
matching the original. slam_toolbox provides `map`/`map→odom` for visualization.

## Build & run

```sh
# 1) system deps (needs sudo)
./install_deps.sh

# 2) (optional) TEB planner from source — only needed for planner:=teb
./src/teb_from_source.sh

# 3) build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 4) run (default world test_static.world, planner DWB)
ros2 launch mrvk_gazebo robot.launch.py

# choose world / planner:
ros2 launch mrvk_gazebo robot.launch.py world_file:=$(ros2 pkg prefix mrvk_gazebo)/share/mrvk_gazebo/worlds/empty.world planner:=teb
```

Set a goal with RViz2's **2D Goal Pose** tool (publishes `/goal_pose`), or the
Gazebo `2D Nav Goal` form panel. Experiment drivers:

```sh
ros2 run mrvk_gazebo waypoint_tour
ros2 run mrvk_gazebo goal_pingpong
```

Worlds with a **dynamic obstacle** (oscillating box): `empty.world`.
Static-obstacle benchmark: `test_static.world`. Corridor: `corridor.world`.
