# MRVK ROS Workspace

## Install

```sh
$ cd ${ROS_WS}
$ rosdep install --from-paths src --ignore-src -r -y
$ catkin_make
$ source ./devel/setup.bash
$ export GAZEBO_MODEL_DATABASE_URI=http://models.gazebosim.org
```

## Launch simulation

In file [robot.launch](./mrvk_gazebo/launch/robot.launch) choose the world to be launched (uncomment or rewrite) and set initial coordinates.

```sh
$ roslaunch mrvk_gazebo robot.launch
```

### Select world

In file [gazebo.launch](./mrvk_gazebo/launch/gazebo.launch) set the `world_file` argument to one of the worlds in [mrvk_gazebo/worlds](./mrvk_gazebo/worlds):

* `empty.world` — prázdna scéna
* `test_static.world` — statické prekážky (benchmark statickej navigácie)
* `corridor.world` — úzka chodba pre head-on / crossing scenár

```xml
<arg name="world_file" default="$(find mrvk_gazebo)/worlds/test_static.world"/>
```


### Launch Path Detection

* launch `path_detection_node`
```sh
$ rosrun path_detection path_detection_node.py
```
* set navigation goal by publishing on `/move_base/goal` topic

### Switch local planner

In file [move_base.yaml](./mrvk_gazebo/config/navigation/move_base.yaml) uncomment one of:

* `base_local_planner: teb_local_planner/TebLocalPlannerROS`
* `base_local_planner: dwa_local_planner/DWAPlannerROS`
* `base_local_planner: base_local_planner/TrajectoryPlannerROS`

Then restart `robot.launch`.

### Run static waypoint tour

Sends the robot through a fixed sequence of waypoints (default for static benchmark).

```sh
$ rosrun mrvk_gazebo waypoint_tour
```

Override defaults with private params (`_wpN_x`, `_wpN_y`, `_wpN_yaw`, `_repeats`, `_pause_min`, `_pause_max`, `_seed`).





