# stomp_moveit

> Note: This package is WIP and not feature complete!

STOMP Planner plugin for MoveIt ROS 2.

## Build

Clone this package and stomp into `src` directory of your workspace `COLCON_WS` and install dependencies by running:

```
cd $COLCON_WS/src
git clone https://github.com/henningkayser/stomp_moveit
vcs import < stomp_moveit/stomp_moveit.repos
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO
```

Compile your and source yourworkspace

```
cd $COLCON_WS
colcon build
source install/setup.bash
```

## Setup

Configure a STOMP planning pipeline by adding a pipeline entry "stomp" to your MoveIt config [here](https://github.com/ros-planning/moveit_resources/blob/ros2/panda_moveit_config/launch/demo.launch.py#L28).
Add a `stomp_planning.yaml` to your config path with the following content:

```
planning_plugin: stomp_moveit/StompPlanner
request_adapters: >-
  default_planner_request_adapters/AddTimeOptimalParameterization
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStateCollision
  default_planner_request_adapters/FixStartStatePathConstraints
```

With this you should now be able to select STOMP as your planning pipeline using the RViZ MotionPlanningPannel or your API of choice.
