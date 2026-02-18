# kanga_sim

ROS 2 package for simulating and visualizing the Kanga robot.

## What this package provides

- Gazebo spawn launch for `kanga_robot.urdf`
- RViz2 launch for model visualization
- URDF, wheel/contact tuning, and Gazebo diff-drive plugins

## Launch files

### Spawn robot in Gazebo

```bash
ros2 launch kanga_sim spawn_robot.launch.py
```

Optional spawn args:

```bash
ros2 launch kanga_sim spawn_robot.launch.py x:=0.0 y:=0.0 z:=0.0 entity:=kanga_robot
```

### View robot in RViz2

```bash
ros2 launch kanga_sim view_robot.launch.py use_sim_time:=true
```

Optional RViz config override:

```bash
ros2 launch kanga_sim view_robot.launch.py rviz_config:=/absolute/path/to/config.rviz
```

## Build

```bash
cd ~/gazebo_ws
colcon build --packages-select kanga_sim
source install/setup.bash
```
