# Navigation RB1

## Install
### Install dependencies

```bash
rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```


## Sim Robot
### Mapping

Terminal 1
```bash
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=True
```
Terminal 2

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### Launch only localization with Rviz
```bash
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_sim.yaml use_sim_time:=True rviz:=True
```
### Launch only PathPlanner 
```bash
ros2 launch path_planner_server path_planner_server.launch.py type_simulation:=sim_robot use_sim_time:=True 
```

### Launch entire navigation
```bash
ros2 launch path_planner_server navigation.launch.py type_simulation:=sim_robot use_sim_time:=True map_file:=warehouse_map_sim_edit.yaml 
```

## Real Robot
### Mapping
Terminal 1
```bash
ros2 launch cartographer_slam cartographer.launch.py use_sim_time:=False
```
Terminal 2

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Launch only localization 
```bash
ros2 launch localization_server localization.launch.py map_file:=warehouse_map_real.yaml use_sim_time:=False
```
### Launch only PathPlanner 

```bash
ros2 launch path_planner_server navigation.launch.py type_simulation:=real_robot use_sim_time:=False 
```
### Launch entire navigation
```bash
ros2 launch path_planner_server navigation.launch.py type_simulation:=real_robot use_sim_time:=false map_file:=warehouse_map_real.yaml 
```