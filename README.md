### Simulación

### Install dependencies

```bash
colcon build --symlink-install
source install/setup.bash
```

```bash
cd ~/project_ws
sudo apt update
rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y
```
### Lanzar simualción y ekf

```bash
source install/setup.bash
ros2 launch agro_gazebo sim.launch.py
```

### Lanzar navegación

```bash
ros2 launch path_planner_server navigation.launch.py
```
### Lanzar mapviz

```bash
ros2 launch agro_mapviz mapviz.launch..py
```
### Lanzar mapviz

```bash
ros2 run agro_gazebo remaping_mapviz_rviz.py
```


```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p use_sim_time:=true --remap /cmd_vel:=/agro_base_controller/cmd_vel
```

ros2 run tf2_ros static_transform_publisher 1 2 3 0.5 0.1 -1.0 odom base_link

