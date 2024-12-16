# Mapear

## Crear un mapa
```bash
ros2 launch cartographer_slam cartographer.launch.py 
```
## Desplazarte por el entorno
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -p use_sim_time:=true --remap /cmd_vel:=/agro_base_controller/cmd_vel
```
### guardar mapa

```bash
ros2 run nav2_map_server map_saver_cli -f map_1
```

# Map server
```bash
ros2 launch map_server map_server.launch.py
```

# Path planner