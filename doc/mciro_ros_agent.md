## Preparando el agente de ROS

# Crear un nuevo espacio de trabajo
```bash
mkdir ~/microros_ws
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```

# Update dependencies using rosdep
```bash
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
```

# Install pip
```bash
sudo apt-get install python3-pip
```

# Build micro-ROS tools and source them
```bash
colcon build
source install/local_setup.bash
```


## Crear y compilar micro-ros agente

```bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```