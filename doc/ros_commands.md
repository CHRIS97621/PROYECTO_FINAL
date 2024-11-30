### Creacion de paquetes 

```bash
ros2 pkg create agro_description --build-type ament_cmake --dependencies rclcpp std_msgs
ros2 pkg create agro_gazebo --build-type ament_cmake --dependencies rclcpp std_msgs
```



## Install gazebo

```bash
sudo apt-get install curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

### Instalar extensiones
ROS
URDF


#### CONVERTIR DE xacro a urdf
```bash
xacro agro.urdf.xacro >> agro.urdf
# xacro path_file_xacro.xacro >> path_destinacion.urdf
```