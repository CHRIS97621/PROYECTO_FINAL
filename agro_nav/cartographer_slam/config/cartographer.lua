include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  --Frame que será pubicado por el SLAM
  map_frame = "map",
  --Frame que será trackeado
  tracking_frame = "base_footprint",
  --Frame hijo que será utilzado para publicar la transformación a /map
  published_frame = "odom",
  --Frame de odometría corregido generado por SLAM, esta odometía es más precisa
  --tiene un nivel de actualización mucho más rápida, solo se proveerá si 
  -- "provide_odom_frame" es igual a true
  odom_frame = "odom",
  provide_odom_frame = false,
  --De todas las transformaciones serán filtrados los componentes 3d
  --rotaciones en x y z y altura en z
  publish_frame_projected_to_2d = true,
  --El SLAM se subscribirá o no al tópico de odometría
  use_odometry = true,
  --Define si se utilizarán datos de gps, el tópico por defecto es /fix
  use_nav_sat = false,
  --Usar identificadores únicos para mejorar en el tema de la localización
  use_landmarks = false,
  -- Subscribirse al tópico de scan para localizar al robot
  -- ~~~~~~~~~~~~~~~~~~~~~~~Scaners~~~~~~~~~~~~~~~~~~~~~~~~~~
  --use_laser_scans = true,
  -- Cantidad de scanes utilizados en total
  num_laser_scans = 1,
  -- Número de scaneres del tipo multi echo, estos haz de laser pueden rebotar
  -- en la superficioe como vidrio, de esta manera puedo tener mas información
  num_multi_echo_laser_scans = 0,
  -- Dividir la data entregada por el laser en tramas, estas tramas se
  --analizaran independientemente
  num_subdivisions_per_laser_scan = 1,
  -- Número de sensoros en 3d a utilizar
  num_point_clouds = 0,
  -- ~~~~~~~~~~~~~~~~~~~~~~~Transfor ~~~~~~~~~~~~~~~~~~~~~~~~~~
  lookup_transform_timeout_sec = 0.2,
  -- frecuencia de publicación de los subs mapas por el local SLAM
  submap_publish_period_sec = 0.3,
  -- Frecuencia de la publicación de la posición del robot 200 hz
  pose_publish_period_sec = 5e-3,
  -- Frecuencia de la publicación de la trayectoria del robot
  trajectory_publish_period_sec = 30e-3,
  -- Porcentraje de la data del sensor lidar a utilizar
  rangefinder_sampling_ratio = 1.,
  -- Porcentraje de la data de odometría a utilizar
  odometry_sampling_ratio = 1.,
  -- Porcentraje de la data de marco fijo, en este caso robot_baser_footprint
  fixed_frame_pose_sampling_ratio = 1.,
  -- Porcentraje de la data de imu a utilizar
  imu_sampling_ratio = 1.,
  -- Porcentraje de la data de landmarks a utilizar
  landmarks_sampling_ratio = 0.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
--Lectura mínima que se tomará en cuenta para evitar lecturas con ruido
TRAJECTORY_BUILDER_2D.min_range = 0.10
--Lectura máxima para no tomar en cuenta distancia muy extensas 
TRAJECTORY_BUILDER_2D.max_range = 6
--
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 4.0
--Ayudan a estimar la posición del robot cuando no se reciban datos de odotmetía o laser(*************)
TRAJECTORY_BUILDER_2D.use_imu_data = false
--el algoritmo de Cartographer utilizará el scan matching correlativo en línea para proporcionar una "suposición inicial"
-- precisa de la posición del robot.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
--Angulo mázimo que el robot puede girar mientras se le aplique el filtro que elimina las nuevas lecturas
--del laser 
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
--Parámetros para enlazar los submapas generados
POSE_GRAPH.constraint_builder.min_score = 0.65
--Parámetro para enlazar dos submapas que pertenecen a la misma área en cuestion
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- POSE_GRAPH.optimize_every_n_nodes = 0

return options