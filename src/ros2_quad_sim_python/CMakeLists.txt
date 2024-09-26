cmake_minimum_required(VERSION 3.0.2)
project(ros2_quad_sim_python)

## Trova catkin e qualsiasi altro pacchetto necessario
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

## Dichiarazione delle dipendenze di runtime
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs
)

# Includi il setup Python per il pacchetto
catkin_python_setup()

# Installa la cartella ros2_quad_sim_python nella directory dist-packages
install(DIRECTORY
  src/ros2_quad_sim_python/ros2_quad_sim_python/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/ros2_quad_sim_python
  FILES_MATCHING PATTERN "*.py"  # Se vuoi installare solo i file .py
)

## Installazione dei file di configurazione e di lancio
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resource
)

## Installazione dei test (se necessario)
install(DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
