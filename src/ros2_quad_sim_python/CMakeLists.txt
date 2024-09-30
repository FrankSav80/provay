cmake_minimum_required(VERSION 3.0.2)
project(ros2_quad_sim_python)

## Trova catkin e qualsiasi altro pacchetto necessario
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  quad_sim_python_msgs
)

## Dichiarazione delle dipendenze di runtime
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs quad_sim_python_msgs
)

## Installazione degli script Python
catkin_install_python(PROGRAMS
  ros2_quad_sim_python/ros_quad_ctrl.py
  ros2_quad_sim_python/ros_quad_sim.py
  ros2_quad_sim_python/ros_quad_sim_and_ctrl.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
