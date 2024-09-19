cmake_minimum_required(VERSION 3.0.2)
project(rclpy_param_helper)

# Utilizza catkin per la gestione delle dipendenze di ROS 1
find_package(catkin REQUIRED COMPONENTS
  rospy
)

# Trova pacchetti Python
find_package(Python3 REQUIRED COMPONENTS NumPy)

# Includi le directory di include di catkin
catkin_package(
  CATKIN_DEPENDS rospy
)

# Includi il setup Python per il pacchetto
catkin_python_setup()

# Installa gli script Python
install(DIRECTORY rclpy_param_helper/
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}/rclpy_param_helper
)

# Installa altri file, come package.xml
install(FILES
  package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)