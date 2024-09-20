cmake_minimum_required(VERSION 3.8)
project(quad_sim_python_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

# Aggiungi i file di messaggi da generare
add_message_files(
  FILES
  QuadMotors.msg
  QuadWind.msg
  QuadControlSetPoint.msg
  QuadState.msg
)

# Genera messaggi
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Definisci il pacchetto
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

# Includi le directory
include_directories(
  ${catkin_INCLUDE_DIRS}
)
