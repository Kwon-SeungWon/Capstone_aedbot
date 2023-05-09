################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(aedbot_navigation2)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aedbot_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

####################
###### BUILD #######
####################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#rosidl_generate_interfaces(${PROJECT_NAME}
#  "srv/FallDetectionToNav2.srv"
#)

set(DEPENDENCIES
  "geometry_msgs"
  "message_filters"
  "nav_msgs"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "std_msgs"
  "std_srvs"
  "tf2"
  "tf2_ros"
  "aedbot_interfaces"
  "builtin_interfaces"
)

####################
### ADD C++ FILE ###
####################
#target_include_directories(program PUBLIC ${CMAKE_SOURCE_DIR}/includes)

#add_executable(to_goal src/go_to_goal.cpp)

#target_include_directories(to_goal PUBLIC ${CMAKE_SOURCE_DIR}/includes)
#target_link_libraries(to_goal ${catkin_LIBRARIES})

#ament_target_dependencies(to_goal
#  ${DEPENDENCIES} aedbot_interfaces)   

################################################################################
# Install
################################################################################
install(
  DIRECTORY launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

#install(TARGETS
#  to_goal
#  DESTINATION share/${PROJECT_NAME}
#)

################################################################################
# Macro for ament package
################################################################################
ament_package()
