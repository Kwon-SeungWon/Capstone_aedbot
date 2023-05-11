################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(aedbot_navigation2)

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(aedbot_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
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

# set(DEPENDENCIES
#   "geometry_msgs"
#   "message_filters"
#   "nav_msgs"
#   "rclcpp"
#   "rcutils"
#   "sensor_msgs"
#   "std_msgs"
#   "std_srvs"
#   "tf2"
#   "tf2_ros"
#   "aedbot_interfaces"
#   "builtin_interfaces"
#)

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
  DIRECTORY launch map param rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
#ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  #scripts/run_inspection.py
  #scripts/lift_controller.py
  #scripts/nav_to_pose.py
  #scripts/nav_through_poses.py
  #scripts/pick_and_deliver.py
  scripts/robot_navigator.py
  scripts/go_to_goal.py
  #scripts/security_demo.py
  #scripts/waypoint_follower.py
  #scripts/battery_state/battery_state_pub.py
  #scripts/navigate_to_charging_dock.py
  #scripts/navigate_to_charging_dock_v2.py
  #scripts/transforms/map_to_base_link_transform.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Macro for ament package
################################################################################
ament_package()
