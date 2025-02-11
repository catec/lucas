# ########################
# # Configuring catkin  ##
# ########################
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cascade_pid_controller_msgs REQUIRED)
find_package(catec_control_manager_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(ROS2_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  mavros_msgs
  cascade_pid_controller_msgs
  catec_control_manager_msgs
  control_toolbox
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

ament_target_dependencies(${PROJECT_LIB_NAME} ${ROS2_DEPENDENCIES})

########################
## Configuring Eigen  ##
########################
find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_LIB_NAME} Eigen3::Eigen)