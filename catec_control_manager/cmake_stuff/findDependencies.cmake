##---------------------------------------------------------------------------------------------------------------------
##  LUCAS: Lightweight framework for UAV Control And Supervision
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2024 CATEC (Advanced Centre for Aerospace Technologies)
##---------------------------------------------------------------------------------------------------------------------
## This program is free software: you may redistribute it and/or modify it under the terms of the GNU General Public
## License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
## version.
## This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
## warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
## You should have received a copy of the GNU General Public License along with this program. If not, see
## https://www.gnu.org/licenses/.
##---------------------------------------------------------------------------------------------------------------------

# ########################
# # Configuring catkin  ##
# ########################
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(catec_control_manager_msgs REQUIRED)
find_package(cascade_pid_controller_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

set(ROS2_DEPENDENCIES
  rclcpp
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2
  std_msgs
  nav_msgs
  std_srvs
  geometry_msgs
  mavros_msgs
  catec_control_manager_msgs
  cascade_pid_controller_msgs
)
ament_target_dependencies(${PROJECT_LIB_NAME} ${ROS2_DEPENDENCIES})

########################
## Configuring spdlog ##
########################
find_package(spdlog REQUIRED)
target_link_libraries(${PROJECT_LIB_NAME} spdlog::spdlog)

##########################
## Configuring yaml-cpp ##
##########################
find_package(yaml-cpp REQUIRED)
target_link_libraries(${PROJECT_LIB_NAME} yaml-cpp)

########################
## Configuring Eigen  ##
########################
find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_LIB_NAME} Eigen3::Eigen)
