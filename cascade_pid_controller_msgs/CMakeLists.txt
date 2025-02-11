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

cmake_minimum_required(VERSION 3.10...3.22)
project(cascade_pid_controller_msgs)

# ---- Configure CATKIN dependecies ----
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- Declare ROS messages, services and actions ----

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrajCommand.msg"
  "msg/State.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()