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
project(catec_control_manager_msgs)

# ---- Configure CATKIN dependecies ----
find_package(catkin REQUIRED 
   COMPONENTS
      message_generation
      std_msgs
      geometry_msgs
)

# ---- Declare ROS messages, services and actions ----
add_service_files(DIRECTORY srv)
add_message_files(DIRECTORY msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# ---- CATKIN specific configuration ----
catkin_package(
   CATKIN_DEPENDS message_runtime
)