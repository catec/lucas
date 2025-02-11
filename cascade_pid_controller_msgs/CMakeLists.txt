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

project(cascade_pid_controller_msgs)
cmake_minimum_required(VERSION 3.5.1)

##################################
## Configure CATKIN dependecies ##
##################################

find_package(catkin REQUIRED COMPONENTS
   message_generation
   std_msgs
   geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(DIRECTORY msg)
# add_service_files(DIRECTORY srv)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## CATKIN specific configuration ##
###################################

catkin_package(
   CATKIN_DEPENDS message_runtime
)