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

# Find includes in corresponding build directories
if(NOT DEFINED CMAKE_PREFIX_PATH)
   if(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
      set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
   endif()
endif()

if(NOT CMAKE_INSTALL_PREFIX STREQUAL "/usr/local")
   set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/lib/cmake/)

message(STATUS "Using CMAKE_PREFIX_PATH - ${CMAKE_PREFIX_PATH}")

# ########################
# # Configuring catkin  ##
# ########################
find_package(catkin REQUIRED
   COMPONENTS roscpp mavros_msgs catec_control_manager_msgs cascade_pid_controller_msgs)

catkin_package(
   INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp mavros_msgs catec_control_manager_msgs cascade_pid_controller_msgs
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})

# To ensure that the messages are built before the node
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

########################
## Configuring spdlog ##
########################
find_package(spdlog REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC spdlog::spdlog)

##########################
## Configuring yaml-cpp ##
##########################
find_package(yaml-cpp REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC yaml-cpp)

########################
## Configuring Eigen  ##
########################
find_package(Eigen3 REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
