# # Find includes in corresponding build directories
if(NOT DEFINED CMAKE_PREFIX_PATH)
   if(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
      if(NOT WIN32)
         STRING(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      else()
         set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
      endif()
   endif()
endif()

if(NOT "/usr/local" IN_LIST CMAKE_INSTALL_PREFIX)
   LIST(APPEND CMAKE_INSTALL_PREFIX "/usr/local")
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_INSTALL_PREFIX}/lib/cmake/)

message(STATUS "Using CMAKE_PREFIX_PATH - ${CMAKE_PREFIX_PATH}")

# ########################
# # Configuring catkin  ##
# ########################
find_package(catkin REQUIRED
   COMPONENTS roscpp control_toolbox std_msgs geometry_msgs nav_msgs cascade_pid_controller_msgs 
   tf2 tf2_ros tf2_geometry_msgs tf_conversions catec_control_manager_msgs tf2_eigen)

catkin_package(
   INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp control_toolbox std_msgs geometry_msgs nav_msgs cascade_pid_controller_msgs
   tf2 tf2_ros tf2_geometry_msgs tf_conversions catec_control_manager_msgs tf2_eigen
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})

# To ensure that the messages are built before the node
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})