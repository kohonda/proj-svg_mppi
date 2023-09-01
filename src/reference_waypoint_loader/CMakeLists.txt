cmake_minimum_required(VERSION 3.13)
project(reference_waypoint_loader)

add_compile_options(-std=c++17)
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

#### Options
option(BUILD_TESTS "Build test" OFF)
option(BUILD_WITH_MARCH_NATIVE "Build with -march=native" OFF)

find_package(catkin REQUIRED COMPONENTS 
  tf2_ros 
  roslib 
  rosbag
  waypoint_msgs
  nav_msgs
  visualization_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES reference_waypoint_loader
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_executable(reference_waypoint_loader_node
  src/reference_waypoint_loader.cpp
  src/reference_waypoint_loader_node.cpp
)

add_dependencies(reference_waypoint_loader_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_include_directories(reference_waypoint_loader_node PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(reference_waypoint_loader_node
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    reference_waypoint_loader_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)