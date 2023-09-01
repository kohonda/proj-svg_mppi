cmake_minimum_required(VERSION 3.13)
project(local_costmap_generator)

add_compile_options(-std=c++17)
add_compile_options(-Wall -Wextra)
add_compile_options(-O3)
add_compile_options(-O3 -fopenmp)
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  nav_msgs
  roscpp
  tf2_geometry_msgs
  tf2_ros
  grid_map_core
  grid_map_ros
  grid_map_filters
  grid_map_loader
  grid_map_pcl
  grid_map_msgs
  grid_map_rviz_plugin
  grid_map_visualization
  laser_geometry
  sensor_msgs
  voxel_grid
  pcl_ros
  pcl_conversions
)

find_package(Eigen3 REQUIRED)

# For OpenMP
# set(OpenMP_HOME "/usr/lib/llvm-10")
# set(OpenMP_omp_LIBRARY "${OpenMP_HOME}/lib/")
# set(OpenMP_C_FLAGS "-fopenmp -I${OpenMP_HOME}/include/openmp -lomp -L${OpenMP_omp_LIBRARY}" CACHE STRING "" FORCE)
# set(OpenMP_CXX_FLAGS "-fopenmp -I${OpenMP_HOME}/include/openmp -lomp -L${OpenMP_omp_LIBRARY}" CACHE STRING "" FORCE)
# set(OpenMP_C_LIB_NAMES "omp")
# set(OpenMP_CXX_LIB_NAMES "omp")
find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES mppi
#  CATKIN_DEPENDS ackermann_msgs geometry_msgs nav_msgs roscpp std_msgs tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########
add_executable(${PROJECT_NAME}_node
  src/local_costmap_generator_node.cpp
  src/local_costmap_generator.cpp
)

## Specify additional locations of header files
## Your package locations should be listed before other locations

target_include_directories(${PROJECT_NAME}_node PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
 )

if (OPENMP_FOUND)
    if (TARGET OpenMP::OpenMP_CXX)
        target_link_libraries(${PROJECT_NAME}_node OpenMP::OpenMP_CXX)
    endif ()
endif ()


 install(
  TARGETS
    ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)