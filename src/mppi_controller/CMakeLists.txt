cmake_minimum_required(VERSION 3.13)
project(mppi_controller)

option(BUILD_WITH_GPU "Build with GPU support" OFF)

add_compile_options(-std=c++17)
add_compile_options(-Wall -Wextra)
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
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  grid_map_core
  grid_map_ros
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_rviz_plugin
  grid_map_visualization
  mppi_metrics_msgs
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


if(BUILD_WITH_GPU)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  link_directories(${CUDA_LIBRARY_DIRS})
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

#### Library (CPU) ####
add_library(mppi_solver SHARED
  src/prior_samples_with_costs.cpp
  src/mpc_base.cpp
  src/forward_mppi.cpp
  src/reverse_mppi.cpp
  src/stein_variational_mpc.cpp
  src/stein_variational_guided_mppi.cpp
)
target_link_libraries(mppi_solver
  ${catkin_LIBRARIES}
)
if (OPENMP_FOUND)
    if (TARGET OpenMP::OpenMP_CXX)
        target_link_libraries(mppi_solver OpenMP::OpenMP_CXX)
    endif ()
endif ()

target_include_directories(mppi_solver PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

#### App ####
add_executable(${PROJECT_NAME}_node 
  src/mppi_controller_node.cpp
  src/mppi_controller_ros.cpp
  )
target_include_directories(${PROJECT_NAME}_node PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(${PROJECT_NAME}_node PRIVATE
  ${catkin_LIBRARIES}
  mppi_solver
)


### CUDA ###
if(BUILD_WITH_GPU)
  set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")
  add_definitions(-DUSE_CUDA)

  cuda_add_library(mppi_solver_gpu SHARED
    src/mppi_gpu.cu
  )

  target_include_directories(mppi_solver_gpu PRIVATE
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )

  target_link_libraries(mppi_solver_gpu 
    ${catkin_LIBRARIES}
    ${CUDA_LIBRARIES}
    ${CUDA_curand_LIBRARY}
  )
  cuda_add_cublas_to_target(mppi_solver_gpu)

  # add mppi_solver_gpu to mppi_solver
  target_link_libraries(mppi_solver
    mppi_solver_gpu
  )
  add_dependencies(mppi_solver mppi_solver_gpu)
endif()


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