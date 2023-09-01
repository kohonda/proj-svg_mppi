cmake_minimum_required(VERSION 3.0.2)
project(eval_local_planner)

find_package(catkin REQUIRED COMPONENTS 
    message_generation 
    rospy 
    std_msgs
    mppi_metrics_msgs
    )

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RaceInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs
)

catkin_install_python(PROGRAMS 
    src/eval.py 
    DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)