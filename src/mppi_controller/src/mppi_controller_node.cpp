#include "mppi_controller/mppi_controller_ros.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mppi_controller");
    mppi::MPPIControllerROS mppi_controller;
    ros::spin();
    return 0;
};
