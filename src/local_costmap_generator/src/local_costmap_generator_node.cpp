#include "local_costmap_generator/local_costmap_generator.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "local_costmap_generator");
    local_costmap_generator::LocalCostmapGenerator local_costmap_generator;
    ros::spin();
    return 0;
};
