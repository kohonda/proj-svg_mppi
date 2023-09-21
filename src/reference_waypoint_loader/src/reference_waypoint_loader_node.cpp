#include "reference_waypoint_loader/reference_waypoint_loader.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "reference_waypoint_loader");
    planning::ReferenceWaypointLoader reference_waypoint_loader;
    ros::spin();
    return 0;
}