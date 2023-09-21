#pragma once

#include <algorithm>
#include <filesystem>
#include <vector>

#include <ros/console.h>
#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <waypoint_msgs/Waypoint.h>  // custom msg

#include "reference_waypoint_loader/rapidcsv.h"
#include "reference_waypoint_loader/tinycolormap.hpp"

namespace planning {

class ReferenceWaypointLoader {
public:
    ReferenceWaypointLoader();
    ~ReferenceWaypointLoader(){};

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    /* publisher */
    ros::Publisher reference_waypoint_pub_;
    ros::Publisher reference_path_pub_;
    ros::Publisher reference_rvizmarker_pub_;

    /* system params */
    std::string csv_waypoint_;
    std::string map_frame_;
    std::string topic_name_waypoint_;
    std::string topic_name_path_;
    std::string topic_name_rviz_waypoint_marker_;
    std::string ref_x_label_;
    std::string ref_y_label_;
    std::string ref_v_label_;
    float ref_v_scale_;
    float scaled_v_min_;
    float scaled_v_max_;
};

}  // namespace planning