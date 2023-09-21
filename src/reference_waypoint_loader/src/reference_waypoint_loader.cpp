#include "reference_waypoint_loader/reference_waypoint_loader.hpp"

namespace planning {

ReferenceWaypointLoader::ReferenceWaypointLoader() : nh_(), private_nh_("~") {
    private_nh_.param<std::string>("reference_waypoint_csv", csv_waypoint_, "");
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("reference_waypoint_topic", topic_name_waypoint_, "reference_waypoint");
    private_nh_.param<std::string>("reference_path_topic", topic_name_path_, "reference_path");
    private_nh_.param<std::string>("reference_rviz_marker_topic", topic_name_rviz_waypoint_marker_, "rviz_waypoint_marker");
    private_nh_.param<std::string>("reference_waypoint_x_column_label", ref_x_label_, "opt_x");
    private_nh_.param<std::string>("reference_waypoint_y_column_label", ref_y_label_, "opt_y");
    private_nh_.param<std::string>("reference_waypoint_v_column_label", ref_v_label_, "ref_v");
    private_nh_.param<float>("reference_velocity_scale", ref_v_scale_, 1.0);
    private_nh_.param<float>("scaled_velocity_min", scaled_v_min_, 0.0);
    private_nh_.param<float>("scaled_velocity_max", scaled_v_max_, 100.0);

    if (!std::filesystem::exists(csv_waypoint_)) {
        ROS_ERROR("reference path file does not exist: %s", csv_waypoint_.c_str());
        exit(1);
    }

    // load csv file
    const rapidcsv::Document doc(csv_waypoint_);

    // get csv data
    const std::vector<double> x = doc.GetColumn<double>(ref_x_label_);
    const std::vector<double> y = doc.GetColumn<double>(ref_y_label_);
    const std::vector<double> v = doc.GetColumn<double>(ref_v_label_);
    const double ref_v_max = *max_element(v.begin(), v.end());  // max reference velocity
    const double ref_v_min = *min_element(v.begin(), v.end());  // min reference velocity

    // publish as waypoint_msgs::Waypoint (= List of "Pose + Twist")
    waypoint_msgs::Waypoint reference_waypoint;

    // publish path as well for visualization with nav_msgs::Path (= List of "Pose")
    nav_msgs::Path reference_path;

    // set waypoint header
    reference_waypoint.header.frame_id = map_frame_;
    reference_waypoint.header.stamp = ros::Time::now();

    // set path header
    reference_path.header.frame_id = map_frame_;
    reference_path.header.stamp = ros::Time::now();

    // declare marker array
    visualization_msgs::MarkerArray polygon_markers;
    polygon_markers.markers.clear();

    // prepare marker template
    int marker_id = 0;
    visualization_msgs::Marker marker_template;
    marker_template.header.frame_id = map_frame_;
    marker_template.header.stamp = ros::Time();
    marker_template.ns = "waypoints";
    marker_template.id = marker_id;
    marker_template.type = visualization_msgs::Marker::LINE_STRIP;
    marker_template.action = visualization_msgs::Marker::ADD;
    marker_template.pose.position.x = 0.0;
    marker_template.pose.position.y = 0.0;
    marker_template.pose.position.z = 0.01;
    marker_template.pose.orientation.x = 0.0;
    marker_template.pose.orientation.y = 0.0;
    marker_template.pose.orientation.z = 0.0;
    marker_template.pose.orientation.w = 1.0;
    marker_template.scale.x = 0.1;
    marker_template.scale.y = 0.1;
    marker_template.scale.z = 0.1;
    marker_template.color.a = 0.5;
    marker_template.color.r = 1.0;
    marker_template.color.g = 0.0;
    marker_template.color.b = 0.0;

    // initialize buffer to keep previous marker position
    geometry_msgs::Point prev_marker_point;
    prev_marker_point.x = x[0];
    prev_marker_point.y = y[0];
    prev_marker_point.z = 0.0;

    for (int i = 0; i < x.size(); i++) {
        // add reference pose info
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x[i];
        pose.pose.position.y = y[i];
        pose.pose.position.z = 0.0;
        reference_waypoint.poses.push_back(pose);
        reference_path.poses.push_back(pose);

        // add reference velocity info
        geometry_msgs::TwistStamped twist;
        float scaled_v = ref_v_scale_ * v[i];
        scaled_v = std::clamp(scaled_v, scaled_v_min_, scaled_v_max_);
        twist.twist.linear.x = v[i];
        twist.twist.linear.y = 0.0;
        twist.twist.linear.z = 0.0;
        reference_waypoint.twists.push_back(twist);

        // add line
        int skip_num = 3;
        if (i > 0 && i % skip_num == 1) {  // skip the first point && i % skip_num == 1)

            // decide marker color
            std_msgs::ColorRGBA current_marker_color;

            // simple normalization of the current velocity
            double normalized_v = (v[i] - ref_v_min) / std::max((ref_v_max - ref_v_min), 0.1);  // 0 <= normalized_v <= 1
            tinycolormap::Color colormap_val = tinycolormap::GetColor(normalized_v, tinycolormap::ColormapType::Heat);

            current_marker_color.r = colormap_val.r();
            current_marker_color.g = colormap_val.g();
            current_marker_color.b = colormap_val.b();
            current_marker_color.a = 1.0;

            // set 2 positions to draw line
            visualization_msgs::Marker current_marker = marker_template;
            current_marker.id = marker_id;  // set an unique id
            current_marker.points.push_back(prev_marker_point);
            current_marker.colors.push_back(current_marker_color);
            geometry_msgs::Point current_marker_point;
            current_marker_point.x = x[i];
            current_marker_point.y = y[i];
            current_marker_point.z = 0.0;
            prev_marker_point = current_marker_point;
            current_marker.points.push_back(current_marker_point);
            current_marker.colors.push_back(current_marker_color);

            // add current marker to markers
            polygon_markers.markers.push_back(current_marker);
            marker_id++;
        }
    }

    // publish waypoint msg
    reference_waypoint_pub_ = nh_.advertise<waypoint_msgs::Waypoint>(topic_name_waypoint_, 1, true);
    reference_waypoint_pub_.publish(reference_waypoint);

    // publish path msg
    reference_path_pub_ = nh_.advertise<nav_msgs::Path>(topic_name_path_, 1, true);
    reference_path_pub_.publish(reference_path);

    // publish visualization msg
    reference_rvizmarker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_name_rviz_waypoint_marker_, 1, true);
    reference_rvizmarker_pub_.publish(polygon_markers);
}

}  // namespace planning