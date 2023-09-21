#include "reference_sdf_generator/reference_sdf_generator.hpp"

namespace reference_sdf_generator {

ReferenceSDFGenerator::ReferenceSDFGenerator()
    : nh_(""),
      private_nh_("~"),
      tf_listener_(tf_buffer_),
      reference_sdf_(std::vector<std::string>({distance_field_layer_name_, angle_field_layer_name_, speed_field_layer_name_})) {
    // set parameters from ros parameter server
    private_nh_.param("update_rate", update_rate_, 0.05);
    private_nh_.param("thread_num", thread_num_, 4);
    private_nh_.param("num_waypoints", num_waypoints_, 30);
    private_nh_.param("backward_margin_num", backward_margin_num_, 5);
    private_nh_.param("waypoint_interval", waypoint_interval_, 0.1);
    private_nh_.param("ref_path_map_resolution", ref_path_map_resolution_, 0.1);
    private_nh_.param("ref_path_map_width", ref_path_map_width_, 50.0);
    private_nh_.param("ref_path_map_height", ref_path_map_height_, 50.0);
    private_nh_.param("max_speed", max_speed_, 100.0);
    private_nh_.param("reference_speed_scale", reference_speed_scale_, 1.0);
    // private_nh_.param("submap_center_ahead", submap_center_ahead_, 3.0);
    // private_nh_.param("submap_length", submap_length_, 15.0);
    // private_nh_.param("submap_width", submap_width_, 15.0);

    std::string out_sdf_topic;
    std::string in_waypoints_topic;
    std::string out_backward_point_topic;

    private_nh_.param("in_waypoints_topic", in_waypoints_topic, static_cast<std::string>("reference_waypoint"));
    private_nh_.param("robot_frame_id", robot_frame_id_, static_cast<std::string>("base_link"));
    private_nh_.param("map_frame_id", map_frame_id_, static_cast<std::string>("map"));
    private_nh_.param("out_sdf_topic", out_sdf_topic, static_cast<std::string>("reference_sdf"));
    private_nh_.param("backward_point_topic", out_backward_point_topic, static_cast<std::string>("backward_point"));

    // initialize reference path grid map
    reference_sdf_.setFrameId(map_frame_id_);
    reference_sdf_.setGeometry(grid_map::Length(ref_path_map_width_, ref_path_map_height_), ref_path_map_resolution_, grid_map::Position(0.0, 0.0));

    // set publishers and subscribers
    sub_ref_path_ = nh_.subscribe(in_waypoints_topic, 1, &ReferenceSDFGenerator::callback_waypoints, this);
    timer_control_ = nh_.createTimer(ros::Duration(update_rate_), &ReferenceSDFGenerator::timer_callback, this);
    pub_reference_sdf_ = nh_.advertise<grid_map_msgs::GridMap>(out_sdf_topic, 1, true);
    pub_backward_point_ = nh_.advertise<geometry_msgs::PointStamped>(out_backward_point_topic, 1, true);

    // For debug
    pub_waypoints_ = nh_.advertise<visualization_msgs::MarkerArray>("reference_sdf/waypoints", 1, true);
    // pub_calc_time_ = nh_.advertise<std_msgs::Float32>("reference_sdf/calc_time", 1, true);
}

void ReferenceSDFGenerator::callback_waypoints(const waypoint_msgs::Waypoint& waypoints_msg) {
    if (waypoints_msg.poses.size() == 0) {
        ROS_WARN_THROTTLE(5.0, "[ReferenceSDFGenerator]Received waypoints is empty");
        waypoints_msg_.poses.clear();
        waypoints_msg_.twists.clear();
        return;
    } else {
        ROS_INFO("[ReferenceSDFGenerator]New waypoints is received.");
    }
    waypoints_msg_ = waypoints_msg;
    is_waypoints_ok_ = true;
}

// main loop
void ReferenceSDFGenerator::timer_callback([[maybe_unused]] const ros::TimerEvent& te) {
    /* Status check */
    if (!is_waypoints_ok_) {
        ROS_WARN_THROTTLE(5.0, "[ReferenceSDFGenerator] path is not ready");
        return;
    }

    // get current robot state
    geometry_msgs::TransformStamped trans_form_stamped;
    try {
        trans_form_stamped = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(3.0, "[ReferenceSDFGenerator] %s", ex.what());
        return;
    };
    RobotState robot_state;
    robot_state.x = trans_form_stamped.transform.translation.x;
    robot_state.y = trans_form_stamped.transform.translation.y;
    const double _yaw = tf2::getYaw(trans_form_stamped.transform.rotation);
    robot_state.yaw = std::atan2(std::sin(_yaw), std::cos(_yaw));

    // Calculate waypoints
    const auto waypoints = calc_waypoints(waypoints_msg_, robot_state);

    // ======== time measurement ========
    // const auto start_time = std::chrono::system_clock::now();

    // Calculate reference SDF
    build_reference_sdf(waypoints, robot_state, &reference_sdf_);

    // ======== time measurement ========
    // const auto end_time = std::chrono::system_clock::now();
    // const auto elapsed_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000000.0;
    // std_msgs::Float32 calc_time_msg;
    // calc_time_msg.data = elapsed_time;
    // pub_calc_time_.publish(calc_time_msg);

    // Publish reference SDF
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(reference_sdf_, message);
    message.info.header.stamp = ros::Time::now();
    pub_reference_sdf_.publish(message);

    // Publish backward point
    geometry_msgs::PointStamped backward_point_msg;
    backward_point_msg.header.frame_id = map_frame_id_;
    backward_point_msg.header.stamp = ros::Time::now();
    backward_point_msg.point.x = waypoints[0].x;
    backward_point_msg.point.y = waypoints[0].y;
    backward_point_msg.point.z = 0;
    pub_backward_point_.publish(backward_point_msg);

    // Publish waypoints for debug
    publish_waypoints(waypoints);
}

int ReferenceSDFGenerator::find_nearest_index(const std::vector<geometry_msgs::PoseStamped>& path, const RobotState& current_state) const {
    int nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    geometry_msgs::Point pos;
    for (size_t i = 0; i < path.size(); i++) {
        const double dist = distance(path[i].pose.position, current_state);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

int ReferenceSDFGenerator::find_lookahead_index(const std::vector<geometry_msgs::PoseStamped>& path,
                                                const int& nearest_index,
                                                const double& lookahead_dist) const {
    // Find target waypoint index with loop path
    int target_waypoint_idx_ = nearest_index;
    bool is_found_target_waypoint_ = false;
    int index = nearest_index;
    while (!is_found_target_waypoint_) {
        index = (index + 1) % path.size();
        const double dist = distance(path[index].pose.position, path[nearest_index].pose.position);
        if (dist >= lookahead_dist) {
            target_waypoint_idx_ = index;
            is_found_target_waypoint_ = true;
        }
    }

    return target_waypoint_idx_;
}

ReferenceSDFGenerator::Waypoints ReferenceSDFGenerator::calc_waypoints(const waypoint_msgs::Waypoint& waypoints_msg,
                                                                       const RobotState& current_state) const {
    Waypoints waypoints;

    auto get_waypoint = [](const waypoint_msgs::Waypoint& waypoints_msgs, const int& index) {
        RobotState waypoint;
        waypoint.x = waypoints_msgs.poses[index].pose.position.x;
        waypoint.y = waypoints_msgs.poses[index].pose.position.y;

        // calculate yaw angle
        int next_index = 0;
        if (index == static_cast<int>(waypoints_msgs.poses.size()) - 1) {
            next_index = index - 1;
            waypoint.yaw = std::atan2(waypoints_msgs.poses[index].pose.position.y - waypoints_msgs.poses[next_index].pose.position.y,
                                      waypoints_msgs.poses[index].pose.position.x - waypoints_msgs.poses[next_index].pose.position.x);
        } else {
            next_index = index + 1;
            waypoint.yaw = std::atan2(waypoints_msgs.poses[next_index].pose.position.y - waypoints_msgs.poses[index].pose.position.y,
                                      waypoints_msgs.poses[next_index].pose.position.x - waypoints_msgs.poses[index].pose.position.x);
        }

        // vel
        waypoint.vel = waypoints_msgs.twists[index].twist.linear.x;

        return waypoint;
    };

    // find nearest point
    const int nearest_idx = find_nearest_index(waypoints_msg.poses, current_state);

    const int nearest_idx_with_margin = std::max(nearest_idx - backward_margin_num_, 0);
    const auto waypoint = get_waypoint(waypoints_msg, nearest_idx_with_margin);
    waypoints.push_back(waypoint);

    // find waypoints with lookahead distance
    int current_waypoint_idx = nearest_idx_with_margin;
    for (int i = 1; i < num_waypoints_; i++) {
        const int target_waypoint_idx = find_lookahead_index(waypoints_msg.poses, current_waypoint_idx, waypoint_interval_);

        const auto waypoint = get_waypoint(waypoints_msg, target_waypoint_idx);
        waypoints.push_back(waypoint);
        current_waypoint_idx = target_waypoint_idx;
    }

    return waypoints;
}

void ReferenceSDFGenerator::build_reference_sdf(const Waypoints& waypoints, const RobotState& current_state, grid_map::GridMap* reference_sdf) const {
    reference_sdf->setGeometry(grid_map::Length(ref_path_map_width_, ref_path_map_height_), ref_path_map_resolution_,
                               grid_map::Position(current_state.x, current_state.y));

    // Ellipse submap setting
    // const double ahead_distance = submap_center_ahead_;
    // const grid_map::Position ellipse_center(current_state.x + ahead_distance * cos(current_state.yaw), current_state.y + ahead_distance *
    // sin(current_state.yaw)); const grid_map::Length length(submap_length_, submap_width_);

    // iterate all cells for calculating distance field
    grid_map::Matrix& dist_data = reference_sdf->get(distance_field_layer_name_);
    grid_map::Matrix& angle_data = reference_sdf->get(angle_field_layer_name_);
    grid_map::Matrix& speed_data = reference_sdf->get(speed_field_layer_name_);

    const auto grid_map_size = reference_sdf->getSize();
    const unsigned int linear_grid_size = grid_map_size.prod();

    // The following are provided normal iterators, but cannot used open mp
    // for (grid_map::EllipseIterator iterator(*reference_sdf, ellipse_center, length, current_state.yaw); !iterator.isPastEnd(); ++iterator)
    // for (grid_map::GridMapIterator iterator(*reference_sdf); !iterator.isPastEnd(); ++iterator)

#pragma omp parallel for num_threads(thread_num_)
    for (unsigned int i = 0; i < linear_grid_size; ++i) {
        // const grid_map::Index index(*iterator);
        const grid_map::Index index(grid_map::getIndexFromLinearIndex(i, grid_map_size));

        grid_map::Position position;
        reference_sdf->getPosition(index, position);

        // calc distance to nearest waypoint
        double min_dist = std::numeric_limits<double>::max();
        int nearest_waypoint_idx = 0;
        for (size_t i = 0; i < waypoints.size(); i++) {
            const double dist = sqrt((waypoints[i].x - position[0]) * (waypoints[i].x - position[0]) +
                                     (waypoints[i].y - position[1]) * (waypoints[i].y - position[1]));
            if (dist < min_dist) {
                min_dist = dist;
                nearest_waypoint_idx = i;
            }
        }

        // set each field value
        dist_data(index[0], index[1]) = min_dist;
        angle_data(index[0], index[1]) = waypoints[nearest_waypoint_idx].yaw;
        speed_data(index[0], index[1]) = std::min(waypoints[nearest_waypoint_idx].vel * reference_speed_scale_, max_speed_);
    }
}

void ReferenceSDFGenerator::publish_waypoints(const Waypoints& waypoints) const {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = map_frame_id_;
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "waypoints";
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Vector3 arrow_scale;
    arrow_scale.x = 0.02;
    arrow_scale.y = 0.04;
    arrow_scale.z = 0.1;
    arrow.scale = arrow_scale;
    arrow.pose.position.x = 0.0;
    arrow.pose.position.y = 0.0;
    arrow.pose.position.z = 0.0;
    arrow.pose.orientation.x = 0.0;
    arrow.pose.orientation.y = 0.0;
    arrow.pose.orientation.z = 0.0;
    arrow.pose.orientation.w = 1.0;
    arrow.color.a = 1.0;
    arrow.color.r = 0.0;
    arrow.color.g = 1.0;
    arrow.color.b = 0.0;
    arrow.lifetime = ros::Duration(0.1);
    arrow.points.resize(2);

    for (size_t i = 0; i < waypoints.size(); i++) {
        arrow.id = i;
        const double length = 0.3;
        geometry_msgs::Point start;
        start.x = waypoints[i].x;
        start.y = waypoints[i].y;
        start.z = 0.1;

        geometry_msgs::Point end;
        end.x = waypoints[i].x + length * cos(waypoints[i].yaw);
        end.y = waypoints[i].y + length * sin(waypoints[i].yaw);
        end.z = 0.1;

        arrow.points[0] = start;
        arrow.points[1] = end;

        marker_array.markers.push_back(arrow);
    }

    pub_waypoints_.publish(marker_array);
}

}  // namespace reference_sdf_generator