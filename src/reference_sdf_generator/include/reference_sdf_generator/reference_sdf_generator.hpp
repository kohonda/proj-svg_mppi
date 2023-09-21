// Kohei Honda, 2023

#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

// custom message
#include <waypoint_msgs/Waypoint.h>

namespace reference_sdf_generator {

class ReferenceSDFGenerator {
public:
    ReferenceSDFGenerator();
    ~ReferenceSDFGenerator(){};

private:
    struct RobotState {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;

        // use only waypoints
        double vel = 0.0;
    };
    using Waypoints = std::vector<RobotState>;

private:
    /* ros system variables */
    ros::NodeHandle nh_;          //!< @brief ros public node handle
    ros::NodeHandle private_nh_;  //!< @brief ros private node handle
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    /* pub sub */
    ros::Subscriber sub_ref_path_;       //!< @brief reference path subscriber
    ros::Timer timer_control_;           //!< @brief timer for control command commutation
    ros::Publisher pub_reference_sdf_;   //!< @brief distance field topic publisher
    ros::Publisher pub_backward_point_;  //!< @brief backward point topic publisher for switchback

    // debug
    ros::Publisher pub_waypoints_;  //!< @brief waypoints topic publisher (for debug)
    // ros::Publisher pub_calc_time_;

    /*Parameters*/
    std::string robot_frame_id_;
    std::string map_frame_id_;
    double update_rate_;  //!< @brief update interval [s]
    int thread_num_ = 4;
    const std::string distance_field_layer_name_ = "distance_field";
    const std::string angle_field_layer_name_ = "angle_field";
    const std::string speed_field_layer_name_ = "speed_field";

    // waypoints for path tracking parameters
    int backward_margin_num_ = 5;
    int num_waypoints_ = 30;
    double waypoint_interval_ = 0.1;        //!< @brief interval between waypoints [m]
    double ref_path_map_resolution_ = 0.1;  //!< @brief resolution of distance field [m]
    double ref_path_map_width_ = 50.0;      //!< @brief width of distance field [m]
    double ref_path_map_height_ = 50.0;     //!< @brief height of distance field [m]
    double reference_speed_scale_ = 1.0;    //!< @brief scale of reference speed
    double max_speed_ = 100.0;              //!< @brief maximum speed [m/s]
    // The submap size is important for calculation cost
    // double submap_center_ahead_ = 3.0; //!< @brief distance from robot to submap center [m]
    // double submap_length_ = 15.0;      //!< @brief width of submap [m]
    // double submap_width_ = 15.0;       //!< @brief height of submap [m]

    // Inner variables
    waypoint_msgs::Waypoint waypoints_msg_;

    bool is_waypoints_ok_ = false;
    grid_map::GridMap reference_sdf_;  // value = [0, inf) (distance field layer), value = [-pi/2, pi/2] (angle layer)

    /**
     * @brief main loop
     *
     */
    void timer_callback(const ros::TimerEvent&);

    void callback_waypoints(const waypoint_msgs::Waypoint& waypoint_msg);

    template <typename T1, typename T2>
    double distance(T1 pt1, T2 pt2) const {
        return sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
    }

    int find_nearest_index(const std::vector<geometry_msgs::PoseStamped>& path, const RobotState& pose) const;

    int find_lookahead_index(const std::vector<geometry_msgs::PoseStamped>& path, const int& nearest_index, const double& lookahead_dist) const;

    Waypoints calc_waypoints(const waypoint_msgs::Waypoint& waypoints, const RobotState& current_state) const;

    void build_reference_sdf(const Waypoints& waypoints, const RobotState& current_state, grid_map::GridMap* distance_field) const;

    void publish_waypoints(const Waypoints& waypoints) const;
};

}  // namespace reference_sdf_generator
