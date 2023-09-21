// Kohei Honda, 2023

#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <deque>
#include <iostream>
#include <limits>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mppi_metrics_msgs/MPPIMetrics.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include "mppi_controller/StopWatch.hpp"
#include "mppi_controller/common.hpp"
#include "mppi_controller/forward_mppi.hpp"
#include "mppi_controller/reverse_mppi.hpp"
#include "mppi_controller/stein_variational_guided_mppi.hpp"
#include "mppi_controller/stein_variational_mpc.hpp"

namespace mppi {
    class MPPIControllerROS {
    public:
        MPPIControllerROS();
        ~MPPIControllerROS(){};

    private:
        struct RobotState {
            double x = 0.0;
            double y = 0.0;
            double yaw = 0.0;
            double vel = 0.0;
            double steer = 0.0;
        };

    private:
        std::mutex mtx_;

        /* ros system variables */
        ros::NodeHandle nh_;          //!< @brief ros public node handle
        ros::NodeHandle private_nh_;  //!< @brief ros private node handle

        std::string robot_frame_id_;
        std::string map_frame_id_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        /* pub sub */
        ros::Subscriber sub_reference_sdf_;   //!< @brief reference sdf subscriber
        ros::Subscriber sub_odom_;            //!< @brief robot odom subscriber
        ros::Subscriber sub_occupancy_grid_;  //!< @brief costmap subscriber (nav_msgs::OccupancyGrid for costmap_2d)
        ros::Subscriber sub_grid_map_;        //!< @brief grid map subscriber (grid_map_msgs::GridMap for local costmap)
        ros::Timer timer_control_;            //!< @brief timer for control command commutation
        ros::Publisher pub_ackermann_cmd_;
        ros::Subscriber sub_activated_;
        ros::Subscriber sub_backward_point_;

        ros::Subscriber sub_start_cmd_;
        ros::Subscriber sub_stop_cmd_;

        // For debug
        StopWatch stop_watch_;                             //!< @brief stop watch for calculation time
        ros::Publisher pub_best_path_;                     //!< @brief best path topic publisher
        ros::Publisher pub_nominal_path_;                  //!< @brief nominal path topic publisher
        ros::Publisher pub_candidate_paths_;               //!< @brief candidate paths topic publisher
        ros::Publisher pub_proposal_state_distributions_;  //!< @brief proposal state distribution topic publisher
        ros::Publisher pub_control_covariances_;           //!< @brief control covariance topic publisher
        ros::Publisher pub_calculation_time_;              //!< @brief calculation time topic publisher
        ros::Publisher pub_speed_;                         //!< @brief robot speed topic publisher
        ros::Publisher pub_collision_rate_;                //!< @brief collision rate topic publisher
        ros::Publisher pub_cost_;                          //!< @brief cost topic publisher
        ros::Publisher pub_mppi_metrics_;                  //!< @brief mppi metrics topic publisher

        /*control system parametes*/
        double control_sampling_time_;  //!< @brief control interval [s]
        bool is_localize_less_mode_ = false;
        bool constant_speed_mode_ = false;
        bool is_visualize_mppi_ = false;
        bool is_start_ = true;
        const std::string obstacle_layer_name_ = "collision_layer";
        const std::string distance_field_layer_name_ = "distance_field";
        const std::string angle_field_layer_name_ = "angle_field";
        const std::string speed_field_layer_name_ = "speed_field";

        /* Variables */
        RobotState robot_state_;
        double reference_speed_ = 0.0;
        double collision_rate_threshold_ = 0.95;  // [0, 1] If collision rate is over this value, stop robot
        grid_map::GridMap obstacle_map_;          // value = [0, 100], 100: collision, 0: free (obstacle layer)
        grid_map::GridMap reference_sdf_;
        ackermann_msgs::AckermannDriveStamped control_msg_;

        double steer_1st_delay_ = 0.1;

        // For stuck detection
        std::deque<float> speed_deque_;
        int speed_deque_size_ = 10;
        float stuck_speed_threshold_ = 0.1;

        bool is_robot_state_ok_ = false;
        bool is_reference_sdf_ok_ = false;
        bool is_costmap_ok_ = false;
        bool is_activate_ad_ = false;
        bool is_simulation_ = false;

        std::unique_ptr<mppi::cpu::MPCTemplate> mpc_solver_ptr_;

        /**
         * @brief Main loop
         *
         */
        void timer_callback(const ros::TimerEvent&);

        void start_cmd_callback(const std_msgs::Empty& msg);

        void stop_cmd_callback(const std_msgs::Empty& msg);

        void callback_odom(const nav_msgs::Odometry& odom);

        void callback_odom_with_pose(const nav_msgs::Odometry& odom);

        void callback_reference_sdf(const grid_map_msgs::GridMap& grid_map);

        void callback_grid_map(const grid_map_msgs::GridMap& grid_map);

        void callback_activate_signal(const std_msgs::Bool& is_activate);

        void publish_candidate_paths(const std::vector<mppi::cpu::StateSeq>& state_seq_batch,
                                     const std::vector<double>& weights,
                                     const ros::Publisher& publisher) const;

        void publish_traj(const mppi::cpu::StateSeq& state_seq,
                          const std::string& name_space,
                          const std::string& rgb,
                          const ros::Publisher& publisher) const;

        void publish_path(const mppi::cpu::StateSeq& state_seq,
                          const std::string& name_space,
                          const std::string& rgb,
                          const ros::Publisher& publisher) const;

        void publish_control_covs(const mppi::cpu::StateSeq& mean,
                                  const mppi::cpu::ControlSeqCovMatrices& cov_matrices,
                                  const ros::Publisher& publisher) const;

        void publish_state_seq_dists(const mppi::cpu::StateSeq& state_seq,
                                     const mppi::cpu::XYCovMatrices& cov_matrices,
                                     const ros::Publisher& publisher) const;
    };

}  // namespace mppi
