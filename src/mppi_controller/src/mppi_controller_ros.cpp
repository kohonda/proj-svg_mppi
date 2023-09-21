#include "mppi_controller/mppi_controller_ros.hpp"

namespace mppi {
MPPIControllerROS::MPPIControllerROS() : nh_(""), private_nh_("~"), tf_listener_(tf_buffer_) {
    // set parameters from ros parameter server
    private_nh_.param("is_simulation", is_simulation_, false);
    private_nh_.param("is_localize_less_mode", is_localize_less_mode_, false);

    private_nh_.param("control_sampling_time", control_sampling_time_, 0.1);

    private_nh_.param("is_visualize_mppi", is_visualize_mppi_, false);
    private_nh_.param("constant_speed_mode", constant_speed_mode_, false);

    std::string control_cmd_topic;
    std::string in_reference_sdf_topic;
    std::string in_odom_topic;
    std::string is_activate_ad_topic;
    std::string occupacy_grid_id;
    std::string local_costmap_id;
    std::string in_backward_point_topic;

    private_nh_.param("control_cmd_topic", control_cmd_topic, static_cast<std::string>("drive"));
    private_nh_.param("in_reference_sdf_topic", in_reference_sdf_topic, static_cast<std::string>("reference_sdf"));
    private_nh_.param("in_odom_topic", in_odom_topic, static_cast<std::string>("odom"));
    private_nh_.param("robot_frame_id", robot_frame_id_, static_cast<std::string>("base_link"));
    private_nh_.param("map_frame_id", map_frame_id_, static_cast<std::string>("map"));
    private_nh_.param("costmap_id", occupacy_grid_id, static_cast<std::string>("costmap"));
    private_nh_.param("local_costmap_id", local_costmap_id, static_cast<std::string>("local_costmap"));
    private_nh_.param("backward_point_topic", in_backward_point_topic, static_cast<std::string>("backward_point"));

    private_nh_.param("is_activate_ad_topic", is_activate_ad_topic, static_cast<std::string>("is_active_ad"));
    private_nh_.param("collision_rate_threshold", collision_rate_threshold_, 0.95);
    private_nh_.param("speed_queue_size", speed_deque_size_, static_cast<int>(10));
    private_nh_.param("stuck_speed_threshold", stuck_speed_threshold_, static_cast<float>(0.1));

    private_nh_.param("steer_1st_delay", steer_1st_delay_, static_cast<double>(0.1));

    // fill deque
    for (int i = 0; i < speed_deque_size_; i++) {
        const double enough_speed = stuck_speed_threshold_ + 1.0;
        speed_deque_.push_back(enough_speed);
    }

    // load params
    Params params;
    // Common params for all MPC methods
    params.common.is_localize_less_mode = is_localize_less_mode_;
    params.common.steer_1st_delay = steer_1st_delay_;

    private_nh_.param("common/thread_num", params.common.thread_num, 12);
    private_nh_.param("reference_speed", params.common.reference_speed, 5.0);
    reference_speed_ = params.common.reference_speed;
    private_nh_.param("common/prediction_step_size", params.common.prediction_step_size, 10);
    private_nh_.param("common/prediction_interval", params.common.prediction_interval, 0.1);

    steer_1st_delay_ = std::max(steer_1st_delay_, params.common.prediction_interval);

    private_nh_.param("common/steer_delay", params.common.steer_delay, 0.1);
    private_nh_.param("common/max_steer_angle", params.common.max_steer_angle, 3.0);
    private_nh_.param("common/min_steer_angle", params.common.min_steer_angle, -3.0);
    private_nh_.param("common/max_accel", params.common.max_accel, 2.0);
    private_nh_.param("common/min_accel", params.common.min_accel, -2.0);
    private_nh_.param("common/speed_prediction_mode", params.common.speed_prediction_mode, static_cast<std::string>("linear"));
    private_nh_.param("common/lr", params.common.lr, 0.17145);
    private_nh_.param("common/lf", params.common.lf, 0.15875);
    private_nh_.param("common/q_dist", params.common.q_dist, 1.0);
    private_nh_.param("common/q_angle", params.common.q_angle, 0.0);
    // private_nh_.param("common/q_speed", params.q_speed, 10.0);
    private_nh_.param("common/collision_weight", params.common.collision_weight, 0.01);
    private_nh_.param("common/q_terminal_dist", params.common.q_terminal_dist, 1.0);
    private_nh_.param("common/q_terminal_angle", params.common.q_terminal_angle, 0.0);
    // private_nh_.param("common/q_terminal_speed",
    // params.q_terminal_speed, 10.0);

    // Forward MPPI params
    private_nh_.param("forward_mppi/sample_batch_num", params.forward_mppi.sample_batch_num, 1000);
    private_nh_.param("forward_mppi/lambda", params.forward_mppi.lambda, 10.0);
    private_nh_.param("forward_mppi/alpha", params.forward_mppi.alpha, 0.1);
    private_nh_.param("forward_mppi/non_biased_sampling_rate", params.forward_mppi.non_biased_sampling_rate, 0.1);
    private_nh_.param("forward_mppi/steer_cov", params.forward_mppi.steer_cov, 0.5);
    // private_nh_.param("forward_mppi/accel_cov", params.accel_cov, 0.5);
    private_nh_.param("forward_mppi/num_itr_for_grad_estimation", params.forward_mppi.num_itr_for_grad_estimation, 1);
    private_nh_.param("forward_mppi/step_size_for_grad_estimation", params.forward_mppi.step_size_for_grad_estimation, 0.1);
    private_nh_.param("forward_mppi/sample_num_for_grad_estimation", params.forward_mppi.sample_num_for_grad_estimation, 10);
    private_nh_.param("forward_mppi/steer_cov_for_grad_estimation", params.forward_mppi.steer_cov_for_grad_estimation, 0.05);

    // Reverse MPPI params
    private_nh_.param("reverse_mppi/sample_batch_num", params.reverse_mppi.sample_batch_num, 1000);
    private_nh_.param("reverse_mppi/negative_ratio", params.reverse_mppi.negative_ratio, 1.0);
    private_nh_.param("reverse_mppi/is_sample_rejection", params.reverse_mppi.is_sample_rejection, true);
    private_nh_.param("reverse_mppi/sample_inflation_ratio", params.reverse_mppi.sample_inflation_ratio, 2.0);
    private_nh_.param("reverse_mppi/iteration_num", params.reverse_mppi.iteration_num, 5);
    private_nh_.param("reverse_mppi/step_size", params.reverse_mppi.step_size, 0.5);
    private_nh_.param("reverse_mppi/warm_start_ratio", params.reverse_mppi.warm_start_ratio, 0.5);
    private_nh_.param("reverse_mppi/lambda", params.reverse_mppi.lambda, 10.0);
    private_nh_.param("reverse_mppi/alpha", params.reverse_mppi.alpha, 0.1);
    private_nh_.param("reverse_mppi/non_biased_sampling_rate", params.reverse_mppi.non_biased_sampling_rate, 0.1);
    private_nh_.param("reverse_mppi/steer_cov", params.reverse_mppi.steer_cov, 0.5);
    // private_nh_.param("reverse_mppi/accel_cov", params.accel_cov, 0.5);

    // Stein variational MPC params
    private_nh_.param("stein_variational_mpc/sample_batch_num", params.stein_variational_mpc.sample_batch_num, 1000);
    private_nh_.param("stein_variational_mpc/lambda", params.stein_variational_mpc.lambda, 10.0);
    private_nh_.param("stein_variational_mpc/alpha", params.stein_variational_mpc.alpha, 0.1);
    private_nh_.param("stein_variational_mpc/non_biased_sampling_rate", params.stein_variational_mpc.non_biased_sampling_rate, 0.1);
    private_nh_.param("stein_variational_mpc/steer_cov", params.stein_variational_mpc.steer_cov, 0.5);
    // private_nh_.param("stein_variational_mpc/accel_cov", params.accel_cov,
    // 0.5);
    private_nh_.param("stein_variational_mpc/num_svgd_iteration", params.stein_variational_mpc.num_svgd_iteration, 100);
    private_nh_.param("stein_variational_mpc/sample_num_for_grad_estimation", params.stein_variational_mpc.sample_num_for_grad_estimation, 10);
    private_nh_.param("stein_variational_mpc/steer_cov_for_grad_estimation", params.stein_variational_mpc.steer_cov_for_grad_estimation, 0.05);
    private_nh_.param("stein_variational_mpc/svgd_step_size", params.stein_variational_mpc.svgd_step_size, 0.1);
    private_nh_.param("stein_variational_mpc/is_max_posterior_estimation", params.stein_variational_mpc.is_max_posterior_estimation, false);

    // Stein Variational Guided MPPI params
    private_nh_.param("svg_mppi/sample_batch_num", params.svg_mppi.sample_batch_num, 1000);
    private_nh_.param("svg_mppi/lambda", params.svg_mppi.lambda, 10.0);
    private_nh_.param("svg_mppi/alpha", params.svg_mppi.alpha, 0.1);
    private_nh_.param("svg_mppi/non_biased_sampling_rate", params.svg_mppi.non_biased_sampling_rate, 0.1);
    private_nh_.param("svg_mppi/steer_cov", params.svg_mppi.steer_cov, 0.5);
    // private_nh_.param("svg_mppi/accel_cov", params.accel_cov, 0.5);
    private_nh_.param("svg_mppi/guide_sample_num", params.svg_mppi.guide_sample_num, 1);
    private_nh_.param("svg_mppi/grad_lambda", params.svg_mppi.grad_lambda, 3.0);
    private_nh_.param("svg_mppi/sample_num_for_grad_estimation", params.svg_mppi.sample_num_for_grad_estimation, 10);
    private_nh_.param("svg_mppi/steer_cov_for_grad_estimation", params.svg_mppi.steer_cov_for_grad_estimation, 0.05);
    private_nh_.param("svg_mppi/svgd_step_size", params.svg_mppi.svgd_step_size, 0.1);
    private_nh_.param("svg_mppi/num_svgd_iteration", params.svg_mppi.num_svgd_iteration, 100);
    private_nh_.param("svg_mppi/is_use_nominal_solution", params.svg_mppi.is_use_nominal_solution, true);
    private_nh_.param("svg_mppi/is_covariance_adaptation", params.svg_mppi.is_covariance_adaptation, true);
    private_nh_.param("svg_mppi/gaussian_fitting_lambda", params.svg_mppi.gaussian_fitting_lambda, 0.1);
    private_nh_.param("svg_mppi/min_steer_cov", params.svg_mppi.min_steer_cov, 0.001);
    private_nh_.param("svg_mppi/max_steer_cov", params.svg_mppi.max_steer_cov, 0.1);

    const std::string mpc_mode = private_nh_.param<std::string>("mpc_mode", "");
    if (mpc_mode == "forward_mppi") {
        mpc_solver_ptr_ = std::make_unique<mppi::cpu::ForwardMPPI>(params.common, params.forward_mppi);
    } else if (mpc_mode == "reverse_mppi") {
        mpc_solver_ptr_ = std::make_unique<mppi::cpu::ReverseMPPI>(params.common, params.reverse_mppi);
    } else if (mpc_mode == "sv_mpc") {
        mpc_solver_ptr_ = std::make_unique<mppi::cpu::SteinVariationalMPC>(params.common, params.stein_variational_mpc);
    } else if (mpc_mode == "svg_mppi") {
        mpc_solver_ptr_ = std::make_unique<mppi::cpu::SVGuidedMPPI>(params.common, params.svg_mppi);
    } else {
        ROS_ERROR("Invalid MPC mode: %s", mpc_mode.c_str());
        exit(1);
    }

    // set publishers and subscribers
    pub_ackermann_cmd_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(control_cmd_topic, 1);
    timer_control_ = nh_.createTimer(ros::Duration(control_sampling_time_), &MPPIControllerROS::timer_callback, this);
    sub_activated_ = nh_.subscribe(is_activate_ad_topic, 1, &MPPIControllerROS::callback_activate_signal, this);
    if (is_localize_less_mode_) {
        sub_odom_ = nh_.subscribe(in_odom_topic, 1, &MPPIControllerROS::callback_odom,
                                  this);  // subscribe only odometry
        sub_grid_map_ = nh_.subscribe(local_costmap_id, 1, &MPPIControllerROS::callback_grid_map, this);
    } else {
        sub_odom_ = nh_.subscribe(in_odom_topic, 1, &MPPIControllerROS::callback_odom_with_pose,
                                  this);  // subscribe odometry and localized pose
        sub_reference_sdf_ = nh_.subscribe(in_reference_sdf_topic, 1, &MPPIControllerROS::callback_reference_sdf, this);

        sub_grid_map_ = nh_.subscribe(local_costmap_id, 1, &MPPIControllerROS::callback_grid_map, this);
    }

    sub_start_cmd_ = nh_.subscribe("mppi/start", 1, &MPPIControllerROS::start_cmd_callback, this);
    sub_stop_cmd_ = nh_.subscribe("mppi/stop", 1, &MPPIControllerROS::stop_cmd_callback, this);

    // For debug
    pub_best_path_ = nh_.advertise<visualization_msgs::MarkerArray>("mppi/best_path", 1, true);
    pub_nominal_path_ = nh_.advertise<visualization_msgs::MarkerArray>("mppi/nominal_path", 1, true);
    pub_candidate_paths_ = nh_.advertise<visualization_msgs::MarkerArray>("mppi/candidate_paths", 1, true);
    pub_proposal_state_distributions_ = nh_.advertise<visualization_msgs::MarkerArray>("mppi/proposal_state_distributions", 1, true);
    pub_control_covariances_ = nh_.advertise<visualization_msgs::MarkerArray>("mppi/control_covariances", 1, true);
    pub_calculation_time_ = nh_.advertise<std_msgs::Float32>("mppi/calculation_time", 1, true);
    pub_speed_ = nh_.advertise<std_msgs::Float32>("mppi/speed", 1, true);
    pub_collision_rate_ = nh_.advertise<std_msgs::Float32>("mppi/collision_rate", 1, true);
    pub_cost_ = nh_.advertise<std_msgs::Float32>("mppi/cost", 1, true);
    pub_mppi_metrics_ = nh_.advertise<mppi_metrics_msgs::MPPIMetrics>("mppi/eval_metrics", 1, true);
}

// Get current pose and velocity used with localization model
void MPPIControllerROS::callback_odom_with_pose(const nav_msgs::Odometry& odom) {
    /*Get current pose via tf*/
    geometry_msgs::TransformStamped trans_form_stamped;
    try {
        trans_form_stamped = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, ros::Time(0));
    } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(3.0, "[MPPIController]: %s", ex.what());
        return;
    };

    /*Update status*/
    robot_state_.x = trans_form_stamped.transform.translation.x;
    robot_state_.y = trans_form_stamped.transform.translation.y;
    const double _yaw = tf2::getYaw(trans_form_stamped.transform.rotation);
    robot_state_.yaw = std::atan2(std::sin(_yaw), std::cos(_yaw));
    robot_state_.vel = odom.twist.twist.linear.x;

    is_robot_state_ok_ = true;
}

// Get only current velocity used with localization less model
void MPPIControllerROS::callback_odom(const nav_msgs::Odometry& odom) {
    /*Update status*/
    robot_state_.x = 0.0;
    robot_state_.y = 0.0;
    robot_state_.yaw = 0.0;
    robot_state_.vel = odom.twist.twist.linear.x;

    is_robot_state_ok_ = true;
}

void MPPIControllerROS::callback_activate_signal(const std_msgs::Bool& is_activate) {
    is_activate_ad_ = static_cast<bool>(is_activate.data);
}

void MPPIControllerROS::callback_grid_map(const grid_map_msgs::GridMap& grid_map) {
    // make grid map for obstacle layer
    if (!grid_map::GridMapRosConverter::fromMessage(grid_map, obstacle_map_)) {
        ROS_ERROR("[MPPIControllerROS]Failed to convert grid map to grid map");
        return;
    }

    is_costmap_ok_ = true;
}

void MPPIControllerROS::callback_reference_sdf(const grid_map_msgs::GridMap& grid_map) {
    // make grid map for reference path layer
    if (!grid_map::GridMapRosConverter::fromMessage(grid_map, reference_sdf_)) {
        ROS_ERROR("[MPPIControllerROS]Failed to convert reference sdf to grid map");
        return;
    }
    is_reference_sdf_ok_ = true;
}

void MPPIControllerROS::start_cmd_callback([[maybe_unused]] const std_msgs::Empty& msg) {
    is_start_ = true;
    ROS_INFO("[MPD] start cmd received");
}

void MPPIControllerROS::stop_cmd_callback([[maybe_unused]] const std_msgs::Empty& msg) {
    is_start_ = false;
    ROS_INFO("[MPD] stop cmd received");
}

// Control loop
void MPPIControllerROS::timer_callback([[maybe_unused]] const ros::TimerEvent& te) {
    /* Status check */
    if (is_localize_less_mode_) {
        if (!is_robot_state_ok_ || !is_costmap_ok_) {
            ROS_WARN_THROTTLE(5.0, "[MPPIControllerROS] Not ready: Robot state: %d, Map: %d", is_robot_state_ok_, is_costmap_ok_);
            return;
        }
    } else {
        if (!is_robot_state_ok_ || !is_reference_sdf_ok_ || !is_costmap_ok_) {
            ROS_WARN_THROTTLE(5.0,
                              "[MPPIControllerROS] Not ready: Robot state: %d, "
                              "Reference SDF: %d, Map: %d",
                              is_robot_state_ok_, is_reference_sdf_ok_, is_costmap_ok_);
            return;
        }
    }

    if (!is_activate_ad_ && !is_simulation_) {
        // publish zero control command and return
        ROS_WARN_THROTTLE(5.0, "[MPPIControllerROS] waiting to activate signal");
        control_msg_.header.stamp = ros::Time::now();
        control_msg_.drive.steering_angle = 0.0;
        control_msg_.drive.speed = 0.0;
        pub_ackermann_cmd_.publish(control_msg_);

        return;
    }

    if (!is_start_) {
        // publish zero control command and
        ROS_WARN_THROTTLE(5.0, "[MPPIControllerROS] waiting to start signal from rviz");
        control_msg_.header.stamp = ros::Time::now();
        control_msg_.drive.steering_angle = 0.0;
        control_msg_.drive.speed = 0.0;
        pub_ackermann_cmd_.publish(control_msg_);

        // publish predicted state for debug
        const int num_visualized_samples = 100;
        const auto [predict_state_seq_batch, weights] = mpc_solver_ptr_->get_state_seq_candidates(num_visualized_samples);
        publish_candidate_paths(predict_state_seq_batch, weights, pub_candidate_paths_);

        return;
    }

    mtx_.lock();

    stop_watch_.lap();

    mpc_solver_ptr_->set_obstacle_map(obstacle_map_);
    if (!is_localize_less_mode_) {
        mpc_solver_ptr_->set_reference_map(reference_sdf_);
    }

    // steer observer
    const double prev_steer_cmd = control_msg_.drive.steering_angle;
    robot_state_.steer = robot_state_.steer + (prev_steer_cmd - robot_state_.steer) * (1 - exp(-control_sampling_time_ / steer_1st_delay_));

    // Solve MPC
    mppi::cpu::State initial_state = mppi::cpu::State::Zero();
    initial_state[STATE_SPACE::x] = robot_state_.x;
    initial_state[STATE_SPACE::y] = robot_state_.y;
    initial_state[STATE_SPACE::yaw] = robot_state_.yaw;
    initial_state[STATE_SPACE::vel] = robot_state_.vel;
    initial_state[STATE_SPACE::steer] = robot_state_.steer;
    const auto [updated_control_seq, collision_rate] = mpc_solver_ptr_->solve(initial_state);

    mtx_.unlock();

    // predict state seq
    const auto [best_state_seq, state_cost, collision_cost, input_error] = mpc_solver_ptr_->get_predictive_seq(initial_state, updated_control_seq);

    // Publish control command
    const double steering_angle = updated_control_seq(0, CONTROL_SPACE::steer);
    // const double accel = updated_control_seq(0, CONTROL_SPACE::accel);

    // const double speed = robot_state_.vel + accel * control_sampling_time_;
    // speed_cmd_ += accel * control_sampling_time_;
    control_msg_.header.stamp = ros::Time::now();
    control_msg_.drive.steering_angle = std::atan2(std::sin(steering_angle), std::cos(steering_angle));
    double speed_cmd = 0.0;
    if (constant_speed_mode_ || is_localize_less_mode_) {
        // Note: constant speed is used in localize_less_mode because reference sdf
        // is not available
        speed_cmd = reference_speed_;
    } else {
        if (reference_sdf_.isInside(grid_map::Position(robot_state_.x, robot_state_.y))) {
            // speed_cmd = reference_sdf_.atPosition(speed_field_layer_name_,
            // grid_map::Position(ahead_x, ahead_y));
            speed_cmd = reference_sdf_.atPosition(speed_field_layer_name_, grid_map::Position(robot_state_.x, robot_state_.y));
        } else {
            speed_cmd = reference_speed_;
            ROS_WARN(
                "[MPPIControllerROS] Robot is out of reference sdf map. Use "
                "constant speed mode.");
        }
    }
    control_msg_.drive.speed = speed_cmd;

    pub_ackermann_cmd_.publish(control_msg_);

    const double calculation_time = stop_watch_.lap();

    // ==========  For debug ===============
    if (is_visualize_mppi_) {
        // publish predicted state
        const int num_visualized_samples = 100;
        const auto [predict_state_seq_batch, weights] = mpc_solver_ptr_->get_state_seq_candidates(num_visualized_samples);
        publish_candidate_paths(predict_state_seq_batch, weights, pub_candidate_paths_);

        // Get covariance of proposed distribution
        const auto cov_matrices = mpc_solver_ptr_->get_cov_matrices();

        // publish best state
        publish_traj(best_state_seq, "best_path", "red", pub_best_path_);

        // publish control covariance
        publish_control_covs(best_state_seq, cov_matrices, pub_control_covariances_);

        // publish proposed state distribution
        const auto [mean, xycov_mat] = mpc_solver_ptr_->get_proposed_state_distribution();
        publish_state_seq_dists(mean, xycov_mat, pub_proposal_state_distributions_);

        // publish nominal state
        const auto nominal_control_seq = mpc_solver_ptr_->get_control_seq();
        const auto nominal_state_seq = std::get<0>(mpc_solver_ptr_->get_predictive_seq(initial_state, nominal_control_seq));
        publish_path(nominal_state_seq, "nominal_path", "g", pub_nominal_path_);
    } else {
        // publish best state
        publish_traj(best_state_seq, "best_path", "red", pub_best_path_);
    }

    // publish cost of the best state seq
    std_msgs::Float32 cost_msg;
    cost_msg.data = static_cast<float>(state_cost + collision_cost);
    pub_cost_.publish(cost_msg);

    // publish speed
    std_msgs::Float32 speed_msg;
    speed_msg.data = robot_state_.vel;
    pub_speed_.publish(speed_msg);

    // publish calculate time
    std_msgs::Float32 calculation_time_msg;
    calculation_time_msg.data = static_cast<float>(calculation_time);
    pub_calculation_time_.publish(calculation_time_msg);

    // publish collision rate
    std_msgs::Float32 collision_rate_msg;
    collision_rate_msg.data = static_cast<float>(collision_rate);
    pub_collision_rate_.publish(collision_rate_msg);

    // publish mppi metrics
    mppi_metrics_msgs::MPPIMetrics mppi_metrics_msg;
    mppi_metrics_msg.header.stamp = ros::Time::now();
    mppi_metrics_msg.calculation_time = calculation_time;
    mppi_metrics_msg.state_cost = state_cost;
    mppi_metrics_msg.collision_cost = collision_cost;
    mppi_metrics_msg.input_error = input_error;
    pub_mppi_metrics_.publish(mppi_metrics_msg);
}

void MPPIControllerROS::publish_traj(const mppi::cpu::StateSeq& state_seq,
                                     const std::string& name_space,
                                     const std::string& rgb,
                                     const ros::Publisher& publisher) const {
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker arrow;
    if (is_localize_less_mode_) {
        arrow.header.frame_id = robot_frame_id_;
    } else {
        arrow.header.frame_id = map_frame_id_;
    }
    arrow.header.stamp = ros::Time::now();
    arrow.ns = name_space;
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
    arrow.color.r = ((rgb == "r" || rgb == "red") ? 1.0 : 0.0);
    arrow.color.g = ((rgb == "g" || rgb == "green") ? 1.0 : 0.0);
    arrow.color.b = ((rgb == "b" || rgb == "blue") ? 1.0 : 0.0);

    // arrow.lifetime = ros::Duration(0.1);
    arrow.points.resize(2);

    for (int i = 0; i < state_seq.rows(); i++) {
        arrow.id = i;
        const auto state = state_seq.row(i);
        const double length = abs(state[STATE_SPACE::vel]) * 0.1;
        geometry_msgs::Point start;
        start.x = state[STATE_SPACE::x];
        start.y = state[STATE_SPACE::y];
        start.z = 0.1;

        geometry_msgs::Point end;
        end.x = state[STATE_SPACE::x] + length * cos(state[STATE_SPACE::yaw]);
        end.y = state[STATE_SPACE::y] + length * sin(state[STATE_SPACE::yaw]);
        end.z = 0.1;

        arrow.points[0] = start;
        arrow.points[1] = end;

        marker_array.markers.push_back(arrow);
    }
    publisher.publish(marker_array);
}

void MPPIControllerROS::publish_path(const mppi::cpu::StateSeq& state_seq,
                                     const std::string& name_space,
                                     const std::string& rgb,
                                     const ros::Publisher& publisher) const {
    visualization_msgs::MarkerArray marker_array;

    visualization_msgs::Marker line;
    if (is_localize_less_mode_) {
        line.header.frame_id = robot_frame_id_;
    } else {
        line.header.frame_id = map_frame_id_;
    }
    line.header.stamp = ros::Time::now();
    line.ns = name_space + "_line";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.x = 0.0;
    line.pose.orientation.y = 0.0;
    line.pose.orientation.z = 0.0;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.01;
    line.color.a = 1.0;
    line.color.r = ((rgb == "r" || rgb == "red") ? 1.0 : 0.0);
    line.color.g = ((rgb == "g" || rgb == "green") ? 1.0 : 0.0);
    line.color.b = ((rgb == "b" || rgb == "blue") ? 1.0 : 0.0);
    // line.lifetime = ros::Duration(0.1);

    // nodes
    visualization_msgs::Marker nodes;
    if (is_localize_less_mode_) {
        nodes.header.frame_id = robot_frame_id_;
    } else {
        nodes.header.frame_id = map_frame_id_;
    }
    nodes.header.stamp = ros::Time::now();
    nodes.ns = name_space + "_nodes";
    nodes.id = 1;
    nodes.type = visualization_msgs::Marker::SPHERE_LIST;
    nodes.action = visualization_msgs::Marker::ADD;
    nodes.pose.orientation.x = 0.0;
    nodes.pose.orientation.y = 0.0;
    nodes.pose.orientation.z = 0.0;
    nodes.pose.orientation.w = 1.0;
    const double node_size = 0.1;
    nodes.scale.x = node_size;
    nodes.scale.y = node_size;
    nodes.scale.z = node_size;
    nodes.color.a = 1.0;
    nodes.color.r = ((rgb == "r" || rgb == "red") ? 1.0 : 0.0);
    nodes.color.g = ((rgb == "g" || rgb == "green") ? 1.0 : 0.0);
    nodes.color.b = ((rgb == "b" || rgb == "blue") ? 1.0 : 0.0);
    // nodes.lifetime = ros::Duration(0.1);

    for (int j = 0; j < state_seq.rows(); j++) {
        geometry_msgs::Point point;
        point.x = state_seq(j, STATE_SPACE::x);
        point.y = state_seq(j, STATE_SPACE::y);
        point.z = 0.1;
        line.points.push_back(point);
        nodes.points.push_back(point);
    }
    marker_array.markers.push_back(line);
    marker_array.markers.push_back(nodes);

    publisher.publish(marker_array);
}

void MPPIControllerROS::publish_candidate_paths(const std::vector<mppi::cpu::StateSeq>& state_seq_batch,
                                                const std::vector<double>& weights,
                                                const ros::Publisher& publisher) const {
    assert(state_seq_batch.size() == weights.size());

    visualization_msgs::MarkerArray marker_array;

    const double max_weight = weights[0];
    const double max_node_size = 0.05;
    for (size_t i = 0; i < state_seq_batch.size(); i++) {
        visualization_msgs::Marker line;
        if (is_localize_less_mode_) {
            line.header.frame_id = robot_frame_id_;
        } else {
            line.header.frame_id = map_frame_id_;
        }
        line.header.stamp = ros::Time::now();
        line.ns = "candidate_path_line";
        line.id = i;
        line.type = visualization_msgs::Marker::LINE_STRIP;
        line.action = visualization_msgs::Marker::ADD;
        line.pose.orientation.x = 0.0;
        line.pose.orientation.y = 0.0;
        line.pose.orientation.z = 0.0;
        line.pose.orientation.w = 1.0;
        line.scale.x = 0.01;
        line.color.a = 0.3;
        line.color.r = 0.0;
        line.color.g = 0.5;
        line.color.b = 1.0;
        // line.lifetime = ros::Duration(0.1);

        // nodes
        visualization_msgs::Marker nodes;
        if (is_localize_less_mode_) {
            nodes.header.frame_id = robot_frame_id_;
        } else {
            nodes.header.frame_id = map_frame_id_;
        }
        nodes.header.stamp = ros::Time::now();
        nodes.ns = "candidate_path_nodes";
        nodes.id = line.id + 10000 + i;
        nodes.type = visualization_msgs::Marker::SPHERE_LIST;
        nodes.action = visualization_msgs::Marker::ADD;
        nodes.pose.orientation.x = 0.0;
        nodes.pose.orientation.y = 0.0;
        nodes.pose.orientation.z = 0.0;
        nodes.pose.orientation.w = 1.0;
        nodes.scale.x = weights[i] * max_node_size / max_weight + 0.01;
        nodes.scale.y = 0.01;
        nodes.scale.z = 0.01;
        nodes.color.a = 0.6;
        nodes.color.r = 0.0;
        nodes.color.g = 0.5;
        nodes.color.b = 1.0;
        // nodes.lifetime = ros::Duration(0.1);

        for (int j = 0; j < state_seq_batch.at(0).rows(); j++) {
            geometry_msgs::Point point;
            point.x = state_seq_batch.at(i)(j, STATE_SPACE::x);
            point.y = state_seq_batch.at(i)(j, STATE_SPACE::y);
            point.z = 0.0;
            line.points.push_back(point);
            nodes.points.push_back(point);
        }
        marker_array.markers.push_back(line);
        marker_array.markers.push_back(nodes);
    }
    publisher.publish(marker_array);
}

void MPPIControllerROS::publish_control_covs(const mppi::cpu::StateSeq& state_seq,
                                             const mppi::cpu::ControlSeqCovMatrices& cov_matrices,
                                             const ros::Publisher& publisher) const {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker ellipse;
    if (is_localize_less_mode_) {
        ellipse.header.frame_id = robot_frame_id_;
    } else {
        ellipse.header.frame_id = map_frame_id_;
    }
    ellipse.header.stamp = ros::Time::now();
    ellipse.ns = "steer_covariance";
    ellipse.type = visualization_msgs::Marker::SPHERE;
    ellipse.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Vector3 ellipse_scale;
    ellipse_scale.x = 0.0;
    ellipse_scale.y = 0.0;
    ellipse_scale.z = 0.0;
    ellipse.scale = ellipse_scale;
    ellipse.pose.position.x = 0.0;
    ellipse.pose.position.y = 0.0;
    ellipse.pose.position.z = 0.0;
    ellipse.pose.orientation.x = 0.0;
    ellipse.pose.orientation.y = 0.0;
    ellipse.pose.orientation.z = 0.0;
    ellipse.pose.orientation.w = 1.0;
    ellipse.color.a = 0.4;
    // yellow
    ellipse.color.r = 1.0;
    ellipse.color.g = 1.0;
    ellipse.color.b = 0.0;
    // ellipse.lifetime = ros::Duration(1.0);

    const double control_cov_scale = 1.5;

    for (int i = 0; i < state_seq.rows() - 1; i++) {
        // ellipse of proposal distribution
        ellipse.id = i;
        const auto cov_matrix = cov_matrices[i];
        // Only publish steer
        const double cov = cov_matrix(CONTROL_SPACE::steer, CONTROL_SPACE::steer);
        const double std_dev = sqrt(cov) * control_cov_scale;
        ellipse_scale.x = 2.0 * std_dev + 0.01;
        ellipse_scale.y = 2.0 * std_dev + 0.01;
        ellipse_scale.z = 0.01;
        ellipse.scale = ellipse_scale;
        ellipse.pose.position.x = state_seq(i, STATE_SPACE::x);
        ellipse.pose.position.y = state_seq(i, STATE_SPACE::y);

        marker_array.markers.push_back(ellipse);
    }
    publisher.publish(marker_array);
}

void MPPIControllerROS::publish_state_seq_dists(const mppi::cpu::StateSeq& state_seq,
                                                const mppi::cpu::XYCovMatrices& cov_matrices,
                                                const ros::Publisher& publisher) const {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker ellipse;
    if (is_localize_less_mode_) {
        ellipse.header.frame_id = robot_frame_id_;
    } else {
        ellipse.header.frame_id = map_frame_id_;
    }
    ellipse.header.stamp = ros::Time::now();
    ellipse.ns = "proposal_state_distributions";
    ellipse.type = visualization_msgs::Marker::SPHERE;
    ellipse.action = visualization_msgs::Marker::ADD;
    geometry_msgs::Vector3 ellipse_scale;
    ellipse_scale.x = 0.0;
    ellipse_scale.y = 0.0;
    ellipse_scale.z = 0.0;
    ellipse.scale = ellipse_scale;
    ellipse.pose.position.x = 0.0;
    ellipse.pose.position.y = 0.0;
    ellipse.pose.position.z = 0.0;
    ellipse.pose.orientation.x = 0.0;
    ellipse.pose.orientation.y = 0.0;
    ellipse.pose.orientation.z = 0.0;
    ellipse.pose.orientation.w = 1.0;
    ellipse.color.a = 0.4;
    // yellow
    ellipse.color.r = 0.0;
    ellipse.color.g = 0.0;
    ellipse.color.b = 1.0;
    // ellipse.lifetime = ros::Duration(0.1);

    for (int i = 0; i < state_seq.rows(); i++) {
        ellipse.id = i;
        ellipse.pose.position.x = state_seq(i, STATE_SPACE::x);
        ellipse.pose.position.y = state_seq(i, STATE_SPACE::y);

        // get length and angle of major and minor axis
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(cov_matrices.at(i).block<2, 2>(0, 0));
        const auto eigen_value = eigensolver.eigenvalues();
        const auto eigen_vector = eigensolver.eigenvectors();
        const double major_axis_length = std::sqrt(eigen_value(0));
        const double minor_axis_length = std::sqrt(eigen_value(1));
        const double yaw = std::atan2(eigen_vector(1, 0), eigen_vector(0, 0));

        ellipse.scale.x = 2.0 * major_axis_length + 0.1;
        ellipse.scale.y = 2.0 * minor_axis_length + 0.1;
        ellipse.scale.z = 0.1;
        // yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        ellipse.pose.orientation.x = q.x();
        ellipse.pose.orientation.y = q.y();
        ellipse.pose.orientation.z = q.z();
        ellipse.pose.orientation.w = q.w();

        marker_array.markers.push_back(ellipse);
    }
    publisher.publish(marker_array);
}

}  // namespace mppi