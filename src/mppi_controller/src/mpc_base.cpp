#include "mppi_controller/mpc_base.hpp"

namespace mppi {
namespace cpu {
    // #### Public functions ####

    MPCBase::MPCBase(const Params::Common& params, const size_t& sample_num)
        : is_localize_less_mode_(params.is_localize_less_mode),
          thread_num_(params.thread_num),
          prediction_step_size_(static_cast<size_t>(params.prediction_step_size)),
          prediction_interval_(params.prediction_interval),
          reference_speed_(params.reference_speed),
          lr_(params.lr),
          lf_(params.lf),
          max_accel_(params.max_accel),
          min_accel_(params.min_accel),
          steer_delay_(params.steer_delay),
          steer_delay_steps_(static_cast<size_t>(std::ceil(steer_delay_ / prediction_interval_))),
          steer_delay_tau_(params.steer_1st_delay),
          q_dist_(params.q_dist),
          q_angle_(params.q_angle),
          // q_speed_(params.q_speed),
          collision_weight_(params.collision_weight),
          q_terminal_dist_(params.q_terminal_dist),
          q_terminal_angle_(params.q_terminal_angle)
    // q_terminal_speed_(params.q_terminal_speed),
    {
        // set parameters

        if (params.speed_prediction_mode == "constant") {
            speed_prediction_mode_ = SpeedPredictionMode::CONSTANT;
        } else if (params.speed_prediction_mode == "linear") {
            speed_prediction_mode_ = SpeedPredictionMode::LINEAR;
        } else if (params.speed_prediction_mode == "reference") {
            speed_prediction_mode_ = SpeedPredictionMode::REFERENCE;
        } else {
            std::cout << "[MPPI] Invalid speed prediction mode: " << params.speed_prediction_mode << std::endl;
            exit(1);
        }

        // check validate mode
        if (speed_prediction_mode_ == SpeedPredictionMode::REFERENCE && is_localize_less_mode_) {
            std::cout << "[MPPI] Invalid speed prediction mode: " << params.speed_prediction_mode << std::endl;
            std::cout << "[MPPI] Speed prediction mode must be constant or linear in localize less mode" << std::endl;
            exit(1);
        }

        // initialize inner variables
        global_state_seq_candidates_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            sample_num, Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim));
        local_state_seq_candidates_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            sample_num, Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim));
    }

    void MPCBase::set_obstacle_map(const grid_map::GridMap& obstacle_map) { obstacle_map_ = obstacle_map; }

    void MPCBase::set_reference_map(const grid_map::GridMap& reference_map) { reference_map_ = reference_map; }

    std::pair<std::vector<double>, std::vector<double>> MPCBase::calc_sample_costs(const PriorSamplesWithCosts& sampler, const State& init_state) {
        if (is_localize_less_mode_) {
            return calc_sample_costs(sampler, init_state, obstacle_map_, &local_state_seq_candidates_);
        } else {
            return calc_sample_costs(sampler, init_state, obstacle_map_, reference_map_, &global_state_seq_candidates_, &local_state_seq_candidates_);
        }
    }

    std::tuple<StateSeq, double, double> MPCBase::get_predictive_seq(const State& initial_state, const ControlSeq& control_input_seq) const {
        if (is_localize_less_mode_) {
            const StateSeq local_state_seq = predict_state_seq(control_input_seq, initial_state, reference_map_);
            const auto [cost, collision_cost] = state_cost(local_state_seq, obstacle_map_);
            return std::make_tuple(local_state_seq, cost - collision_cost, collision_cost);
        } else {
            StateSeq global_state_seq = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);
            StateSeq local_sate_seq = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);
            predict_state_seq(control_input_seq, initial_state, reference_map_, &global_state_seq, &local_sate_seq);
            const auto [cost, collision_cost] = state_cost(global_state_seq, local_sate_seq, obstacle_map_, reference_map_);
            return std::make_tuple(global_state_seq, cost - collision_cost, collision_cost);
        }
    }

    std::pair<std::vector<StateSeq>, std::vector<double>> MPCBase::get_state_seq_candidates(const int& _num_samples,
                                                                                            const std::vector<double>& weights) const {
        if (weights.size() == 0) {
            std::cerr << "weights is empty" << std::endl;
            return std::make_pair(std::vector<StateSeq>(), std::vector<double>());
        }

        const int num_samples = std::min(static_cast<int>(weights.size()), _num_samples);

        std::vector<double> sorted_weights = weights;
        std::sort(sorted_weights.data(), sorted_weights.data() + sorted_weights.size());

        // get indices of top num_samples
        std::vector<int> indices;
        for (int i = 0; i < num_samples; i++) {
            const double weight = sorted_weights[sorted_weights.size() - 1 - i];
            const int index = std::distance(weights.data(), std::find(weights.data(), weights.data() + weights.size(), weight));
            indices.push_back(index);
        }

        std::vector<double> selected_weights(num_samples);
        std::vector<StateSeq> selected_state_seq_candidates(num_samples);
        for (int i = 0; i < num_samples; i++) {
            selected_weights[i] = weights[indices.at(i)];
            if (is_localize_less_mode_) {
                selected_state_seq_candidates[i] = local_state_seq_candidates_.at(indices.at(i));
            } else {
                selected_state_seq_candidates[i] = global_state_seq_candidates_.at(indices.at(i));
            }
        }

        return std::make_pair(selected_state_seq_candidates, selected_weights);
    }

    std::pair<StateSeq, XYCovMatrices> MPCBase::get_proposed_distribution() const {
        if (is_localize_less_mode_) {
            return calc_state_distribution(local_state_seq_candidates_);
        } else {
            return calc_state_distribution(global_state_seq_candidates_);
        }
    }

    // #### Private functions ####
    std::pair<std::vector<double>, std::vector<double>> MPCBase::calc_sample_costs(const PriorSamplesWithCosts& sampler,
                                                                                   const State& global_init_state,
                                                                                   const grid_map::GridMap& obstacle_map,
                                                                                   const grid_map::GridMap& reference_map,
                                                                                   StateSeqBatch* global_state_seq_candidates,
                                                                                   StateSeqBatch* local_state_seq_candidates) const {
        std::vector<double> costs(sampler.get_num_samples());
        std::vector<double> collision_costs(sampler.get_num_samples());

        // Rollout for each control sequence
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < sampler.get_num_samples(); i++) {
            // Predict state sequence
            predict_state_seq(sampler.noised_control_seq_samples_[i], global_init_state, reference_map, &global_state_seq_candidates->at(i),
                              &local_state_seq_candidates->at(i));

            // Calculate cost
            const auto [cost, collision_cost] =
                state_cost(global_state_seq_candidates->at(i), local_state_seq_candidates->at(i), obstacle_map, reference_map);
            costs.at(i) = cost;
            collision_costs.at(i) = collision_cost;
        }
        return std::make_pair(costs, collision_costs);
    }

    std::pair<std::vector<double>, std::vector<double>> MPCBase::calc_sample_costs(const PriorSamplesWithCosts& sampler,
                                                                                   const State& local_init_state,
                                                                                   const grid_map::GridMap& obstacle_map,
                                                                                   StateSeqBatch* local_state_seq_candidates) const {
        std::vector<double> costs(sampler.get_num_samples());
        std::vector<double> collision_costs(sampler.get_num_samples());

// Rollout for each control sequence
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < sampler.get_num_samples(); i++) {
            // Predict state sequence
            local_state_seq_candidates->at(i) = predict_state_seq(sampler.noised_control_seq_samples_[i], local_init_state, obstacle_map);

            // calculate cost
            const auto [cost, collision_cost] = state_cost(local_state_seq_candidates->at(i), obstacle_map);
            costs.at(i) = cost;
            collision_costs.at(i) = collision_cost;
        }

        return std::make_pair(costs, collision_costs);
    }

    // Predict local or global state sequence from control sequence
    StateSeq MPCBase::predict_state_seq(const ControlSeq& control_seq, const State& init_state, const grid_map::GridMap& reference_map) const {
        StateSeq state_seq = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);
        // observed state
        state_seq.row(0) = init_state;

        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            double steer_angle = 0.0;
            if (i <= steer_delay_steps_) {
                // To consider input delay
                steer_angle = control_seq(0, CONTROL_SPACE::steer);
            } else {
                steer_angle = control_seq(i, CONTROL_SPACE::steer);
            }

            // const double accel = control_seq(i, CONTROL_SPACE::accel);

            const double x = state_seq(i, STATE_SPACE::x);
            const double y = state_seq(i, STATE_SPACE::y);
            const double yaw = state_seq(i, STATE_SPACE::yaw);
            const double vel = state_seq(i, STATE_SPACE::vel);
            const double steer = state_seq(i, STATE_SPACE::steer);

            // Kinematic Bicycle Model
            const double beta = atan(lf_ / (lf_ + lr_) * tan(steer_angle));
            const double delta_x = vel * cos(yaw + beta) * prediction_interval_;
            const double delta_y = vel * sin(yaw + beta) * prediction_interval_;
            const double delta_yaw = vel * sin(beta) / lr_ * prediction_interval_;
            const double delta_steer = ((steer_angle - steer) / steer_delay_tau_) * prediction_interval_;
            double next_vel = 0.0;
            if (speed_prediction_mode_ == SpeedPredictionMode::CONSTANT) {
                next_vel = constant_speed_prediction(vel);
            } else if (speed_prediction_mode_ == SpeedPredictionMode::LINEAR) {
                next_vel = linear_speed_prediction(vel, reference_speed_, prediction_interval_, min_accel_, max_accel_);
            } else if (speed_prediction_mode_ == SpeedPredictionMode::REFERENCE) {
                next_vel = reference_speed_prediction(x, y, reference_map);
            }

            state_seq(i + 1, STATE_SPACE::x) = x + delta_x;
            state_seq(i + 1, STATE_SPACE::y) = y + delta_y;
            state_seq(i + 1, STATE_SPACE::yaw) = std::atan2(sin(yaw + delta_yaw), cos(yaw + delta_yaw));
            state_seq(i + 1, STATE_SPACE::vel) = next_vel;
            state_seq(i + 1, STATE_SPACE::steer) = steer + delta_steer;
        }
        return state_seq;
    }

    // Predict both local and global state sequence from control sequence
    void MPCBase::predict_state_seq(const ControlSeq& control_seq,
                                    const State& global_init_state,
                                    const grid_map::GridMap& reference_map,
                                    StateSeq* global_state_seq,
                                    StateSeq* local_state_seq) const {
        // observed state
        global_state_seq->row(0) = global_init_state;

        // // observed state
        local_state_seq->row(0) = State::Zero(STATE_SPACE::dim);
        local_state_seq->row(0)(STATE_SPACE::vel) = global_init_state(STATE_SPACE::vel);

        // This is for linear prediction mode of speed
        const double init_x = global_init_state(STATE_SPACE::x);
        const double init_y = global_init_state(STATE_SPACE::y);
        double current_reference_speed = 0;
        if (reference_map.isInside(grid_map::Position(init_x, init_y))) {
            current_reference_speed = reference_map.atPosition(speed_field_layer_name_, grid_map::Position(init_x, init_y));
        }

        // double prev_steer_angle = control_seq(0, CONTROL_SPACE::steer);
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            double steer_angle = 0.0;
            if (i <= steer_delay_steps_) {
                // To consider input delay
                steer_angle = control_seq(0, CONTROL_SPACE::steer);
            } else {
                steer_angle = control_seq(i, CONTROL_SPACE::steer);
            }
            // const double steer_angle = control_seq(i, CONTROL_SPACE::steer);
            // const double accel = control_seq(i, CONTROL_SPACE::accel);

            // Update global state
            const double global_x = global_state_seq->row(i)(STATE_SPACE::x);
            const double global_y = global_state_seq->row(i)(STATE_SPACE::y);
            const double global_yaw = global_state_seq->row(i)(STATE_SPACE::yaw);
            const double global_vel = global_state_seq->row(i)(STATE_SPACE::vel);
            const double global_steer = global_state_seq->row(i)(STATE_SPACE::steer);

            const double beta_global = atan(lf_ / (lf_ + lr_) * tan(global_steer));

            // Kinematic Bicycle Model with steer 1st order delay
            const double delta_global_x = global_vel * cos(global_yaw + beta_global) * prediction_interval_;
            const double delta_global_y = global_vel * sin(global_yaw + beta_global) * prediction_interval_;
            const double delta_global_yaw = global_vel * sin(beta_global) / lr_ * prediction_interval_;
            const double delta_global_steer = ((steer_angle - global_steer) / steer_delay_tau_) * prediction_interval_;

            double next_vel = 0.0;
            if (speed_prediction_mode_ == SpeedPredictionMode::CONSTANT) {
                next_vel = constant_speed_prediction(global_vel);
            } else if (speed_prediction_mode_ == SpeedPredictionMode::LINEAR) {
                next_vel = linear_speed_prediction(global_vel, current_reference_speed, prediction_interval_, min_accel_, max_accel_);
            } else if (speed_prediction_mode_ == SpeedPredictionMode::REFERENCE) {
                next_vel = reference_speed_prediction(global_x, global_y, reference_map);
            }

            global_state_seq->row(i + 1)(STATE_SPACE::x) = global_x + delta_global_x;
            global_state_seq->row(i + 1)(STATE_SPACE::y) = global_y + delta_global_y;
            global_state_seq->row(i + 1)(STATE_SPACE::yaw) = std::atan2(sin(global_yaw + delta_global_yaw), cos(global_yaw + delta_global_yaw));
            global_state_seq->row(i + 1)(STATE_SPACE::vel) = next_vel;
            global_state_seq->row(i + 1)(STATE_SPACE::steer) = global_steer + delta_global_steer;

            // Update local state
            const double local_x = local_state_seq->row(i)(STATE_SPACE::x);
            const double local_y = local_state_seq->row(i)(STATE_SPACE::y);
            const double local_yaw = local_state_seq->row(i)(STATE_SPACE::yaw);
            const double local_vel = local_state_seq->row(i)(STATE_SPACE::vel);
            const double local_steer = local_state_seq->row(i)(STATE_SPACE::steer);

            const double beta_local = atan(lf_ / (lf_ + lr_) * tan(local_steer));

            // Kinematic Bicycle Model with steer 1st order delay
            const double delta_local_x = local_vel * cos(local_yaw + beta_local) * prediction_interval_;
            const double delta_local_y = local_vel * sin(local_yaw + beta_local) * prediction_interval_;
            const double delta_local_yaw = local_vel * sin(beta_local) / lr_ * prediction_interval_;
            const double delta_local_steer = ((steer_angle - local_steer) / steer_delay_tau_) * prediction_interval_;

            local_state_seq->row(i + 1)(STATE_SPACE::x) = local_x + delta_local_x;
            local_state_seq->row(i + 1)(STATE_SPACE::y) = local_y + delta_local_y;
            local_state_seq->row(i + 1)(STATE_SPACE::yaw) = std::atan2(sin(local_yaw + delta_local_yaw), cos(local_yaw + delta_local_yaw));
            local_state_seq->row(i + 1)(STATE_SPACE::vel) = next_vel;
            local_state_seq->row(i + 1)(STATE_SPACE::steer) = local_steer + delta_local_steer;
        }
    }

    double MPCBase::constant_speed_prediction(const double& current_speed) const { return current_speed; }

    double MPCBase::linear_speed_prediction(const double& current_speed,
                                            const double& target_speed,
                                            const double& prediction_interval,
                                            const double& min_accel,
                                            const double& max_accel) const {
        if (current_speed < target_speed) {
            return std::min(current_speed + max_accel * prediction_interval, target_speed);
        } else if (current_speed > target_speed) {
            return std::max(current_speed - min_accel * prediction_interval, target_speed);
        } else {
            return current_speed;
        }
    }

    double MPCBase::reference_speed_prediction(const double& pos_x, const double& pos_y, const grid_map::GridMap& reference_map) const {
        if (reference_map.isInside(grid_map::Position(pos_x, pos_y))) {
            return reference_map.atPosition(speed_field_layer_name_, grid_map::Position(pos_x, pos_y));
        } else {
            return 0.0;
        }
    }

    // calculate state cost using only obstacle map from local state sequence
    std::pair<double, double> MPCBase::state_cost(const StateSeq& local_state_seq, const grid_map::GridMap& obstacle_map) const {
        // calc cost for each state
        double sum_collision_cost = 0.0;
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            // state cost
            const State local_state = local_state_seq.row(i);

            // collision cost
            // check inside of gridmap
            double collision_cost = 10.0;
            if (obstacle_map.isInside(grid_map::Position(local_state(STATE_SPACE::x), local_state(STATE_SPACE::y)))) {
                collision_cost =
                    obstacle_map.atPosition(obstacle_layer_name_, grid_map::Position(local_state(STATE_SPACE::x), local_state(STATE_SPACE::y)));
            }

            sum_collision_cost += collision_cost * collision_weight_;
        }

        // terminal cost
        const State terminal_local_state = local_state_seq.row(prediction_step_size_ - 1);
        double collision_cost = 10.0;
        if (obstacle_map.isInside(grid_map::Position(terminal_local_state(STATE_SPACE::x), terminal_local_state(STATE_SPACE::y)))) {
            collision_cost = obstacle_map.atPosition(obstacle_layer_name_,
                                                     grid_map::Position(terminal_local_state(STATE_SPACE::x), terminal_local_state(STATE_SPACE::y)));
        }

        sum_collision_cost += collision_cost * collision_weight_;

        return std::make_pair(sum_collision_cost, sum_collision_cost);
    }

    // calculate state cost using obstacle map and reference map from both global state sequence and local state sequence
    // obstacle cost is calculated from local state sequence
    // reference cost is calculated from global state sequence
    std::pair<double, double> MPCBase::state_cost(const StateSeq& global_state_seq,
                                                  const StateSeq& local_state_seq,
                                                  const grid_map::GridMap& local_obstacle_map,
                                                  const grid_map::GridMap& ref_path_map) const {
        // calc cost for each state
        double sum_ref_cost = 0.0;
        double sum_collision_cost = 0.0;
        const double max_diff_ref_path = ref_path_map.getLength().x();
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            // state cost
            const State global_state = global_state_seq.row(i);
            const State local_state = local_state_seq.row(i);
            // const double diff_speed = reference_speed_ - state(STATE_SPACE::vel);
            double diff_ref_path = max_diff_ref_path;
            double diff_angle = 2.0;
            if (ref_path_map.isInside(grid_map::Position(global_state(STATE_SPACE::x), global_state(STATE_SPACE::y)))) {
                diff_ref_path = ref_path_map.atPosition(distance_field_layer_name_,
                                                        grid_map::Position(global_state(STATE_SPACE::x), global_state(STATE_SPACE::y)));
                diff_angle =
                    global_state(STATE_SPACE::yaw) -
                    ref_path_map.atPosition(angle_field_layer_name_, grid_map::Position(global_state(STATE_SPACE::x), global_state(STATE_SPACE::y)));
                diff_angle = std::atan2(sin(diff_angle), cos(diff_angle));
            }
            // cost += q_speed_ * diff_speed * diff_speed;
            sum_ref_cost += q_dist_ * diff_ref_path * diff_ref_path;
            sum_ref_cost += q_angle_ * diff_angle * diff_angle;

            // collision cost
            // check inside of gridmap
            double collision_cost = 100.0;
            if (local_obstacle_map.isInside(grid_map::Position(local_state(STATE_SPACE::x), local_state(STATE_SPACE::y)))) {
                collision_cost =
                    local_obstacle_map.atPosition(obstacle_layer_name_, grid_map::Position(local_state(STATE_SPACE::x), local_state(STATE_SPACE::y)));
            }

            sum_collision_cost += collision_cost * collision_weight_;
        }

        // terminal cost
        const State global_terminal_state = global_state_seq.row(prediction_step_size_ - 1);
        const State local_terminal_state = local_state_seq.row(prediction_step_size_ - 1);
        // const double diff_terminal_speed = reference_speed_ - terminal_state(STATE_SPACE::vel);
        double diff_terminal_ref_path = max_diff_ref_path;
        double diff_terminal_angle = 2.0;
        if (ref_path_map.isInside(grid_map::Position(global_terminal_state(STATE_SPACE::x), global_terminal_state(STATE_SPACE::y)))) {
            diff_terminal_ref_path = ref_path_map.atPosition(
                distance_field_layer_name_, grid_map::Position(global_terminal_state(STATE_SPACE::x), global_terminal_state(STATE_SPACE::y)));
            diff_terminal_angle = global_terminal_state(STATE_SPACE::yaw) -
                                  ref_path_map.atPosition(angle_field_layer_name_, grid_map::Position(global_terminal_state(STATE_SPACE::x),
                                                                                                      global_terminal_state(STATE_SPACE::y)));
            diff_terminal_angle = std::atan2(sin(diff_terminal_angle), cos(diff_terminal_angle));
        }
        // cost += q_terminal_speed_ * diff_terminal_speed * diff_terminal_speed;
        sum_ref_cost += q_terminal_dist_ * diff_terminal_ref_path * diff_terminal_ref_path;
        sum_ref_cost += q_terminal_angle_ * diff_terminal_angle * diff_terminal_angle;

        double collision_cost = 10.0;
        if (local_obstacle_map.isInside(grid_map::Position(local_terminal_state(STATE_SPACE::x), local_terminal_state(STATE_SPACE::y)))) {
            collision_cost = local_obstacle_map.atPosition(
                obstacle_layer_name_, grid_map::Position(local_terminal_state(STATE_SPACE::x), local_terminal_state(STATE_SPACE::y)));
        }

        sum_collision_cost += collision_cost * collision_weight_;

        return std::make_pair(sum_ref_cost + sum_collision_cost, sum_collision_cost);
    }

    std::pair<StateSeq, XYCovMatrices> MPCBase::calc_state_distribution(const StateSeqBatch& state_seq_candidates) const {
        // calc mean state
        StateSeq mean_state_seq = Eigen::MatrixXd::Zero(prediction_step_size_, STATE_SPACE::dim);
        for (size_t i = 0; i < state_seq_candidates.size(); i++) {
            mean_state_seq += state_seq_candidates[i];
        }
        mean_state_seq /= static_cast<double>(state_seq_candidates.size());

        // calc covariance matrices of x and y
        XYCovMatrices xy_cov_matrices =
            std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(prediction_step_size_, Eigen::MatrixXd::Zero(2, 2));
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < state_seq_candidates.size(); i++) {
            for (size_t j = 0; j < prediction_step_size_; j++) {
                Eigen::VectorXd diff = Eigen::VectorXd::Zero(2);
                diff(0) = state_seq_candidates[i](j, STATE_SPACE::x) - mean_state_seq(j, STATE_SPACE::x);
                diff(1) = state_seq_candidates[i](j, STATE_SPACE::y) - mean_state_seq(j, STATE_SPACE::y);
                xy_cov_matrices[j] += diff * diff.transpose();
            }
        }
        for (size_t j = 0; j < prediction_step_size_; j++) {
            xy_cov_matrices[j] /= static_cast<double>(state_seq_candidates.size());
        }

        return std::make_pair(mean_state_seq, xy_cov_matrices);
    }

}  // namespace cpu
}  // namespace mppi