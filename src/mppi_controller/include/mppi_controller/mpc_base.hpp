// Kohei Honda, 2023

#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <array>
#include <grid_map_core/GridMap.hpp>
#include <memory>
#include <utility>

#include "mppi_controller/common.hpp"
#include "mppi_controller/prior_samples_with_costs.hpp"

namespace mppi {
namespace cpu {
    class MPCBase {
    public:
        MPCBase(const Params::Common& params, const size_t& sample_num);
        ~MPCBase(){};
        enum class SpeedPredictionMode {
            CONSTANT,
            LINEAR,
            REFERENCE,
        };

        void set_obstacle_map(const grid_map::GridMap& obstacle_map);

        void set_reference_map(const grid_map::GridMap& reference_map);

        std::pair<std::vector<double>, std::vector<double>> calc_sample_costs(const PriorSamplesWithCosts& sampler, const State& init_state);

        std::tuple<StateSeq, double, double> get_predictive_seq(const State& initial_state, const ControlSeq& control_input_seq) const;

        std::pair<std::vector<StateSeq>, std::vector<double>> get_state_seq_candidates(const int& num_samples,
                                                                                       const std::vector<double>& weights) const;

        std::pair<StateSeq, XYCovMatrices> get_proposed_distribution() const;

    private:
        std::pair<std::vector<double>, std::vector<double>> calc_sample_costs(const PriorSamplesWithCosts& sampler,
                                                                              const State& global_init_state,
                                                                              const grid_map::GridMap& obstacle_map,
                                                                              const grid_map::GridMap& reference_map,
                                                                              StateSeqBatch* global_state_seq_candidates,
                                                                              StateSeqBatch* local_state_seq_candidates) const;
        std::pair<std::vector<double>, std::vector<double>> calc_sample_costs(const PriorSamplesWithCosts& sampler,
                                                                              const State& local_init_state,
                                                                              const grid_map::GridMap& obstacle_map,
                                                                              StateSeqBatch* local_state_seq_candidates) const;

        StateSeq predict_state_seq(const ControlSeq& control_seq, const State& init_state, const grid_map::GridMap& reference_map) const;
        void predict_state_seq(const ControlSeq& control_seq,
                               const State& global_init_state,
                               const grid_map::GridMap& reference_map,
                               StateSeq* global_state_seq,
                               StateSeq* local_state_seq) const;

        double constant_speed_prediction(const double& current_speed) const;
        double linear_speed_prediction(const double& current_speed,
                                       const double& target_speed,
                                       const double& prediction_interval,
                                       const double& min_accel,
                                       const double& max_accel) const;
        double reference_speed_prediction(const double& pos_x, const double& pos_y, const grid_map::GridMap& reference_map) const;

        std::pair<double, double> state_cost(const StateSeq& local_state_seq, const grid_map::GridMap& obstacle_map) const;
        std::pair<double, double> state_cost(const StateSeq& global_state_seq,
                                             const StateSeq& local_base_state_seq,
                                             const grid_map::GridMap& obstacle_map,
                                             const grid_map::GridMap& ref_path_map) const;

        std::pair<StateSeq, XYCovMatrices> calc_state_distribution(const StateSeqBatch& state_seq_candidates) const;

    private:
        // == Constant parameters ==
        const std::string obstacle_layer_name_ = "collision_layer";
        const std::string distance_field_layer_name_ = "distance_field";
        const std::string angle_field_layer_name_ = "angle_field";
        const std::string speed_field_layer_name_ = "speed_field";

        const bool is_localize_less_mode_;
        const int thread_num_;  //!< @brief number of thread for parallel computation
        const size_t prediction_step_size_;
        const double prediction_interval_;  //!< @brief prediction interval [s]
        const double reference_speed_;      //!< @brief robot reference speed [m/s] Only used when speed_prediction_mode_ == CONST
        const double lr_;
        const double lf_;
        SpeedPredictionMode speed_prediction_mode_ = SpeedPredictionMode::LINEAR;
        const double max_accel_;  //!< @brief maximum acceleration [m/s^2]
        const double min_accel_;
        const double steer_delay_;  //!< @brief dead_time [s]
        const size_t steer_delay_steps_;
        const double steer_delay_tau_;  //!< @brief time delay tau [s]
        const double q_dist_;
        const double q_angle_;
        // const double q_speed_;
        const double collision_weight_;
        const double q_terminal_dist_;
        const double q_terminal_angle_;
        // const double q_terminal_speed_;

        // == Inner-variables ==
        grid_map::GridMap obstacle_map_;
        grid_map::GridMap reference_map_;
        // To reduce memory allocation and visualize candidates paths
        StateSeqBatch global_state_seq_candidates_ = {};
        StateSeqBatch local_state_seq_candidates_ = {};
    };

}  // namespace cpu
}  // namespace mppi
