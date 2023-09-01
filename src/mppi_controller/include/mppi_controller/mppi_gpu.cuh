// Kohei Honda, 2023

#pragma once
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <vector>
#include <algorithm>
#include <random>
#include <array>
#include <utility>
#include <Eigen/Dense>

#include <cublas_v2.h>
#include <cuda.h>
#include <curand.h>

#include <thrust/device_new.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <grid_map_core/GridMap.hpp>

#include "mppi_controller/common.hpp"


namespace mppi
{
    namespace cuda
    {

        class MPPIGPU
        {
        public:
            // using State = Eigen::Matrix<double, STATE_SPACE::dim, 1>;
            // using Control = Eigen::Matrix<double, CONTROL_SPACE::dim, 1>;
            // using StateSeq = Eigen::Matrix<double, PREDICTION_HORIZON, STATE_SPACE::dim>;
            // using ControlSeq = Eigen::Matrix<double, PREDICTION_HORIZON - 1, CONTROL_SPACE::dim>;
            // using StateSeqBatch = std::array<StateSeq, RANDOM_SAMPLES>;
            // using ControlSeqBatch = std::array<ControlSeq, RANDOM_SAMPLES>;
            // using ControlCov = Eigen::Matrix<double, CONTROL_SPACE::dim, CONTROL_SPACE::dim>;
            // using Weights = Eigen::Matrix<double, RANDOM_SAMPLES, 1>;

            using State = Eigen::Matrix<double, STATE_SPACE::dim, 1>;
            using Control = Eigen::Matrix<double, CONTROL_SPACE::dim, 1>;
            using StateSeq = thrust::device_vector<State, thrust::device_allocator<State>>;
            using ControlSeq = thrust::device_vector<Control, thrust::device_allocator<Control>>;
            using StateSeqBatch = thrust::device_vector<StateSeq, thrust::device_allocator<StateSeq>>;
            using ControlSeqBatch = thrust::device_vector<ControlSeq, thrust::device_allocator<ControlSeq>>;

            MPPIGPU(const Params &params);
            ~MPPIGPU();

            /**
             * @brief solve mppi problem and return optimal control sequence
             */
            std::array<double, CONTROL_SPACE::dim> solve(const std::array<double, STATE_SPACE::dim> &initial_state);

            void set_params(const Params &params)
            {
                thread_num_ = params.thread_num;
                prediction_interval_ = params.prediction_interval;
                reference_speed_ = params.reference_speed;
                max_steer_angle_ = params.max_steer_angle;
                min_steer_angle_ = params.min_steer_angle;
                max_accel_ = params.max_accel;
                min_accel_ = params.min_accel;
                lr_ = params.lr;
                lf_ = params.lf;
                q_dist_ = params.q_dist;
                q_angle_ = params.q_angle;
                q_speed_ = params.q_speed;
                collision_weight_ = params.collision_weight;
                q_terminal_dist_ = params.q_terminal_dist;
                q_terminal_angle_ = params.q_terminal_angle;
                q_terminal_speed_ = params.q_terminal_speed;

                lambda_ = params.lambda;
                alpha_ = params.alpha;
                steer_cov_ = params.steer_cov;
                accel_cov_ = params.accel_cov;
                // TODO: 
                // control_cov_(CONTROL_SPACE::steer, CONTROL_SPACE::steer) = steer_cov_;
                // control_cov_(CONTROL_SPACE::accel, CONTROL_SPACE::accel) = accel_cov_;

            };

            /**
             * @brief set obstacle map and reference map
             */
            void set_obstacle_map(const grid_map::GridMap &obstacle_map)
            {
                // obstacle_map_ = obstacle_map;
            };

            void set_reference_map(const grid_map::GridMap &reference_map)
            {
                // reference_map_ = reference_map;
            };

            /**
             * @brief get state sequence candidates and their weights, top num_samples
             * @return std::pair<std::vector<StateSeq>, std::vector<double>> state sequence candidates and their weights
             */
            // std::pair<std::vector<StateSeq>, std::vector<double>> get_state_seq_candidates(const int &num_samples) const;

            // StateSeq get_predictive_seq(const State &initial_state) const;

            // Private parameters
        private:
            const std::string obstacle_layer_name_ = "obstacle_cost";
            const std::string distance_field_layer_name_ = "distance_field";
            const std::string angle_field_layer_name_ = "angle_field";
            
            // random noise
            curandGenerator_t rng_;
            

            // Parameters
            double reference_speed_ = 5.0;     //!< @brief robot reference speed [m/s] TODO: あとでpathから得るようにする
            double prediction_interval_ = 0.1; //!< @brief prediction interval [s]
            int thread_num_ = 8;               //!< @brief number of thread for parallel computation
            double max_steer_angle_ = 3.0;     //!< @brief maximum steering angle [rad]
            double min_steer_angle_ = -3.0;
            double max_accel_ = 9.5; //!< @brief maximum acceleration [m/s^2]
            double min_accel_ = -2.0;
            double lr_ = 0.17145;
            double lf_ = 0.15875;

            double q_dist_ = 1.0;
            double q_angle_ = 0.0;
            double q_speed_ = 10.0;
            double collision_weight_ = 0.01;
            double q_terminal_dist_ = 1.0;
            double q_terminal_angle_ = 0.0;
            double q_terminal_speed_ = 10.0;

            // MPPI parameters
            double lambda_ = 10; //!< @brief temperature parameter [0, inf) of free energy.
            // temperature parameter is a balancing term between control cost and state cost.
            // If lambda_ is small, weighted state cost is dominant, thus control sequence is more aggressive.
            // If lambda_ is large, weighted control cost is dominant, thus control sequence is more smooth.
            double alpha_ = 0.1;                          //!< @brief weighting parameter [0, 1], which balances control penalties from previous control sequence and nominal control sequence.
            double steer_cov_ = 0.5;                      //!< @brief covariance of steering angle noise
            double accel_cov_ = 0.5;                      //!< @brief covariance of acceleration noise
            // ControlCov control_cov_ = ControlCov::Zero(); //!< @brief covariance matrix of control noise
            // These sigmas determines how much the control sequence update from previous one.

        private:
            // Internal variables
            // std::unique_ptr<ControlSeq> prev_control_seq_ptr_;
            // std::unique_ptr<ControlSeqBatch> noised_control_seq_batch_ptr_;
            // std::unique_ptr<ControlSeqBatch> noise_seq_batch_ptr_;
            ControlSeq prev_control_seq_;
            ControlSeqBatch noised_control_seq_batch_;
            ControlSeqBatch noise_seq_batch_;

            // ControlSeq prev_control_seq_ = ControlSeq::Zero();
            // ControlSeqBatch noised_control_seq_batch_ = {};
            // ControlSeqBatch noise_seq_batch_ = {};

            // StateSeqBatch state_seq_candidates_ = {};
            // Weights cost_batch_ = Weights::Zero();
            // Weights weights_ = Weights::Zero();

            // grid_map::GridMap obstacle_map_;
            // grid_map::GridMap reference_map_;

        private:
            void generate_noise_seq_batch(const ControlSeq &prev_control_seq, ControlSeqBatch *noised_control_seq_batch, ControlSeqBatch *noise_seq_batch);
            ControlSeqBatch generate_noise_seq_batch(const ControlSeq &prev_control_seq) const;


            // void predict_state_seq_batch(const ControlSeqBatch &control_seq_batch, const State &init_state, StateSeqBatch *state_seq_batch) const;

            // StateSeq predict_state_seq(const ControlSeq &control_seq, const State &init_state) const;

            // void calc_state_cost_batch(const StateSeqBatch &state_seq_batch, const grid_map::GridMap &obstacle_map, const grid_map::GridMap &ref_path_map, Weights *cost_batch) const;

            // double state_cost(const StateSeq &state_seq, const grid_map::GridMap &obstacle_map, const grid_map::GridMap &ref_path_map) const;

            // void calc_weights_mppi(const Eigen::Matrix<double, RANDOM_SAMPLES, 1> &costs, const ControlSeqBatch &noised_control_seq_batch, const ControlSeq &prev_control_seq, Weights *weights) const;
        };

    } // namespace cuda

} // namespace mppi