// Kohei Honda, 2023

#pragma once
#include <Eigen/Dense>
#include <array>
#include <iostream>
#include <memory>
#include <random>
#include <vector>
#include "mppi_controller/common.hpp"

namespace mppi {
namespace cpu {

    class PriorSamplesWithCosts {
    private:
        const int thread_num_;
        const size_t num_samples_;
        const size_t prediction_horizon_;

        // For random noise generation
        std::vector<std::mt19937> rngs_;
        std::unique_ptr<std::vector<std::array<std::normal_distribution<>, CONTROL_SPACE::dim>>> normal_dists_ptr_;
        const double non_biased_sampling_rate_;
        const std::array<double, CONTROL_SPACE::dim> max_control_inputs_;
        const std::array<double, CONTROL_SPACE::dim> min_control_inputs_;
        std::discrete_distribution<> discrete_dist_;

        ControlSeq control_seq_mean_;
        ControlSeqCovMatrices control_seq_cov_matrices_;
        ControlSeqCovMatrices control_seq_inv_cov_matrices_;

    public:
        ControlSeqBatch noise_seq_samples_;
        ControlSeqBatch noised_control_seq_samples_;
        std::vector<double> costs_;

    public:
        PriorSamplesWithCosts(const size_t& num_samples,
                              const size_t& prediction_horizon,
                              const std::array<double, CONTROL_SPACE::dim>& max_control_inputs,
                              const std::array<double, CONTROL_SPACE::dim>& min_control_inputs,
                              const double& non_biased_sampling_rate,
                              const int& thread_num,
                              const int& seed = 42);

        ~PriorSamplesWithCosts() = default;

        void random_sampling(const ControlSeq& control_seq_mean, const ControlSeqCovMatrices& control_seq_cov_matrices);

        std::vector<int> random_sample_choice(const size_t& num_samples, const std::vector<double>& probabilities);

        std::vector<double> get_costs_with_control_term(const double& lambda, const double& alpha, const ControlSeq& nominal_control_seq) const;

        /* Setters */
        void shrink_copy_from(const PriorSamplesWithCosts& source_samples, const std::vector<int>& indices);

        void inflate_copy_from(const PriorSamplesWithCosts& source_samples, const ControlSeqCovMatrices& noise_covs);

        void set_control_seq_mean(const ControlSeq& control_seq_mean);

        void set_control_seq_cov_matrices(const ControlSeqCovMatrices& control_seq_cov_matrices);

        /* Getters */
        size_t get_num_samples() const { return num_samples_; }

        size_t get_prediction_horizon() const { return prediction_horizon_; }

        ControlSeq get_zero_control_seq() const { return Eigen::MatrixXd::Zero(prediction_horizon_ - 1, CONTROL_SPACE::dim); }

        ControlSeq get_constant_control_seq(const std::array<double, CONTROL_SPACE::dim>& value) const {
            ControlSeq control_seq = get_zero_control_seq();
            for (size_t i = 0; i < prediction_horizon_ - 1; i++) {
                for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
                    control_seq(i, j) = value[j];
                }
            }
            return control_seq;
        }

        ControlSeqBatch get_zero_control_seq_batch() const {
            return std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
                num_samples_, Eigen::MatrixXd::Zero(prediction_horizon_ - 1, CONTROL_SPACE::dim));
        }

        StateSeq get_zero_state_seq() const { return Eigen::MatrixXd::Zero(prediction_horizon_, STATE_SPACE::dim); }

        StateSeqBatch get_zero_state_seq_batch() const {
            return std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
                num_samples_, Eigen::MatrixXd::Zero(prediction_horizon_, STATE_SPACE::dim));
        }

        ControlSeqCovMatrices get_zero_control_seq_cov_matrices() const {
            return std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
                prediction_horizon_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
        }

        ControlSeqCovMatrices get_constant_control_seq_cov_matrices(const std::array<double, CONTROL_SPACE::dim>& diag) const {
            ControlSeqCovMatrices control_seq_covs = get_zero_control_seq_cov_matrices();
            for (auto& cov : control_seq_covs) {
                for (size_t i = 0; i < CONTROL_SPACE::dim; i++) {
                    cov(i, i) = diag[i];
                }
            }
            return control_seq_covs;
        }

        ControlSeq get_mean() const { return control_seq_mean_; }

        ControlSeqCovMatrices get_cov_matrices() const { return control_seq_cov_matrices_; }

        ControlSeqCovMatrices get_inv_cov_matrices() const { return control_seq_inv_cov_matrices_; }

        ControlSeq clipped_control_seq(const ControlSeq& control_seq) const {
            ControlSeq clipped_control_seq = control_seq;
            for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
                for (size_t k = 0; k < prediction_horizon_ - 1; k++) {
                    clipped_control_seq(k, j) = std::clamp(control_seq(k, j), min_control_inputs_[j], max_control_inputs_[j]);
                }
            }
            return clipped_control_seq;
        }
    };
}  // namespace cpu
}  // namespace mppi