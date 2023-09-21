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
#include "mppi_controller/mpc_base.hpp"
#include "mppi_controller/mpc_template.hpp"
#include "mppi_controller/prior_samples_with_costs.hpp"

namespace mppi {
namespace cpu {

    /**
     * @brief Reverse MPPI with sample rejection, by K. Kobayashi, https://arxiv.org/abs/2212.04298
     */
    class ReverseMPPI : public MPCTemplate {
    public:
        ReverseMPPI(const Params::Common& common_params, const Params::ReverseMPPI& reverse_mppi_params);
        ~ReverseMPPI(){};

        /**
         * @brief solve mppi problem and return optimal control sequence
         * @param initial_state initial state
         * @return optimal control sequence and collision rate
         */
        std::pair<ControlSeq, double> solve(const State& initial_state) override;
        /**
         * @brief set obstacle map and reference map
         */
        void set_obstacle_map(const grid_map::GridMap& obstacle_map) override;

        void set_reference_map(const grid_map::GridMap& reference_map) override;

        /**
         * @brief get state sequence candidates and their weights, top num_samples
         * @return std::pair<std::vector<StateSeq>, std::vector<double>> state sequence candidates and their weights
         */
        std::pair<std::vector<StateSeq>, std::vector<double>> get_state_seq_candidates(const int& num_samples) const override;

        std::tuple<StateSeq, double, double, double> get_predictive_seq(const State& initial_state,
                                                                        const ControlSeq& control_input_seq) const override;

        ControlSeqCovMatrices get_cov_matrices() const override;

        ControlSeq get_control_seq() const override;

        std::pair<StateSeq, XYCovMatrices> get_proposed_state_distribution() const override;

    private:
        const size_t prediction_step_size_;  //!< @brief prediction step size
        const int thread_num_;               //!< @brief number of thread for parallel computation

        // MPPI parameters
        const double negative_ratio_;
        const bool is_sample_rejection_;
        const double sample_inflation_ratio_;  // sample inflation for sample rejection
        const int iteration_num_;              //!< @brief number of iteration for MD method
        const double step_size_;               //!< @brief step size for MD method
        const double warm_start_ratio_;        //!< @brief warm start ratio
        const double lambda_;                  //!< @brief temperature parameter [0, inf) of free energy.
        const double alpha_;  //!< @brief weighting parameter [0, 1], which balances control penalties from previous control sequence and nominal
                              //!< control sequence.
        const double non_biased_sampling_rate_;  //!< @brief non-previous-control-input-biased sampling rate [0, 1], Add random noise to control
                                                 //!< sequence with this rate.
        const double steer_cov_;                 //!< @brief covariance of steering angle noise
        // double accel_cov_;                      //!< @brief covariance of acceleration noise

        // Internal variables
        ControlSeq prev_control_seq_;
        ControlSeqCovMatrices prev_covs_;
        ControlSeq prev_rejected_mean_;
        ControlSeqCovMatrices prev_rejected_covs_;
        std::vector<double> weights_ = {};  // for visualization

        // Libraries
        std::unique_ptr<MPCBase> mpc_base_ptr_;
        std::unique_ptr<PriorSamplesWithCosts> inflated_samples_ptr_;
        std::unique_ptr<PriorSamplesWithCosts> prior_samples_ptr_;

    private:
        std::pair<std::vector<double>, std::vector<double>> calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs) const;

        std::pair<ControlSeq, ControlSeqCovMatrices> estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const;

        std::pair<ControlSeq, ControlSeqCovMatrices> weighted_mean_and_sigma(const PriorSamplesWithCosts& samples,
                                                                             const std::vector<double>& weights) const;

        std::pair<ControlSeq, ControlSeqCovMatrices> md_update(const ControlSeq& prior_mean,
                                                               const ControlSeqCovMatrices& prior_covs,
                                                               const ControlSeq& weighted_mean,
                                                               const ControlSeqCovMatrices& weighted_covs,
                                                               const double step_size) const;

        ControlSeq normal_pdf(const ControlSeq& x, const ControlSeq& mean, const ControlSeqCovMatrices& covs) const;

        ControlSeq interpolate(const ControlSeq& x1, const ControlSeq& x2, const double& ratio) const;
        ControlSeqCovMatrices interpolate(const ControlSeqCovMatrices& covs1, const ControlSeqCovMatrices& covs2, const double& ratio) const;
    };

}  // namespace cpu

}  // namespace mppi