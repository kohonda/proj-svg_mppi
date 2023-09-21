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
     * @brief Model Predictive Path Integral Control, G. Williams, https://arxiv.org/abs/1707.02342
     */
    class ForwardMPPI : public MPCTemplate {
    public:
        ForwardMPPI(const Params::Common& common_params, const Params::ForwardMPPI& forward_mppi_params);
        ~ForwardMPPI(){};

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
        const double lambda_;  //!< @brief temperature parameter [0, inf) of free energy.
        // temperature parameter is a balancing term between control cost and state cost.
        // If lambda_ is small, weighted state cost is dominant, thus control sequence is more aggressive.
        // If lambda_ is large, weighted control cost is dominant, thus control sequence is more smooth.
        const double alpha_;  //!< @brief weighting parameter [0, 1], which balances control penalties from previous control sequence and nominal
                              //!< control sequence.
        const double non_biased_sampling_rate_;  //!< @brief non-previous-control-input-biased sampling rate [0, 1], Add random noise to control
                                                 //!< sequence with this rate.
        const double steer_cov_;                 //!< @brief covariance of steering angle noise
        // double accel_cov_;                      //!< @brief covariance of acceleration noise

        // Parameters for estimating gradient of R-KLD
        const int num_itr_for_grad_estimation_;
        const double step_size_for_grad_estimation_;
        const size_t sample_num_for_grad_estimation_;
        const double steer_cov_for_grad_estimation_;

        // Internal variables
        ControlSeq prev_control_seq_;
        std::vector<double> weights_ = {};  // for visualization

        // Libraries
        std::unique_ptr<MPCBase> mpc_base_ptr_;
        std::unique_ptr<PriorSamplesWithCosts> prior_samples_ptr_;
        std::vector<std::unique_ptr<PriorSamplesWithCosts>> grad_sampler_ptrs_;

    private:
        std::vector<double> calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs) const;

        std::vector<int> get_top_indices(const std::vector<double>& values, const int& num) const;

        ControlSeq grad_reverse_kld(const ControlSeq& mean_seq,
                                    const ControlSeq& noised_seq,
                                    const ControlSeqCovMatrices& inv_covs,
                                    const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
                                    PriorSamplesWithCosts* sampler) const;

        ControlSeqBatch grad_reverse_kld_batch(const PriorSamplesWithCosts& samples,
                                               const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs) const;

        ControlSeqBatch transport_samples(const PriorSamplesWithCosts& samples_with_cost,
                                          const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
                                          const int& num_itr,
                                          const double& step_size);

        ControlSeqBatch transport_samples(const PriorSamplesWithCosts& samples,
                                          const ControlSeqBatch& grad_kld_batch,
                                          const int& num_itr,
                                          const double& step_size) const;

        std::pair<ControlSeq, ControlSeqCovMatrices> estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const;
    };

}  // namespace cpu

}  // namespace mppi