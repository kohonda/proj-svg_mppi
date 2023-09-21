// Kohei Honda, 2023
#pragma once

#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>
#include <utility>
#include "mppi_controller/common.hpp"

namespace mppi {
namespace cpu {
    /*
     * @brief virtual class for MPC
     */
    class MPCTemplate {
    public:
        virtual ~MPCTemplate(){};

        virtual std::pair<ControlSeq, double> solve(const State& initial_state) = 0;

        virtual void set_obstacle_map(const grid_map::GridMap& obstacle_map) = 0;

        virtual void set_reference_map(const grid_map::GridMap& reference_map) = 0;

        virtual ControlSeq get_control_seq() const = 0;

        virtual std::pair<std::vector<StateSeq>, std::vector<double>> get_state_seq_candidates(const int& num_samples) const = 0;

        virtual std::tuple<StateSeq, double, double, double> get_predictive_seq(const State& initial_state,
                                                                                const ControlSeq& control_input_seq) const = 0;

        virtual ControlSeqCovMatrices get_cov_matrices() const = 0;

        virtual std::pair<StateSeq, XYCovMatrices> get_proposed_state_distribution() const = 0;

        std::vector<double> softmax(const std::vector<double>& costs, const double& lambda, const int thread_num) const {
            const double min_cost = *std::min_element(costs.begin(), costs.end());
            double normalization_term = 1e-10;
            for (const auto& cost : costs) {
                normalization_term += std::exp(-(cost - min_cost) / lambda);
            }

            std::vector<double> softmax_costs(costs.size());
#pragma omp parallel for num_threads(thread_num)
            for (size_t i = 0; i < costs.size(); i++) {
                softmax_costs[i] = std::exp(-(costs[i] - min_cost) / lambda) / normalization_term;
            }

            return softmax_costs;
        };
    };

}  // namespace cpu
}  // namespace mppi