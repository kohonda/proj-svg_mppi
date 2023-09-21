#include "mppi_controller/stein_variational_guided_mppi.hpp"

namespace mppi {
namespace cpu {
    SVGuidedMPPI::SVGuidedMPPI(const Params::Common& common_params, const Params::SVGuidedMPPI& svg_mppi_params)
        : prediction_step_size_(static_cast<size_t>(common_params.prediction_step_size)),
          thread_num_(common_params.thread_num),
          lambda_(svg_mppi_params.lambda),
          alpha_(svg_mppi_params.alpha),
          non_biased_sampling_rate_(svg_mppi_params.non_biased_sampling_rate),
          steer_cov_(svg_mppi_params.steer_cov),
          sample_num_for_grad_estimation_(svg_mppi_params.sample_num_for_grad_estimation),
          grad_lambda_(svg_mppi_params.grad_lambda),
          steer_cov_for_grad_estimation_(svg_mppi_params.steer_cov_for_grad_estimation),
          svgd_step_size_(svg_mppi_params.svgd_step_size),
          num_svgd_iteration_(svg_mppi_params.num_svgd_iteration),
          is_use_nominal_solution_(svg_mppi_params.is_use_nominal_solution),
          is_covariance_adaptation_(svg_mppi_params.is_covariance_adaptation),
          gaussian_fitting_lambda_(svg_mppi_params.gaussian_fitting_lambda),
          min_steer_cov_(svg_mppi_params.min_steer_cov),
          max_steer_cov_(svg_mppi_params.max_steer_cov) {
        const size_t sample_batch_num = static_cast<size_t>(svg_mppi_params.sample_batch_num);
        const size_t guide_sample_num = static_cast<size_t>(svg_mppi_params.guide_sample_num);
        const size_t sample_num_for_grad_estimation = static_cast<size_t>(svg_mppi_params.sample_num_for_grad_estimation);

        const size_t sample_num_for_cache = std::max(std::max(sample_batch_num, sample_num_for_grad_estimation), guide_sample_num);
        mpc_base_ptr_ = std::make_unique<MPCBase>(common_params, sample_num_for_cache);

        const double max_steer_angle = common_params.max_steer_angle;
        const double min_steer_angle = common_params.min_steer_angle;
        std::array<double, CONTROL_SPACE::dim> max_control_inputs = {max_steer_angle};
        std::array<double, CONTROL_SPACE::dim> min_control_inputs = {min_steer_angle};
        prior_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(sample_batch_num, prediction_step_size_, max_control_inputs, min_control_inputs,
                                                                     non_biased_sampling_rate_, thread_num_);

        guide_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(guide_sample_num, prediction_step_size_, max_control_inputs, min_control_inputs,
                                                                     non_biased_sampling_rate_, thread_num_);

        prev_control_seq_ = prior_samples_ptr_->get_zero_control_seq();
        nominal_control_seq_ = prior_samples_ptr_->get_zero_control_seq();

        // initialize prior distribution
        const ControlSeqCovMatrices control_seq_cov_matrices = guide_samples_ptr_->get_constant_control_seq_cov_matrices({steer_cov_});
        guide_samples_ptr_->random_sampling(guide_samples_ptr_->get_zero_control_seq(), control_seq_cov_matrices);

        // initialize grad samplers
        for (size_t i = 0; i < sample_batch_num; i++) {
            grad_sampler_ptrs_.emplace_back(std::make_unique<PriorSamplesWithCosts>(sample_num_for_grad_estimation_, prediction_step_size_,
                                                                                    max_control_inputs, min_control_inputs, non_biased_sampling_rate_,
                                                                                    thread_num_, i));
        }
    }

    std::pair<ControlSeq, double> SVGuidedMPPI::solve(const State& initial_state) {
        // Transport guide particles by SVGD
        std::vector<double> costs_history;
        std::vector<ControlSeq> control_seq_history;
        auto func_calc_costs = [&](const PriorSamplesWithCosts& sampler) { return mpc_base_ptr_->calc_sample_costs(sampler, initial_state).first; };
        for (int i = 0; i < num_svgd_iteration_; i++) {
            // Transport samples by stein variational gradient descent
            const ControlSeqBatch grad_log_posterior = approx_grad_posterior_batch(*guide_samples_ptr_, func_calc_costs);
#pragma omp parallel for num_threads(thread_num_)
            for (size_t i = 0; i < guide_samples_ptr_->get_num_samples(); i++) {
                guide_samples_ptr_->noised_control_seq_samples_[i] += svgd_step_size_ * grad_log_posterior[i];
            }

            // store costs and samples for adaptive covariance calculation
            const std::vector<double> costs = mpc_base_ptr_->calc_sample_costs(*guide_samples_ptr_, initial_state).first;
            // guide_samples_ptr_->costs_ = costs;
            // const std::vector<double> cost_with_control_term = guide_samples_ptr_->get_costs_with_control_term(gaussian_fitting_lambda, 0,
            // prior_samples_ptr_->get_zero_control_seq());
            costs_history.insert(costs_history.end(), costs.begin(), costs.end());
            control_seq_history.insert(control_seq_history.end(), guide_samples_ptr_->noised_control_seq_samples_.begin(),
                                       guide_samples_ptr_->noised_control_seq_samples_.end());
        }
        const auto guide_costs = mpc_base_ptr_->calc_sample_costs(*guide_samples_ptr_, initial_state).first;
        const size_t min_idx = std::distance(guide_costs.begin(), std::min_element(guide_costs.begin(), guide_costs.end()));
        const ControlSeq best_particle = guide_samples_ptr_->noised_control_seq_samples_[min_idx];

        // calculate adaptive covariance matrices for prior distribution
        // TODO: Support multiple control input dimensions
        ControlSeqCovMatrices covs = prior_samples_ptr_->get_constant_control_seq_cov_matrices({steer_cov_});
        if (is_covariance_adaptation_) {
            // calculate softmax costs
            const std::vector<double> softmax_costs = softmax(costs_history, gaussian_fitting_lambda_, thread_num_);

            // calculate covariance using gaussian fitting
            for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
                std::vector<double> steer_samples(control_seq_history.size());
                std::vector<double> q_star(softmax_costs.size());
                for (size_t j = 0; j < steer_samples.size(); j++) {
                    steer_samples[j] = control_seq_history[j](i, 0);
                    q_star[j] = softmax_costs[j];
                }
                const double sigma = gaussian_fitting(steer_samples, q_star).second;

                const double sigma_clamped = std::clamp(sigma, min_steer_cov_, max_steer_cov_);

                covs[i] = Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim) * sigma_clamped;
            }
        }

        // random sampling from prior distribution
        prior_samples_ptr_->random_sampling(prev_control_seq_, covs);

        // Rollout samples and calculate costs
        auto [_costs, collision_costs] = mpc_base_ptr_->calc_sample_costs(*prior_samples_ptr_, initial_state);
        prior_samples_ptr_->costs_ = std::forward<std::vector<double>>(_costs);

        // calculate weights
        if (is_use_nominal_solution_) {
            // with nominal sequence
            nominal_control_seq_ = best_particle;
        } else {
            // without nominal sequence
            nominal_control_seq_ = prior_samples_ptr_->get_zero_control_seq();
        }
        const std::vector<double> weights = calc_weights(*prior_samples_ptr_, nominal_control_seq_);
        weights_ = weights;  // for visualization

        // Get control input sequence by weighted average of samples
        ControlSeq updated_control_seq = prior_samples_ptr_->get_zero_control_seq();
        for (size_t i = 0; i < prior_samples_ptr_->get_num_samples(); i++) {
            updated_control_seq += weights[i] * prior_samples_ptr_->noised_control_seq_samples_.at(i);
        }

        const int collision_num = std::count_if(collision_costs.begin(), collision_costs.end(), [](const double& cost) { return cost > 0.0; });
        const double collision_rate = static_cast<double>(collision_num) / static_cast<double>(prior_samples_ptr_->get_num_samples());

        // update previous control sequence for next time step
        prev_control_seq_ = updated_control_seq;

        return std::make_pair(updated_control_seq, collision_rate);
    }

    void SVGuidedMPPI::set_obstacle_map(const grid_map::GridMap& obstacle_map) { mpc_base_ptr_->set_obstacle_map(obstacle_map); };

    void SVGuidedMPPI::set_reference_map(const grid_map::GridMap& reference_map) { mpc_base_ptr_->set_reference_map(reference_map); };

    std::pair<std::vector<StateSeq>, std::vector<double>> SVGuidedMPPI::get_state_seq_candidates(const int& num_samples) const {
        return mpc_base_ptr_->get_state_seq_candidates(num_samples, weights_);
    }

    std::tuple<StateSeq, double, double, double> SVGuidedMPPI::get_predictive_seq(const State& initial_state,
                                                                                  const ControlSeq& control_input_seq) const {
        const auto [prediction_state, state_cost, collision_cost] = mpc_base_ptr_->get_predictive_seq(initial_state, control_input_seq);
        double input_error = 0.0;
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            input_error += (control_input_seq.row(i)).norm();
        }
        return std::make_tuple(prediction_state, state_cost, collision_cost, input_error);
    }

    ControlSeqCovMatrices SVGuidedMPPI::get_cov_matrices() const { return prior_samples_ptr_->get_cov_matrices(); }

    ControlSeq SVGuidedMPPI::get_control_seq() const { return nominal_control_seq_; }

    std::pair<StateSeq, XYCovMatrices> SVGuidedMPPI::get_proposed_state_distribution() const { return mpc_base_ptr_->get_proposed_distribution(); }

    // == private functions ==
    ControlSeq SVGuidedMPPI::approx_grad_log_likelihood(const ControlSeq& mean_seq,
                                                        const ControlSeq& noised_seq,
                                                        const ControlSeqCovMatrices& inv_covs,
                                                        const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
                                                        PriorSamplesWithCosts* sampler) const {
        const ControlSeqCovMatrices grad_cov = sampler->get_constant_control_seq_cov_matrices({steer_cov_for_grad_estimation_});

        // generate gaussian random samples, center of which is input_seq
        sampler->random_sampling(noised_seq, grad_cov);

        // calculate forward simulation and costs
        sampler->costs_ = calc_costs(*sampler);

        // calculate cost with control term
        // const std::vector<double> costs_with_control_term = sampler->get_costs_with_control_term(grad_lambda_, 0.0,
        // sampler->get_zero_control_seq());
        std::vector<double> exp_costs(sampler->get_num_samples());
        ControlSeq sum_of_grads = mean_seq * 0.0;
        const ControlSeqCovMatrices sampler_inv_covs = sampler->get_inv_cov_matrices();
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < sampler->get_num_samples(); i++) {
            double cost_with_control_term = sampler->costs_[i];
            for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
                // const double control_term = grad_lambda_ * prev_control_seq_.row(j) * inv_covs[j] *
                // sampler->noised_control_seq_samples_[i].row(j).transpose();
                const double diff_control_term = grad_lambda_ * (prev_control_seq_.row(j) - sampler->noised_control_seq_samples_[i].row(j)) *
                                                 inv_covs[j] *
                                                 (prev_control_seq_.row(j) - sampler->noised_control_seq_samples_[i].row(j)).transpose();
                // cost_with_control_term += control_term + diff_control_term;
                cost_with_control_term += diff_control_term;
            }
            const double exp_cost = std::exp(-cost_with_control_term / grad_lambda_);
            exp_costs[i] = exp_cost;

            ControlSeq grad_log_gaussian = mean_seq * 0.0;
            for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
                grad_log_gaussian.row(j) = exp_cost * sampler_inv_covs[j] * (sampler->noised_control_seq_samples_[i] - noised_seq).row(j).transpose();
            }
            sum_of_grads += grad_log_gaussian;
        }
        const double sum_of_costs = std::accumulate(exp_costs.begin(), exp_costs.end(), 0.0);
        return sum_of_grads / (sum_of_costs + 1e-10);
    }

    ControlSeqBatch SVGuidedMPPI::approx_grad_posterior_batch(
        const PriorSamplesWithCosts& samples,
        const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs) const {
        ControlSeqBatch grad_log_likelihoods = samples.get_zero_control_seq_batch();
        const ControlSeq mean = samples.get_mean();
        // #pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < samples.get_num_samples(); i++) {
            const ControlSeq grad_log_likelihood = approx_grad_log_likelihood(
                mean, samples.noised_control_seq_samples_[i], samples.get_inv_cov_matrices(), calc_costs, grad_sampler_ptrs_.at(i).get());
            grad_log_likelihoods[i] = grad_log_likelihood;
        }

        return grad_log_likelihoods;
    }

    std::pair<ControlSeq, ControlSeqCovMatrices> SVGuidedMPPI::weighted_mean_and_sigma(const PriorSamplesWithCosts& samples,
                                                                                       const std::vector<double>& weights) const {
        ControlSeq mean = samples.get_zero_control_seq();
        ControlSeqCovMatrices sigma = samples.get_zero_control_seq_cov_matrices();

        const ControlSeq prior_mean = samples.get_mean();
        const ControlSeqCovMatrices prior_inv_covs = samples.get_inv_cov_matrices();
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < samples.get_num_samples(); i++) {
            mean += weights[i] * samples.noised_control_seq_samples_[i];

            const ControlSeq diff = samples.noised_control_seq_samples_[i] - prior_mean;
            for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
                sigma[j] += weights[i] * diff.row(j).transpose() * prior_inv_covs[j] * diff.row(j);
            }
        }

        return std::make_pair(mean, sigma);
    }

    std::pair<ControlSeq, ControlSeqCovMatrices> SVGuidedMPPI::estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const {
        ControlSeq mu = samples.get_zero_control_seq();
        ControlSeqCovMatrices sigma = samples.get_zero_control_seq_cov_matrices();

// calculate mean
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < samples.get_num_samples(); i++) {
            mu += samples.noised_control_seq_samples_[i];
        }
        mu /= static_cast<double>(samples.get_num_samples());

#pragma omp parallel for num_threads(thread_num_)
        // calculate covariance matrices
        for (size_t i = 0; i < samples.get_num_samples(); i++) {
            for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
                sigma[j] += (samples.noised_control_seq_samples_[i].row(j) - mu.row(j)).transpose() *
                            (samples.noised_control_seq_samples_[i].row(j) - mu.row(j));
            }
        }

        for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
            sigma[j] /= static_cast<double>(samples.get_num_samples());

            // add small value to avoid singular matrix
            sigma[j] += 1e-5 * Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim);
        }

        return std::make_pair(mu, sigma);
    }

    std::vector<double> SVGuidedMPPI::calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs,
                                                   const ControlSeq& nominal_control_seq) const {
        // calculate costs with control term
        const std::vector<double> costs_with_control_term =
            prior_samples_with_costs.get_costs_with_control_term(lambda_, alpha_, nominal_control_seq);

        // softmax weights
        return softmax(costs_with_control_term, lambda_, thread_num_);
    }

    // Gao's Algorithm
    // H. GUO, “A Simple Algorithm for Fitting a Gaussian Function,” IEEE Signal Process, Mag.28, No. 5 (2011), 134–137.
    std::pair<double, double> SVGuidedMPPI::gaussian_fitting(const std::vector<double>& x, const std::vector<double>& y) const {
        assert(x.size() == y.size());

        // Should y is larger than 0 for log function
        std::vector<double> y_hat(y.size(), 0.0);
        std::transform(y.begin(), y.end(), y_hat.begin(), [](double y) { return std::max(y, 1e-10); });

        // const double epsilon = 1e-8;
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < x.size(); i++) {
            const double y_hat_2 = y_hat[i] * y_hat[i];
            const double y_hat_log = std::log(y_hat[i]);

            A(0, 0) += y_hat_2;
            A(0, 1) += y_hat_2 * x[i];
            A(0, 2) += y_hat_2 * x[i] * x[i];

            A(1, 0) += y_hat_2 * x[i];
            A(1, 1) += y_hat_2 * x[i] * x[i];
            A(1, 2) += y_hat_2 * x[i] * x[i] * x[i];

            A(2, 0) += y_hat_2 * x[i] * x[i];
            A(2, 1) += y_hat_2 * x[i] * x[i] * x[i];
            A(2, 2) += y_hat_2 * x[i] * x[i] * x[i] * x[i];

            b(0) += y_hat_2 * y_hat_log;
            b(1) += y_hat_2 * x[i] * y_hat_log;
            b(2) += y_hat_2 * x[i] * x[i] * y_hat_log;
        }

        // solve Au = b
        const Eigen::Vector3d u = A.colPivHouseholderQr().solve(b);

        // calculate mean and variance

        // original
        // const double mean = -u(1) / (2.0 * u(2));
        // const double variance = std::sqrt(-1.0 / (2.0 * u(2)));

        // To avoid nan;
        const double eps = 1e-5;
        const double mean = -u(1) / (2.0 * std::min(u(2), -eps));
        // const double variance = std::sqrt(1.0 / (2.0 * std::abs(std::min(u(2), -eps))));
        const double variance = std::sqrt(1.0 / (2.0 * std::abs(u(2))));

        return std::make_pair(mean, variance);
    }

}  // namespace cpu
}  // namespace mppi
