#include "mppi_controller/stein_variational_mpc.hpp"

namespace mppi {
namespace cpu {
    SteinVariationalMPC::SteinVariationalMPC(const Params::Common& common_params, const Params::SteinVariationalMPC& sv_mpc_params)
        : prediction_step_size_(static_cast<size_t>(common_params.prediction_step_size)),
          thread_num_(common_params.thread_num),
          lambda_(sv_mpc_params.lambda),
          alpha_(sv_mpc_params.alpha),
          non_biased_sampling_rate_(sv_mpc_params.non_biased_sampling_rate),
          steer_cov_(sv_mpc_params.steer_cov),
          num_svgd_iteration_(sv_mpc_params.num_svgd_iteration),
          steer_cov_for_grad_estimation_(sv_mpc_params.steer_cov_for_grad_estimation),
          svgd_step_size_(sv_mpc_params.svgd_step_size),
          is_max_posterior_estimation_(sv_mpc_params.is_max_posterior_estimation) {
        const size_t sample_batch_num = static_cast<size_t>(sv_mpc_params.sample_batch_num);
        const size_t sample_num_for_grad_estimation = static_cast<size_t>(sv_mpc_params.sample_num_for_grad_estimation);

        const size_t sample_num_for_cache = std::max(sample_batch_num, sample_num_for_grad_estimation);
        mpc_base_ptr_ = std::make_unique<MPCBase>(common_params, sample_num_for_cache);

        const double max_steer_angle = common_params.max_steer_angle;
        const double min_steer_angle = common_params.min_steer_angle;
        std::array<double, CONTROL_SPACE::dim> max_control_inputs = {max_steer_angle};
        std::array<double, CONTROL_SPACE::dim> min_control_inputs = {min_steer_angle};
        prior_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(sample_batch_num, prediction_step_size_, max_control_inputs, min_control_inputs,
                                                                     non_biased_sampling_rate_, thread_num_);
        prev_control_seq_ = prior_samples_ptr_->get_zero_control_seq();

        // initialize prior distribution
        const std::array<double, CONTROL_SPACE::dim> control_cov_diag = {steer_cov_};
        ControlSeqCovMatrices control_seq_cov_matrices = prior_samples_ptr_->get_constant_control_seq_cov_matrices(control_cov_diag);
        prior_samples_ptr_->random_sampling(prior_samples_ptr_->get_zero_control_seq(), control_seq_cov_matrices);

        // initialize grad samplers
        for (size_t i = 0; i < sample_batch_num; i++) {
            grad_sampler_ptrs_.emplace_back(std::make_unique<PriorSamplesWithCosts>(
                sample_num_for_grad_estimation, prediction_step_size_, max_control_inputs, min_control_inputs, non_biased_sampling_rate_, 1, i));
        }
    }

    std::pair<ControlSeq, double> SteinVariationalMPC::solve(const State& initial_state) {
        // calculate gradient of log posterior (= approx grad log_likelihood estimated by monte carlo method + grad log prior)
        // NOTE: currently, prior is approximated as normal distribution
        auto func_calc_costs = [&](const PriorSamplesWithCosts& sampler) {
            const auto [costs, _flags] = mpc_base_ptr_->calc_sample_costs(sampler, initial_state);
            return costs;
        };

        for (int i = 0; i < num_svgd_iteration_; i++) {
            const ControlSeqBatch grad_log_posterior = approx_grad_posterior_batch(*prior_samples_ptr_, func_calc_costs);

            // Transport samples by stein variational gradient descent
            const ControlSeqBatch phis = phi_batch(*prior_samples_ptr_, grad_log_posterior);
#pragma omp parallel for num_threads(thread_num_)
            for (size_t i = 0; i < prior_samples_ptr_->get_num_samples(); i++) {
                prior_samples_ptr_->noised_control_seq_samples_[i] += svgd_step_size_ * phis[i];
            }
        }

        // calculate weights for shifting prior distribution
        auto [_costs, collision_costs] = mpc_base_ptr_->calc_sample_costs(*prior_samples_ptr_, initial_state);
        prior_samples_ptr_->costs_ = std::forward<std::vector<double>>(_costs);
        const std::vector<double> weights = calc_weights(*prior_samples_ptr_);

        const int collision_num = std::count_if(collision_costs.begin(), collision_costs.end(), [](const double& cost) { return cost > 0.0; });
        const double collision_rate = static_cast<double>(collision_num) / static_cast<double>(prior_samples_ptr_->get_num_samples());

        // NAN and inf check for weights
        auto has_nan_or_inf = [](const double& x) { return std::isnan(x) || std::isinf(x); };
        const bool is_valid = std::none_of(weights.begin(), weights.end(), has_nan_or_inf);

        if (is_valid) {
            weights_ = weights;  // for visualization

            ControlSeq updated_control_seq = prior_samples_ptr_->get_zero_control_seq();
            if (is_max_posterior_estimation_) {
                // max posterior
                const int max_idx = std::distance(weights.begin(), std::max_element(weights.begin(), weights.end()));
                updated_control_seq = prior_samples_ptr_->noised_control_seq_samples_.at(max_idx);
            } else {
                // Get control input sequence by weighted average of samples
                for (size_t i = 0; i < prior_samples_ptr_->get_num_samples(); i++) {
                    updated_control_seq += weights[i] * prior_samples_ptr_->noised_control_seq_samples_.at(i);
                }
            }

            // shift particles
            // This cause unstable behavior
            // #pragma omp parallel for num_threads(thread_num_)
            //                 for (size_t i = 0; i < prior_samples_ptr_->get_num_samples(); i++)
            //                 {
            //                     // shift control sequence by 1 step
            //                     ControlSeq tmp = prior_samples_ptr_->get_zero_control_seq();
            //                     for (size_t j = 0; j < prediction_step_size_ - 2; j++)
            //                     {
            //                         tmp.row(j) = prior_samples_ptr_->noised_control_seq_samples_.at(i).row(j + 1);
            //                     }
            //                     tmp.row(prediction_step_size_ - 1) = updated_control_seq.row(prediction_step_size_ - 2);
            //                     prior_samples_ptr_->noised_control_seq_samples_.at(i) = tmp;
            //                 }

            // update prior distribution
            const auto [new_mean, new_cov] = weighted_mean_and_sigma(*prior_samples_ptr_, weights);
            prior_samples_ptr_->set_control_seq_mean(new_mean);
            prior_samples_ptr_->set_control_seq_cov_matrices(new_cov);

            prev_control_seq_ = updated_control_seq;
            return std::make_pair(updated_control_seq, collision_rate);
        } else {
            // std::cout << "NAN or INF is detected in control sequence. Resampling..." << std::endl;
            prior_samples_ptr_->random_sampling(prior_samples_ptr_->get_zero_control_seq(),
                                                prior_samples_ptr_->get_constant_control_seq_cov_matrices({steer_cov_}));

            return std::make_pair(prev_control_seq_, collision_rate);
        }
    }

    void SteinVariationalMPC::set_obstacle_map(const grid_map::GridMap& obstacle_map) { mpc_base_ptr_->set_obstacle_map(obstacle_map); };

    void SteinVariationalMPC::set_reference_map(const grid_map::GridMap& reference_map) { mpc_base_ptr_->set_reference_map(reference_map); };

    std::pair<std::vector<StateSeq>, std::vector<double>> SteinVariationalMPC::get_state_seq_candidates(const int& num_samples) const {
        return mpc_base_ptr_->get_state_seq_candidates(num_samples, weights_);
    }

    std::tuple<StateSeq, double, double, double> SteinVariationalMPC::get_predictive_seq(const State& initial_state,
                                                                                         const ControlSeq& control_input_seq) const {
        const auto [prediction_state, state_cost, collision_cost] = mpc_base_ptr_->get_predictive_seq(initial_state, control_input_seq);
        double input_error = 0.0;
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            input_error += (control_input_seq.row(i)).norm();
        }
        return std::make_tuple(prediction_state, state_cost, collision_cost, input_error);
    }

    ControlSeqCovMatrices SteinVariationalMPC::get_cov_matrices() const {
        const auto [_mean, cov] = estimate_mu_and_sigma(*prior_samples_ptr_);
        return cov;
    }

    ControlSeq SteinVariationalMPC::get_control_seq() const { return prev_control_seq_; }

    std::pair<StateSeq, XYCovMatrices> SteinVariationalMPC::get_proposed_state_distribution() const {
        return mpc_base_ptr_->get_proposed_distribution();
    }

    // == private functions ==
    ControlSeq SteinVariationalMPC::approx_grad_log_likelihood(const ControlSeq& mean_seq,
                                                               const ControlSeq& noised_seq,
                                                               const ControlSeqCovMatrices& inv_covs,
                                                               const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
                                                               PriorSamplesWithCosts* sampler) const {
        const ControlSeqCovMatrices grad_cov = sampler->get_constant_control_seq_cov_matrices({steer_cov_for_grad_estimation_});

        // generate gaussian random samples, center of which is input_seq
        sampler->random_sampling(noised_seq, grad_cov);

        // calculate forward simulation and costs
        sampler->costs_ = calc_costs(*sampler);

        double sum_of_costs = 0.0;
        ControlSeq sum_of_grads = mean_seq * 0.0;
        const ControlSeqCovMatrices sampler_inv_covs = sampler->get_inv_cov_matrices();
        for (size_t i = 0; i < sampler->get_num_samples(); i++) {
            double cost_with_control_term = (sampler->costs_[i]) / lambda_;
            for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
                const double control_term = mean_seq.row(j) * inv_covs[j] * sampler->noised_control_seq_samples_[i].row(j).transpose();
                cost_with_control_term += control_term;
            }
            const double exp_cost = std::exp(-cost_with_control_term);
            sum_of_costs += exp_cost;

            ControlSeq grad_log_gaussian = mean_seq * 0.0;
            for (size_t j = 0; j < prediction_step_size_ - 1; j++) {
                grad_log_gaussian.row(j) = exp_cost * sampler_inv_covs[j] * (sampler->noised_control_seq_samples_[i] - noised_seq).row(j).transpose();
            }
            sum_of_grads += grad_log_gaussian;
        }

        return sum_of_grads / (sum_of_costs + 1e-10);
    }

    ControlSeq SteinVariationalMPC::grad_log_normal_dist(const ControlSeq& sample,
                                                         const ControlSeq& prior_mean,
                                                         const ControlSeqCovMatrices& prior_covs) const {
        // Gradient of log prior distribution
        // the prior distribution is assumed to be normal distribution
        // TODO: approximate as GMM like https://github.com/lubaroli/dust/blob/master/dust/inference/svgd.py
        ControlSeq grad_log_prior = sample * 0.0;
        ControlSeq diff = sample - prior_mean;
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            grad_log_prior.row(i) = prior_covs[i].inverse() * diff.row(i).transpose();
        }
        return grad_log_prior;
    }

    ControlSeqBatch SteinVariationalMPC::approx_grad_posterior_batch(
        const PriorSamplesWithCosts& samples,
        const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs) const {
        ControlSeqBatch grad_log_likelihoods = samples.get_zero_control_seq_batch();
        const ControlSeq mean = samples.get_mean();
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < samples.get_num_samples(); i++) {
            const ControlSeq grad_log_likelihood = approx_grad_log_likelihood(
                mean, samples.noised_control_seq_samples_[i], samples.get_inv_cov_matrices(), calc_costs, grad_sampler_ptrs_.at(i).get());
            const ControlSeq grad_log_prior =
                grad_log_normal_dist(samples.noised_control_seq_samples_[i], samples.get_mean(), samples.get_inv_cov_matrices());
            grad_log_likelihoods[i] = grad_log_likelihood + grad_log_prior;
        }

        return grad_log_likelihoods;
    }

    double SteinVariationalMPC::RBF_kernel(const ControlSeq& seq1, const ControlSeq& seq2, const double& h) const {
        const double kernel = (seq1 - seq2).squaredNorm();
        return std::exp(-kernel / h);
    }

    ControlSeq SteinVariationalMPC::grad_RBF_kernel(const ControlSeq& seq, const ControlSeq& seq_const, const double& h) const {
        const double kernel = RBF_kernel(seq, seq_const, h);
        const ControlSeq grad_kernel = -2.0 / h * kernel * (seq - seq_const);
        return grad_kernel;
    }

    ControlSeqBatch SteinVariationalMPC::phi_batch(const PriorSamplesWithCosts& samples, const ControlSeqBatch& grad_posterior_batch) const {
        // calculate median of samples
        // This makes the sum of kernel values close to 1
        std::vector<double> dists(samples.get_num_samples(), 0.0);
        for (size_t i = 0; i < samples.get_num_samples(); i++) {
            dists[i] = (samples.noised_control_seq_samples_[i]).squaredNorm();
        }
        std::sort(dists.begin(), dists.end());
        double h = dists[static_cast<size_t>(samples.get_num_samples() / 2)] / std::log(static_cast<double>(samples.get_num_samples()));
        h = std::max(h, 1e-5);

        // calculate phi batch
        ControlSeqBatch phi_batch = samples.get_zero_control_seq_batch();
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < samples.get_num_samples(); i++) {
            for (size_t j = 0; j < samples.get_num_samples(); j++) {
                const double kernel = RBF_kernel(samples.noised_control_seq_samples_[j], samples.noised_control_seq_samples_[i], h);

                phi_batch[i] += kernel * grad_posterior_batch[j];
                phi_batch[i] += grad_RBF_kernel(samples.noised_control_seq_samples_[j], samples.noised_control_seq_samples_[i], h);
            }

            phi_batch[i] /= static_cast<double>(samples.get_num_samples());
        }

        return phi_batch;
    }

    std::pair<ControlSeq, ControlSeqCovMatrices> SteinVariationalMPC::weighted_mean_and_sigma(const PriorSamplesWithCosts& samples,
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

    std::pair<ControlSeq, ControlSeqCovMatrices> SteinVariationalMPC::estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const {
        ControlSeq mu = samples.get_zero_control_seq();
        ControlSeqCovMatrices sigma = samples.get_zero_control_seq_cov_matrices();

// calculate mean
#pragma omp parallel for num_threads(thread_num_)

        for (size_t i = 0; i < samples.get_num_samples(); i++) {
            mu += samples.noised_control_seq_samples_[i];
        }
        mu /= static_cast<double>(samples.get_num_samples());

        // mu = samples.control_seq_mean_;

// calculate covariance matrices
#pragma omp parallel for num_threads(thread_num_)
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

    std::vector<double> SteinVariationalMPC::calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs) const {
        // calculate costs with control cost term
        const std::vector<double> costs_with_control_term =
            prior_samples_with_costs.get_costs_with_control_term(lambda_, alpha_, prior_samples_ptr_->get_zero_control_seq());

        // softmax weights
        return softmax(costs_with_control_term, lambda_, thread_num_);
    }

}  // namespace cpu
}  // namespace mppi
