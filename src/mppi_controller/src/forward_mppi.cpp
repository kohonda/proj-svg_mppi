#include "mppi_controller/forward_mppi.hpp"

namespace mppi {
namespace cpu {
    ForwardMPPI::ForwardMPPI(const Params::Common& common_params, const Params::ForwardMPPI& forward_mppi_params)
        : prediction_step_size_(static_cast<size_t>(common_params.prediction_step_size)),
          thread_num_(common_params.thread_num),
          lambda_(forward_mppi_params.lambda),
          alpha_(forward_mppi_params.alpha),
          non_biased_sampling_rate_(forward_mppi_params.non_biased_sampling_rate),
          steer_cov_(forward_mppi_params.steer_cov),
          num_itr_for_grad_estimation_(forward_mppi_params.num_itr_for_grad_estimation),
          step_size_for_grad_estimation_(forward_mppi_params.step_size_for_grad_estimation),
          sample_num_for_grad_estimation_(forward_mppi_params.sample_num_for_grad_estimation),
          steer_cov_for_grad_estimation_(forward_mppi_params.steer_cov_for_grad_estimation) {
        const size_t sample_batch_num = static_cast<size_t>(forward_mppi_params.sample_batch_num);
        mpc_base_ptr_ = std::make_unique<MPCBase>(common_params, sample_batch_num);

        const double max_steer_angle = common_params.max_steer_angle;
        const double min_steer_angle = common_params.min_steer_angle;
        std::array<double, CONTROL_SPACE::dim> max_control_inputs = {max_steer_angle};
        std::array<double, CONTROL_SPACE::dim> min_control_inputs = {min_steer_angle};
        prior_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(sample_batch_num, prediction_step_size_, max_control_inputs, min_control_inputs,
                                                                     non_biased_sampling_rate_, thread_num_);
        prev_control_seq_ = prior_samples_ptr_->get_zero_control_seq();

        // initialize grad samplers
        for (size_t i = 0; i < sample_batch_num; i++) {
            grad_sampler_ptrs_.emplace_back(std::make_unique<PriorSamplesWithCosts>(
                sample_num_for_grad_estimation_, prediction_step_size_, max_control_inputs, min_control_inputs, non_biased_sampling_rate_, 1, i));
        }
    }

    std::pair<ControlSeq, double> ForwardMPPI::solve(const State& initial_state) {
        int collision_num = 0;
        const std::array<double, CONTROL_SPACE::dim> control_cov_diag = {steer_cov_};

        ControlSeq control_seq_mean = prev_control_seq_;
        ControlSeqCovMatrices control_seq_cov_matrices = prior_samples_ptr_->get_constant_control_seq_cov_matrices(control_cov_diag);

        // generate random noised control sequences based on previous control sequence as mean
        prior_samples_ptr_->random_sampling(control_seq_mean, control_seq_cov_matrices);

        // Transport random samples with stein variational gradient descent
        auto func_calc_costs = [&](const PriorSamplesWithCosts& sampler) { return mpc_base_ptr_->calc_sample_costs(sampler, initial_state).first; };
        prior_samples_ptr_->noised_control_seq_samples_ =
            transport_samples(*prior_samples_ptr_, func_calc_costs, num_itr_for_grad_estimation_, step_size_for_grad_estimation_);
        // const ControlSeqBatch grad_kld_batch = grad_reverse_kld_batch(*prior_samples_ptr_, func_calc_costs);
        // prior_samples_ptr_->noised_control_seq_samples_ = transport_samples(*prior_samples_ptr_, grad_kld_batch,
        //                                                                     num_itr_for_grad_estimation_, step_size_for_grad_estimation_);

        // Predict and calculate trajectory costs
        auto [_costs, collision_costs] = mpc_base_ptr_->calc_sample_costs(*prior_samples_ptr_, initial_state);
        prior_samples_ptr_->costs_ = std::forward<std::vector<double>>(_costs);
        collision_num = std::count_if(collision_costs.begin(), collision_costs.end(), [](const double& cost) { return cost > 0.0; });

        // calculate weights for each state sequence based on costs and prior samples
        const std::vector<double> weights = calc_weights(*prior_samples_ptr_);
        weights_ = weights;  // for visualization

        // calculate optimal control sequence by weighted sum of control sequences
        ControlSeq updated_control_seq = prior_samples_ptr_->get_zero_control_seq();
        for (size_t i = 0; i < prior_samples_ptr_->get_num_samples(); i++) {
            updated_control_seq += weights[i] * prior_samples_ptr_->noised_control_seq_samples_.at(i);
        }

        // update mean control sequence for next iteration
        prev_control_seq_ = updated_control_seq;

        const double collision_rate = static_cast<double>(collision_num) / static_cast<double>(prior_samples_ptr_->get_num_samples());

        return std::make_pair(updated_control_seq, collision_rate);
    }

    void ForwardMPPI::set_obstacle_map(const grid_map::GridMap& obstacle_map) { mpc_base_ptr_->set_obstacle_map(obstacle_map); };

    void ForwardMPPI::set_reference_map(const grid_map::GridMap& reference_map) { mpc_base_ptr_->set_reference_map(reference_map); };

    std::pair<std::vector<StateSeq>, std::vector<double>> ForwardMPPI::get_state_seq_candidates(const int& num_samples) const {
        return mpc_base_ptr_->get_state_seq_candidates(num_samples, weights_);
    }

    std::tuple<StateSeq, double, double, double> ForwardMPPI::get_predictive_seq(const State& initial_state,
                                                                                 const ControlSeq& control_input_seq) const {
        const auto [prediction_state, state_cost, collision_cost] = mpc_base_ptr_->get_predictive_seq(initial_state, control_input_seq);
        double input_error = 0.0;
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            input_error += (control_input_seq.row(i)).norm();
        }
        return std::make_tuple(prediction_state, state_cost, collision_cost, input_error);
    }

    ControlSeqCovMatrices ForwardMPPI::get_cov_matrices() const {
        const auto [_mean, cov] = estimate_mu_and_sigma(*prior_samples_ptr_);
        return cov;
    }

    ControlSeq ForwardMPPI::get_control_seq() const { return prev_control_seq_; }

    std::pair<StateSeq, XYCovMatrices> ForwardMPPI::get_proposed_state_distribution() const { return mpc_base_ptr_->get_proposed_distribution(); }

    // == private functions ==
    std::vector<double> ForwardMPPI::calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs) const {
        // calculate costs with control cost term
        const std::vector<double> costs_with_control_term =
            prior_samples_with_costs.get_costs_with_control_term(lambda_, alpha_, prior_samples_ptr_->get_zero_control_seq());

        // softmax weights
        return softmax(costs_with_control_term, lambda_, thread_num_);
    }

    std::vector<int> ForwardMPPI::get_top_indices(const std::vector<double>& values, const int& num) const {
        std::vector<int> indices(values.size());
        std::iota(indices.begin(), indices.end(), 0);

        std::partial_sort(indices.begin(), indices.begin() + num, indices.end(), [&](int i1, int i2) { return values[i1] > values[i2]; });

        indices.resize(num);

        return indices;
    }

    ControlSeq ForwardMPPI::grad_reverse_kld(const ControlSeq& mean_seq,
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

        const ControlSeq phi = -sum_of_grads / (sum_of_costs + 1e-10);

        return phi;
    }

    ControlSeqBatch ForwardMPPI::grad_reverse_kld_batch(const PriorSamplesWithCosts& samples,
                                                        const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs) const {
        ControlSeqBatch grad_kld = samples.get_zero_state_seq_batch();
        const ControlSeq mean = samples.get_mean();
        const ControlSeqCovMatrices inv_covs = samples.get_inv_cov_matrices();
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < samples.get_num_samples(); i++) {
            grad_kld[i] = grad_reverse_kld(mean, samples.noised_control_seq_samples_[i], inv_covs, calc_costs, grad_sampler_ptrs_.at(i).get());
        }

        return grad_kld;
    }

    ControlSeqBatch ForwardMPPI::transport_samples(const PriorSamplesWithCosts& samples_with_cost,
                                                   const std::function<std::vector<double>(const PriorSamplesWithCosts&)>& calc_costs,
                                                   const int& num_itr,
                                                   const double& step_size) {
        // If all cost is too small, return samples as it is
        // const double max_cost = *std::max_element(samples_with_cost.costs_.begin(), samples_with_cost.costs_.end());
        // if (max_cost < 1e-5)
        // {
        //     return samples_with_cost.noised_control_seq_samples_;
        // }

        ControlSeqBatch transported_samples = samples_with_cost.noised_control_seq_samples_;
        const ControlSeq mean = samples_with_cost.get_mean();
        const ControlSeqCovMatrices inv_covs = samples_with_cost.get_inv_cov_matrices();
        for (int i = 0; i < num_itr; i++) {
#pragma omp parallel for num_threads(thread_num_)
            for (size_t j = 0; j < transported_samples.size(); j++) {
                const ControlSeq grad = grad_reverse_kld(mean, transported_samples[j], inv_covs, calc_costs, grad_sampler_ptrs_.at(j).get());

                transported_samples[j] = transported_samples[j] - step_size * grad;
            }
        }

        return transported_samples;
    }

    ControlSeqBatch ForwardMPPI::transport_samples(const PriorSamplesWithCosts& samples,
                                                   const ControlSeqBatch& grad_kld_batch,
                                                   const int& num_itr,
                                                   const double& step_size) const {
        ControlSeqBatch transported_samples = samples.noised_control_seq_samples_;

        for (int i = 0; i < num_itr; i++) {
#pragma omp parallel for num_threads(thread_num_)
            for (size_t j = 0; j < transported_samples.size(); j++) {
                transported_samples[j] = transported_samples[j] - step_size * grad_kld_batch[j];
            }
        }

        return transported_samples;
    }

    std::pair<ControlSeq, ControlSeqCovMatrices> ForwardMPPI::estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const {
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

}  // namespace cpu
}  // namespace mppi
