#include "mppi_controller/reverse_mppi.hpp"

namespace mppi {
namespace cpu {
    ReverseMPPI::ReverseMPPI(const Params::Common& common_params, const Params::ReverseMPPI& reverse_mppi_params)
        : prediction_step_size_(static_cast<size_t>(common_params.prediction_step_size)),
          thread_num_(common_params.thread_num),
          negative_ratio_(reverse_mppi_params.negative_ratio),
          is_sample_rejection_(reverse_mppi_params.is_sample_rejection),
          sample_inflation_ratio_(reverse_mppi_params.sample_inflation_ratio),
          iteration_num_(reverse_mppi_params.iteration_num),
          step_size_(reverse_mppi_params.step_size),
          warm_start_ratio_(reverse_mppi_params.warm_start_ratio),
          lambda_(reverse_mppi_params.lambda),
          alpha_(reverse_mppi_params.alpha),
          non_biased_sampling_rate_(reverse_mppi_params.non_biased_sampling_rate),
          steer_cov_(reverse_mppi_params.steer_cov) {
        const size_t sample_batch_num = static_cast<size_t>(reverse_mppi_params.sample_batch_num);
        mpc_base_ptr_ = std::make_unique<MPCBase>(common_params, sample_batch_num);

        const double max_steer_angle = common_params.max_steer_angle;
        const double min_steer_angle = common_params.min_steer_angle;
        std::array<double, CONTROL_SPACE::dim> max_control_inputs = {max_steer_angle};
        std::array<double, CONTROL_SPACE::dim> min_control_inputs = {min_steer_angle};

        prior_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(sample_batch_num, prediction_step_size_, max_control_inputs, min_control_inputs,
                                                                     non_biased_sampling_rate_, thread_num_);

        const size_t inflated_sample_batch_num = static_cast<size_t>(std::round(sample_batch_num * sample_inflation_ratio_));
        inflated_samples_ptr_ = std::make_unique<PriorSamplesWithCosts>(inflated_sample_batch_num, prediction_step_size_, max_control_inputs,
                                                                        min_control_inputs, non_biased_sampling_rate_, thread_num_);

        prev_control_seq_ = prior_samples_ptr_->get_zero_control_seq();
        const std::array<double, CONTROL_SPACE::dim> control_cov_diag = {steer_cov_};
        prev_covs_ = prior_samples_ptr_->get_constant_control_seq_cov_matrices(control_cov_diag);

        prev_rejected_mean_ = prior_samples_ptr_->get_zero_control_seq();
        prev_rejected_covs_ = prior_samples_ptr_->get_constant_control_seq_cov_matrices(control_cov_diag);
    }

    std::pair<ControlSeq, double> ReverseMPPI::solve(const State& initial_state) {
        int collision_num = 0;

        ControlSeq control_seq_mean = prev_control_seq_;
        ControlSeqCovMatrices control_seq_cov_matrices = prev_covs_;
        ControlSeq rejected_mean = prev_rejected_mean_;
        ControlSeqCovMatrices rejected_covs = prev_rejected_covs_;
        for (int i = 0; i < iteration_num_; i++) {
            if (is_sample_rejection_) {
                // generate inflated (sample_batch_num x sample_inflation_ratio) samples, not sample rejection yet
                inflated_samples_ptr_->random_sampling(control_seq_mean, control_seq_cov_matrices);

                // calculate sample rejection probabilities
                ControlSeq normal_prob = normal_pdf(rejected_mean, control_seq_mean, control_seq_cov_matrices);
                std::vector<double> probabilities(inflated_samples_ptr_->get_num_samples(), 0.0);
#pragma omp parallel for num_threads(thread_num_)
                for (size_t j = 0; j < inflated_samples_ptr_->get_num_samples(); j++) {
                    const ControlSeq pr = normal_pdf(inflated_samples_ptr_->noised_control_seq_samples_[j], rejected_mean, rejected_covs);
                    probabilities[j] = (normal_prob + pr).cwiseInverse().prod();
                }

                // normalize probabilities
                const double sum_prob = std::min(std::accumulate(probabilities.begin(), probabilities.end(), 0.0), 1e-10);
#pragma omp parallel for num_threads(thread_num_)
                for (size_t j = 0; j < inflated_samples_ptr_->get_num_samples(); j++) {
                    probabilities[j] /= sum_prob;
                }

                const std::vector<int> selected_indices =
                    inflated_samples_ptr_->random_sample_choice(prior_samples_ptr_->get_num_samples(), probabilities);
                prior_samples_ptr_->shrink_copy_from(*inflated_samples_ptr_, selected_indices);
            } else {
                prior_samples_ptr_->random_sampling(control_seq_mean, control_seq_cov_matrices);
            }

            // Predict and calculate trajectory costs
            auto [_costs, collision_costs] = mpc_base_ptr_->calc_sample_costs(*prior_samples_ptr_, initial_state);
            prior_samples_ptr_->costs_ = std::forward<std::vector<double>>(_costs);
            collision_num = std::count_if(collision_costs.begin(), collision_costs.end(), [](const double& cost) { return cost > 0.0; });

            // calculate weights for each state sequence based on costs and prior samples
            const auto [weights_eval, weights_reject] = calc_weights(*prior_samples_ptr_);
            weights_ = weights_eval;  // for visualization

            // calculate weighted mean and standard deviation of evaluated and rejected samples
            const auto [mean_eval, sigma_eval] = weighted_mean_and_sigma(*prior_samples_ptr_, weights_eval);
            const auto [mean_reject, sigma_reject] = weighted_mean_and_sigma(*prior_samples_ptr_, weights_reject);

            // MD update
            const auto [new_mean, new_covs] = md_update(control_seq_mean, control_seq_cov_matrices, mean_eval, sigma_eval, step_size_);
            control_seq_mean = new_mean;
            control_seq_cov_matrices = new_covs;

            const auto [new_rejected_mean, new_rejected_covs] = md_update(rejected_mean, rejected_covs, mean_reject, sigma_reject, step_size_);
            rejected_mean = new_rejected_mean;
            rejected_covs = new_rejected_covs;
        }

        // Warm start for next control iteration
        prev_control_seq_ = interpolate(prev_control_seq_, control_seq_mean, warm_start_ratio_);
        prev_covs_ = interpolate(prev_covs_, control_seq_cov_matrices, warm_start_ratio_);
        prev_rejected_mean_ = interpolate(prev_rejected_mean_, rejected_mean, warm_start_ratio_);
        prev_rejected_covs_ = interpolate(prev_rejected_covs_, rejected_covs, warm_start_ratio_);

        const double collision_rate = static_cast<double>(collision_num) / static_cast<double>(prior_samples_ptr_->get_num_samples());

        return std::make_pair(control_seq_mean, collision_rate);
    }

    void ReverseMPPI::set_obstacle_map(const grid_map::GridMap& obstacle_map) { mpc_base_ptr_->set_obstacle_map(obstacle_map); };

    void ReverseMPPI::set_reference_map(const grid_map::GridMap& reference_map) { mpc_base_ptr_->set_reference_map(reference_map); };

    std::pair<std::vector<StateSeq>, std::vector<double>> ReverseMPPI::get_state_seq_candidates(const int& num_samples) const {
        return mpc_base_ptr_->get_state_seq_candidates(num_samples, weights_);
    }

    std::tuple<StateSeq, double, double, double> ReverseMPPI::get_predictive_seq(const State& initial_state,
                                                                                 const ControlSeq& control_input_seq) const {
        const auto [prediction_state, state_cost, collision_cost] = mpc_base_ptr_->get_predictive_seq(initial_state, control_input_seq);
        double input_error = 0.0;
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            input_error += (control_input_seq.row(i)).norm();
        }
        return std::make_tuple(prediction_state, state_cost, collision_cost, input_error);
    }

    ControlSeqCovMatrices ReverseMPPI::get_cov_matrices() const {
        const auto [_mean, cov] = estimate_mu_and_sigma(*prior_samples_ptr_);
        return cov;
    }

    ControlSeq ReverseMPPI::get_control_seq() const { return prev_control_seq_; }

    std::pair<StateSeq, XYCovMatrices> ReverseMPPI::get_proposed_state_distribution() const { return mpc_base_ptr_->get_proposed_distribution(); }

    // == private functions ==
    std::pair<std::vector<double>, std::vector<double>> ReverseMPPI::calc_weights(const PriorSamplesWithCosts& prior_samples_with_costs) const {
        // calculate costs with control cost term
        const std::vector<double> costs_with_control_term =
            prior_samples_with_costs.get_costs_with_control_term(lambda_, alpha_, prior_samples_ptr_->get_zero_control_seq());

        // calculate normalization term
        double positive_normalization_term = 1e-10;
        double negative_normalization_term = 1e-10;
        const double min_cost = *std::min_element(costs_with_control_term.begin(), costs_with_control_term.end());
        for (size_t i = 0; i < prior_samples_with_costs.get_num_samples(); i++) {
            positive_normalization_term += std::exp(-1.0 / lambda_ * (costs_with_control_term[i] - min_cost));
            negative_normalization_term += std::exp(1.0 / lambda_ * (costs_with_control_term[i] - min_cost));
        }

        std::vector<double> weights_eval(prior_samples_with_costs.get_num_samples(), 0.0);
        std::vector<double> weights_reject(prior_samples_with_costs.get_num_samples(), 0.0);
// calculate weights for importance sampling
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < prior_samples_with_costs.get_num_samples(); i++) {
            const double positive_term = std::exp(-1.0 / lambda_ * (costs_with_control_term[i] - min_cost)) / positive_normalization_term;
            const double negative_term =
                negative_ratio_ * std::exp(1.0 / lambda_ * (costs_with_control_term[i] - min_cost)) / negative_normalization_term;

            const double reversed_weight = positive_term - negative_term;
            weights_eval[i] = std::max(reversed_weight, 0.0);
            weights_reject[i] = -std::min(reversed_weight, 0.0);
        }

        // normalize weights
        const double sum_weights_eval = std::accumulate(weights_eval.begin(), weights_eval.end(), 0.0);
        const double sum_weights_reject = std::accumulate(weights_reject.begin(), weights_reject.end(), 0.0);
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < prior_samples_with_costs.get_num_samples(); i++) {
            weights_eval[i] /= sum_weights_eval;
            weights_reject[i] /= sum_weights_reject;
        }

        return std::make_pair(weights_eval, weights_reject);
    }

    std::pair<ControlSeq, ControlSeqCovMatrices> ReverseMPPI::estimate_mu_and_sigma(const PriorSamplesWithCosts& samples) const {
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

    std::pair<ControlSeq, ControlSeqCovMatrices> ReverseMPPI::weighted_mean_and_sigma(const PriorSamplesWithCosts& samples,
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

    std::pair<ControlSeq, ControlSeqCovMatrices> ReverseMPPI::md_update(const ControlSeq& prior_mean,
                                                                        const ControlSeqCovMatrices& prior_covs,
                                                                        const ControlSeq& weighted_mean,
                                                                        const ControlSeqCovMatrices& weighted_covs,
                                                                        const double step_size) const {
        ControlSeq updated_mean = prior_mean * 0.0;
        updated_mean = prior_mean - step_size * (prior_mean - weighted_mean);

        ControlSeqCovMatrices prior_stds = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            prediction_step_size_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
        ControlSeqCovMatrices weighted_stds = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            prediction_step_size_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            prior_stds[i] = prior_covs[i].diagonal().cwiseSqrt().asDiagonal();
            weighted_stds[i] = weighted_covs[i].diagonal().cwiseSqrt().asDiagonal();
        }

        ControlSeqCovMatrices tmp = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            prediction_step_size_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            tmp[i] = -step_size * (prior_stds[i] - weighted_stds[i]);
        }
        ControlSeqCovMatrices updated_covs = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            prediction_step_size_ - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            const Eigen::MatrixXd updated_std =
                0.5 * (tmp[i] + (tmp[i].transpose() * tmp[i] + 4.0 * prior_stds[i].transpose() * prior_stds[i]).cwiseSqrt());
            updated_covs[i] = updated_std * updated_std;
        }

        return std::make_pair(updated_mean, updated_covs);
    }

    ControlSeq ReverseMPPI::normal_pdf(const ControlSeq& x, const ControlSeq& mean, const ControlSeqCovMatrices& var) const {
        ControlSeq pdf = x * 0.0;
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            pdf.row(i) = (x.row(i) - mean.row(i))
                             .cwiseProduct((var[i].diagonal().cwiseSqrt().asDiagonal().inverse() * (x.row(i) - mean.row(i)).transpose()).transpose())
                             .array()
                             .exp() /
                         std::sqrt(std::pow(2.0 * M_PI, CONTROL_SPACE::dim) * var[i].determinant());
        }
        return pdf;
    }

    ControlSeq ReverseMPPI::interpolate(const ControlSeq& x1, const ControlSeq& x2, const double& ratio) const {
        ControlSeq x = x1 * 0.0;
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            x.row(i) = (1.0 - ratio) * x1.row(i) + ratio * x2.row(i);
        }
        return x;
    }

    ControlSeqCovMatrices ReverseMPPI::interpolate(const ControlSeqCovMatrices& covs1,
                                                   const ControlSeqCovMatrices& covs2,
                                                   const double& ratio) const {
        ControlSeqCovMatrices covs = covs1;
        for (size_t i = 0; i < prediction_step_size_ - 1; i++) {
            covs[i] = (1.0 - ratio) * covs1[i] + ratio * covs2[i];
        }
        return covs;
    }

}  // namespace cpu
}  // namespace mppi
