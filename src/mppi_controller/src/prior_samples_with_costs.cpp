#include "mppi_controller/prior_samples_with_costs.hpp"

namespace mppi {
namespace cpu {
    PriorSamplesWithCosts::PriorSamplesWithCosts(const size_t& num_samples,
                                                 const size_t& prediction_horizon,
                                                 const std::array<double, CONTROL_SPACE::dim>& max_control_inputs,
                                                 const std::array<double, CONTROL_SPACE::dim>& min_control_inputs,
                                                 const double& non_biased_sampling_rate,
                                                 const int& thread_num,
                                                 const int& seed)
        : thread_num_(thread_num),
          num_samples_(num_samples),
          prediction_horizon_(prediction_horizon),
          non_biased_sampling_rate_(non_biased_sampling_rate),
          max_control_inputs_(max_control_inputs),
          min_control_inputs_(min_control_inputs) {
        control_seq_mean_ = Eigen::MatrixXd::Zero(prediction_horizon - 1, CONTROL_SPACE::dim);
        control_seq_cov_matrices_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            prediction_horizon - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
        control_seq_inv_cov_matrices_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            prediction_horizon - 1, Eigen::MatrixXd::Zero(CONTROL_SPACE::dim, CONTROL_SPACE::dim));
        noised_control_seq_samples_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            num_samples, Eigen::MatrixXd::Zero(prediction_horizon - 1, CONTROL_SPACE::dim));
        noise_seq_samples_ = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>(
            num_samples, Eigen::MatrixXd::Zero(prediction_horizon - 1, CONTROL_SPACE::dim));
        costs_ = std::vector<double>(num_samples, 0.0);

        // Initialize random number generator
        for (int i = 0; i < thread_num_; i++) {
            rngs_.push_back(std::mt19937(seed + i));
        }

        // Initialize normal distributions
        normal_dists_ptr_ = std::make_unique<std::vector<std::array<std::normal_distribution<>, CONTROL_SPACE::dim>>>();
        for (size_t i = 0; i < prediction_horizon - 1; i++) {
            std::array<std::normal_distribution<>, CONTROL_SPACE::dim> normal_dists = {};
            for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
                std::normal_distribution<> normal_dist(0.0, 1.0);
                normal_dists[j] = normal_dist;
            }
            (*normal_dists_ptr_).push_back(normal_dists);
        }
    }

    void PriorSamplesWithCosts::random_sampling(const ControlSeq& control_seq_mean, const ControlSeqCovMatrices& control_seq_cov_matrices) {
        set_control_seq_mean(control_seq_mean);
        set_control_seq_cov_matrices(control_seq_cov_matrices);

        // set normal distributions parameters
        for (size_t i = 0; i < prediction_horizon_ - 1; i++) {
            for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
                const double std_dev = std::sqrt(control_seq_cov_matrices_[i](j, j));
                std::normal_distribution<>::param_type param(0.0, std_dev);
                (*normal_dists_ptr_)[i][j].param(param);
            }
        }

#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < num_samples_; i++) {
            // generate noise sequence
            for (size_t j = 0; j < prediction_horizon_ - 1; j++) {
                for (size_t k = 0; k < CONTROL_SPACE::dim; k++) {
                    noise_seq_samples_[i](j, k) = (*normal_dists_ptr_)[j][k](rngs_[omp_get_thread_num()]);
                }
            }

            // sampling control sequences with non-biased (around zero) sampling rate
            if (i < static_cast<size_t>((1 - non_biased_sampling_rate_) * num_samples_)) {
                // biased sampling (around control_seq_mean)
                noised_control_seq_samples_[i] = control_seq_mean + noise_seq_samples_[i];
            } else {
                // non-biased sampling (around zero)
                noised_control_seq_samples_[i] = noise_seq_samples_[i];
            }

            // clip input with control input constraints
            for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
                for (size_t k = 0; k < prediction_horizon_ - 1; k++) {
                    noised_control_seq_samples_[i](k, j) =
                        std::clamp(noised_control_seq_samples_[i](k, j), min_control_inputs_[j], max_control_inputs_[j]);
                }
            }
        }
    }

    std::vector<int> PriorSamplesWithCosts::random_sample_choice(const size_t& num_samples, const std::vector<double>& probabilities) {
        if (num_samples >= num_samples_) {
            std::cout << "Error: num_samples is larger than num_samples_." << std::endl;
            exit(1);
        }

        if (probabilities.size() != num_samples_) {
            std::cout << "Error: probability size is not equal to num_samples_." << std::endl;
            exit(1);
        }

        // choose random indices with probability
        discrete_dist_.param(std::discrete_distribution<>::param_type(probabilities.begin(), probabilities.end()));
        std::vector<int> indices(num_samples);
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < num_samples; i++) {
            indices[i] = discrete_dist_(rngs_[omp_get_thread_num()]);
        }

        return indices;
    }

    std::vector<double> PriorSamplesWithCosts::get_costs_with_control_term(const double& lambda,
                                                                           const double& alpha,
                                                                           const ControlSeq& nominal_control_seq) const {
        std::vector<double> costs_with_control_term = costs_;
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < num_samples_; i++) {
            for (size_t j = 0; j < prediction_horizon_ - 1; j++) {
                const double control_term = lambda * (1 - alpha) * (control_seq_mean_.row(j) - nominal_control_seq.row(j)) *
                                            control_seq_inv_cov_matrices_[j] * noised_control_seq_samples_[i].row(j).transpose();
                costs_with_control_term[i] += control_term;
            }
        }
        return costs_with_control_term;
    }

    // copy part of samples that are selected from source samples
    void PriorSamplesWithCosts::shrink_copy_from(const PriorSamplesWithCosts& source_samples, const std::vector<int>& indices) {
        // check: source samples size is larger than indices size because shrinked samples are stored in this class
        if (indices.size() != num_samples_) {
            std::cout << "Error: indices size is not equal to num_samples_." << std::endl;
            exit(1);
        }

        if (source_samples.prediction_horizon_ != prediction_horizon_) {
            std::cout << "Error: source_samples.prediction_horizon_ is not equal to prediction_horizon_." << std::endl;
            exit(1);
        }

        if (source_samples.num_samples_ < indices.size()) {
            std::cout << "Error: source_samples.num_samples_ is smaller than indices.size()." << std::endl;
            exit(1);
        }

#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < indices.size(); i++) {
            noised_control_seq_samples_[i] = source_samples.noised_control_seq_samples_[indices[i]];
            noise_seq_samples_[i] = source_samples.noise_seq_samples_[indices[i]];
            costs_[i] = source_samples.costs_[indices[i]];
        }
        set_control_seq_mean(source_samples.control_seq_mean_);
        set_control_seq_cov_matrices(source_samples.control_seq_cov_matrices_);
    }

    // copy from source samples and inflate samples until num_samples_ is satisfied with probabilities
    // covもいるね
    void PriorSamplesWithCosts::inflate_copy_from(const PriorSamplesWithCosts& source_samples, const ControlSeqCovMatrices& noise_covs) {
        // check
        if (source_samples.prediction_horizon_ != prediction_horizon_) {
            std::cout << "Error: source_samples.prediction_horizon_ is not equal to prediction_horizon_." << std::endl;
            exit(1);
        }

        if (source_samples.num_samples_ > num_samples_) {
            std::cout << "Error: source_samples.num_samples_ should is smaller than num_samples_." << std::endl;
            exit(1);
        }

        // copy samples from source samples
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < source_samples.num_samples_; i++) {
            noised_control_seq_samples_[i] = source_samples.noised_control_seq_samples_[i];
            noise_seq_samples_[i] = source_samples.noise_seq_samples_[i];
            costs_[i] = source_samples.costs_[i];
        }

        // inflate samples
        const size_t num_inflated_samples = num_samples_ - source_samples.num_samples_;
        std::vector<double> probabilities(source_samples.num_samples_, 1.0 / source_samples.num_samples_);
        discrete_dist_.param(std::discrete_distribution<>::param_type(probabilities.begin(), probabilities.end()));
        std::vector<int> indices(num_inflated_samples, 0);
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < num_inflated_samples; i++) {
            indices[i] = discrete_dist_(rngs_[omp_get_thread_num()]);
        }

        // add sample with noise
        for (size_t i = 0; i < prediction_horizon_ - 1; i++) {
            for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
                const double std_dev = std::sqrt(noise_covs[i](j, j));
                std::normal_distribution<>::param_type param(0.0, std_dev);
                (*normal_dists_ptr_)[i][j].param(param);
            }
        }
#pragma omp parallel for num_threads(thread_num_)
        for (size_t i = 0; i < num_inflated_samples; i++) {
            for (size_t j = 0; j < prediction_horizon_ - 1; j++) {
                for (size_t k = 0; k < CONTROL_SPACE::dim; k++) {
                    noise_seq_samples_[source_samples.num_samples_ + i](j, k) = (*normal_dists_ptr_)[j][k](rngs_[omp_get_thread_num()]);
                }
            }

            noised_control_seq_samples_[source_samples.num_samples_ + i] =
                source_samples.noised_control_seq_samples_[indices[i]] + noise_seq_samples_[source_samples.num_samples_ + i];

            // clip input with control input constraints
            for (size_t j = 0; j < CONTROL_SPACE::dim; j++) {
                for (size_t k = 0; k < prediction_horizon_ - 1; k++) {
                    noised_control_seq_samples_[source_samples.num_samples_ + i](k, j) = std::clamp(
                        noised_control_seq_samples_[source_samples.num_samples_ + i](k, j), min_control_inputs_[j], max_control_inputs_[j]);
                }
            }
        }
    }

    void PriorSamplesWithCosts::set_control_seq_mean(const ControlSeq& control_seq_mean) { control_seq_mean_ = control_seq_mean; }

    void PriorSamplesWithCosts::set_control_seq_cov_matrices(const ControlSeqCovMatrices& control_seq_cov_matrices) {
        // To prevent singular matrix, add small value to diagonal elements
        const double eps = 1e-4;
        control_seq_cov_matrices_ = control_seq_cov_matrices;

        // calculate inverse of covariance matrices in advance to reduce computational cost
        for (size_t i = 0; i < prediction_horizon_ - 1; i++) {
            control_seq_inv_cov_matrices_[i] =
                control_seq_cov_matrices_[i].inverse() + eps * Eigen::MatrixXd::Identity(CONTROL_SPACE::dim, CONTROL_SPACE::dim);
        }
    }

}  // namespace cpu

}  // namespace mppi