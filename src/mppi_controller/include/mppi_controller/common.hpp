#pragma once

#include <string>

namespace mppi {
namespace STATE_SPACE {
    static constexpr int x = 0;
    static constexpr int y = 1;
    static constexpr int yaw = 2;
    static constexpr int vel = 3;
    static constexpr int steer = 4;
    static constexpr int dim = 5;
};  // namespace STATE_SPACE

namespace CONTROL_SPACE {
    static constexpr int steer = 0;
    // static constexpr int accel = 1;
    // static constexpr int dim = 2;

    static constexpr int dim = 1;
};  // namespace CONTROL_SPACE

struct Params {
    struct Common {
        int thread_num;
        int prediction_step_size;
        double prediction_interval;
        double steer_delay;
        double steer_1st_delay;
        bool is_localize_less_mode;
        double reference_speed;
        double max_steer_angle;
        double min_steer_angle;
        double max_accel;
        double min_accel;
        std::string speed_prediction_mode;
        double lr;
        double lf;
        double q_dist;
        double q_angle;
        double q_speed;
        double collision_weight;
        double q_terminal_dist;
        double q_terminal_angle;
        double q_terminal_speed;
    };
    Common common;

    struct ForwardMPPI {
        int sample_batch_num;
        double lambda;
        double alpha;
        double non_biased_sampling_rate;
        double steer_cov;
        // double accel_cov;
        int sample_num_for_grad_estimation;
        double steer_cov_for_grad_estimation;
        int num_itr_for_grad_estimation;
        double step_size_for_grad_estimation;
    };
    ForwardMPPI forward_mppi;

    struct ReverseMPPI {
        int sample_batch_num;
        double negative_ratio;
        bool is_sample_rejection;
        double sample_inflation_ratio;
        double warm_start_ratio;
        int iteration_num;
        double step_size;
        double lambda;
        double alpha;
        double non_biased_sampling_rate;
        double steer_cov;
        // double accel_cov;
    };
    ReverseMPPI reverse_mppi;

    struct SteinVariationalMPC {
        int sample_batch_num;
        double lambda;
        double alpha;
        double non_biased_sampling_rate;
        double steer_cov;
        // double accel_cov;
        int num_svgd_iteration;
        int sample_num_for_grad_estimation;
        double steer_cov_for_grad_estimation;
        double svgd_step_size;
        bool is_max_posterior_estimation;
    };
    SteinVariationalMPC stein_variational_mpc;

    struct SVGuidedMPPI {
        int sample_batch_num;
        double lambda;
        double alpha;
        double non_biased_sampling_rate;
        double steer_cov;
        // double accel_cov;
        int guide_sample_num;
        double grad_lambda;
        int sample_num_for_grad_estimation;
        double steer_cov_for_grad_estimation;
        double svgd_step_size;
        int num_svgd_iteration;
        bool is_use_nominal_solution;
        bool is_covariance_adaptation;
        double gaussian_fitting_lambda;
        double min_steer_cov;
        double max_steer_cov;
    };
    SVGuidedMPPI svg_mppi;
};

namespace cpu {
    using State = Eigen::Matrix<double, STATE_SPACE::dim, 1>;
    using Control = Eigen::Matrix<double, CONTROL_SPACE::dim, 1>;
    using StateSeq = Eigen::MatrixXd;
    using ControlSeq = Eigen::MatrixXd;
    using StateSeqBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
    using ControlSeqBatch = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
    using ControlSeqCovMatrices = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
    using XYCovMatrices = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd>>;
}  // namespace cpu

}  // namespace mppi
