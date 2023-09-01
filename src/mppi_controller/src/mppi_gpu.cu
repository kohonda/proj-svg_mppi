#include "mppi_controller/mppi_gpu.cuh"


namespace mppi
{
    namespace cuda
    {
    
        MPPIGPU::MPPIGPU(const Params &params)
        {
            std::cout << "MPPIGPU::MPPIGPU()" << std::endl;
            // warming up GPU
            cudaDeviceSynchronize();

            // curand setting
            curandCreateGenerator(&rng_, CURAND_RNG_PSEUDO_MTGP32);
            const auto seed = 42;
            curandSetPseudoRandomGeneratorSeed(rng_, seed);


            set_params(params);

            // initialize member variables as zero
            // prev_control_seq_ptr_.reset(new thrust::device_vector<Control>(1));
            // (*prev_control_seq_ptr_)[0] = Control::Zero().eval();

            // noised_control_seq_batch_ptr_.reset(new thrust::device_vector<ControlSeq>(RANDOM_SAMPLES));

            // noise_seq_batch_ptr_.reset(new thrust::device_vector<ControlSeq>(RANDOM_SAMPLES));
            
            prev_control_seq_.resize(PREDICTION_HORIZON-1);
            // noised_control_seq_batch_.resize(RANDOM_SAMPLES);
            // noise_seq_batch_.resize(RANDOM_SAMPLES);
        }

        MPPIGPU::~MPPIGPU()
        {
            curandDestroyGenerator(rng_);
        }

        std::array<double, CONTROL_SPACE::dim> MPPIGPU::solve(const std::array<double, STATE_SPACE::dim> &initial_state)
        {
            thrust::device_vector<double> d_initial_state(initial_state.begin(), initial_state.end());

            // generate random noised control sequences based on previous control sequence as mean
            // generate_noise_seq_batch(prev_control_seq_, &noised_control_seq_batch_, &noise_seq_batch_);
        }

        // MPPIGPU::StateSeq MPPIGPU::get_predictive_seq(const State &initial_state) const
        // {
        //     // return predict_state_seq(prev_control_seq_, initial_state);
        // }

        // std::pair<std::vector<MPPIGPU::StateSeq>, std::vector<double>> MPPIGPU::get_state_seq_candidates(const int &num_samples) const
        // {
          
        // }

        // struct random_sampling_kearnel
        // {
        //     const double sigma_steer_;
        //     const double sigma_accel_;
        //     const ControlSeq prev_control_seq_;

        //     thrust::device_vector<double> steer_noise_seq_;
        //     thrust::device_vector<double> accel_noise_seq_;

        //     thrust::device_ptr<ControlSeqBatch> noised_control_seq_batch_ptr_;
        //     thrust::device_ptr<ControlSeqBatch> noise_seq_batch_ptr_;
        //     random_sampling_kearnel(const double &sigma_steer, const double &sigma_accel, const ControlSeq &prev_control_seq, ControlSeqBatch &noised_control_seq_batch, ControlSeqBatch &noise_seq_batch)
        //         : sigma_steer_(sigma_steer), sigma_accel_(sigma_accel), prev_control_seq_(prev_control_seq), noised_control_seq_batch_ptr_(&noised_control_seq_batch), noise_seq_batch_ptr_(&noise_seq_batch){
        //             steer_noise_seq_.resize(RANDOM_SAMPLES * PREDICTION_HORIZON-1);
        //             accel_noise_seq_.resize(RANDOM_SAMPLES * PREDICTION_HORIZON-1);
        //             curandGenerateNormalDouble(rng_, thrust::raw_pointer_cast(steer_noise_seq_.data()), steer_noise_seq_.size(), 0.0, sigma_steer_);
        //             curandGenerateNormalDouble(rng_, thrust::raw_pointer_cast(accel_noise_seq_.data()), accel_noise_seq_.size(), 0.0, sigma_accel_);
        //         }
            
        //     __device__ void operator()(const int &i) const
        //     {
        //         for (int j = 0; j < PREDICTION_HORIZON-1; j++)
        //         {
        //             noise_seq_batch_ptr_[i][j][0] = steer_noise_seq_[i * (PREDICTION_HORIZON-1) + j];
        //         }
        //     }

        // };

        void MPPIGPU::generate_noise_seq_batch(const ControlSeq &prev_control_seq, ControlSeqBatch *noised_control_seq_batch, ControlSeqBatch *noise_seq_batch)
        {
            // generate gaussian noise N(0, sigma)
            thrust::device_vector<double> steer_noise_seq(RANDOM_SAMPLES * PREDICTION_HORIZON-1);
            thrust::device_vector<double> accel_noise_seq(RANDOM_SAMPLES * PREDICTION_HORIZON-1);
            // const double sigma_steer = sqrt(steer_cov_);
            // const double sigma_accel = sqrt(accel_cov_);
            const double sigma_steer = steer_cov_;
            const double sigma_accel = accel_cov_;
            curandGenerateNormalDouble(rng_, thrust::raw_pointer_cast(steer_noise_seq.data()), steer_noise_seq.size(), 0.0, sigma_steer);
            curandGenerateNormalDouble(rng_, thrust::raw_pointer_cast(accel_noise_seq.data()), accel_noise_seq.size(), 0.0, sigma_accel);
            
            // add noise to previous control sequence
        }

        // __device__ 
        // inline MPPIGPU::ControlSeq add_noise(const MPPIGPU::ControlSeq &prev_control_seq, const thrust::device_vector<double>& steer_noise, const thrust::device_vector<double>& accel_noise) noexcept
        // {
        //     MPPIGPU::ControlSeq noised_control_seq(PREDICTION_HORIZON-1);
        //     for (int i = 0; i < PREDICTION_HORIZON-1; i++)
        //     {
        //         noised_control_seq[i](0) = prev_control_seq[i](0) + steer_noise[i];
        //         noised_control_seq[i](1) = prev_control_seq[i](1) + accel_noise[i];
        //     }
        //     return noised_control_seq;
        // }

       MPPIGPU::ControlSeqBatch MPPIGPU::generate_noise_seq_batch(const ControlSeq &prev_control_seq) const
        {
            // generate gaussian noise N(0, sigma)
            thrust::device_vector<double> steer_noise_seq(RANDOM_SAMPLES * PREDICTION_HORIZON-1);
            thrust::device_vector<double> accel_noise_seq(RANDOM_SAMPLES * PREDICTION_HORIZON-1);
            // const double sigma_steer = sqrt(steer_cov_);
            // const double sigma_accel = sqrt(accel_cov_);
            const double sigma_steer = steer_cov_;
            const double sigma_accel = accel_cov_;
            curandGenerateNormalDouble(rng_, thrust::raw_pointer_cast(steer_noise_seq.data()), steer_noise_seq.size(), 0.0, sigma_steer);
            curandGenerateNormalDouble(rng_, thrust::raw_pointer_cast(accel_noise_seq.data()), accel_noise_seq.size(), 0.0, sigma_accel);



            // ControlSeqBatch noise_seq_batch(RANDOM_SAMPLES);
            // thrust::device_vector<thrust::device_vector<double>> noise_seq_batch(RANDOM_SAMPLES-1);
            // for (int i = 0; i < RANDOM_SAMPLES; i++)
            // {
            //     thrust::device_vector<double> noise_seq(PREDICTION_HORIZON-1);
            //     // just copy
            //     thrust::for_each(thrust::counting_iterator<int>(0), thrust::counting_iterator<int>(PREDICTION_HORIZON-1), 
            //     // for (int j = 0; j < PREDICTION_HORIZON-1; j++)
            //     // {
            //     //     noise_seq_batch[i][j][0] = steer_noise_seq[i * (PREDICTION_HORIZON-1) + j];
            //     //     noise_seq_batch[i][j][1] = accel_noise_seq[i * (PREDICTION_HORIZON-1) + j];
            //     // }
            // }

            // // generate noise sequence batch by adding noise to previous control sequence
            // ControlSeqBatch noised_control_seq_batch(RANDOM_SAMPLES);
            // thrust::for_each(thrust::counting_iterator<int>(0), thrust::counting_iterator<int>(RANDOM_SAMPLES), add_noise(prev_control_seq, noised_control_seq_batch));
            // thrust::for_each(thrust::counting_iterator<int>(0), thrust::counting_iterator<int>(RANDOM_SAMPLES), add_noise(prev_control_seq, noise_seq_batch , noised_control_seq_batch));
        }

        // void MPPIGPU::predict_state_seq_batch(const ControlSeqBatch &control_seq_batch, const State &init_state, StateSeqBatch *state_seq_batch) const
        // {

        // }

        // MPPIGPU::StateSeq MPPIGPU::predict_state_seq(const ControlSeq &control_seq, const State &init_state) const
        // {
            
        // }

        // void MPPIGPU::calc_state_cost_batch(const StateSeqBatch &state_seq_batch, const grid_map::GridMap &obstacle_map, const grid_map::GridMap &ref_path_map, Weights *cost_batch) const
        // {

        // }

        // double MPPIGPU::state_cost(const StateSeq &state_seq, const grid_map::GridMap &obstacle_map, const grid_map::GridMap &ref_path_map) const
        // {
            
        // }

        // void MPPIGPU::calc_weights_mppi(const Eigen::Matrix<double, RANDOM_SAMPLES, 1> &costs, const ControlSeqBatch &noised_control_seq_batch, const ControlSeq &prev_control_seq, Weights *weights) const
        // {
            
        // }

        ////
    } // namespace cuda
} // namespace mppi
