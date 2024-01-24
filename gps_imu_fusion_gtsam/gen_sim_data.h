//
// Created by Zhang Zhimeng on 24-1-23.
//

#ifndef MATCH_TEST_GEN_SIM_DATA_H
#define MATCH_TEST_GEN_SIM_DATA_H

#include "type.h"
#include <random>
#include <fstream>

class PreIntegrationOptimizationTest {
public:
    PreIntegrationOptimizationTest() = default;

    // 生成圆弧运动时的IMU数据和GPS数据
    void GenImuData(std::vector<IMUData> &imu_data_vec, std::vector<GPSData> &gps_data_vec,
                    std::vector<NavStateDataMy> &nav_state_gt_data_vec) const {
        // 噪声生成器
        std::random_device rd;
        std::default_random_engine eng(rd());
        std::normal_distribution<double> acc_normal_noise(0.0, imu_acc_noise_std_);
        std::normal_distribution<double> gyro_normal_noise(0.0, imu_gyro_noise_std_);
        std::normal_distribution<double> gps_posi_normal_noise(0.0, gps_posi_noise_std_);
        std::normal_distribution<double> gps_vel_normal_noise(0.0, gps_velocity_noise_std_);
        std::normal_distribution<double> gps_ori_normal_noise(0.0, gps_orientation_noise_std_);

        // 真实加速度
        double acceleration_true = omega_true_ * velocity_true_;

        imu_data_vec.clear();
        gps_data_vec.clear();
        nav_state_gt_data_vec.clear();

        // 根据总时间和步进时间计算总仿真数据量
        int data_size = static_cast<int>(sum_time_ / imu_dt_);

        nav_state_gt_data_vec.reserve(data_size);
        imu_data_vec.reserve(data_size);
        gps_data_vec.reserve(data_size);

        NavStateDataMy last_nav_state_data;
        last_nav_state_data.P_.setZero();
        last_nav_state_data.V_ << 0.0, velocity_true_, 0.0;
        last_nav_state_data.R_.setIdentity();

        for (int i = 0; i < data_size; i++) {
            Eigen::Vector3d acceleration;
            acceleration << acceleration_true, 0.0, 0.0;

            Eigen::Vector3d angular_velocity;
            angular_velocity << 0.0, 0.0, -omega_true_;

            // 生成IMU的仿真数据量
            IMUData imu_data;
            imu_data.linear_acceleration_
                    << acceleration_true + imu_acc_bias_ + acc_normal_noise(eng),
                    imu_acc_bias_ + acc_normal_noise(eng),
                    imu_acc_bias_ + acc_normal_noise(eng);
            imu_data.angular_velocity_
                    << angular_velocity + Vec3d(imu_gyro_bias_ + gyro_normal_noise(eng),
                                                imu_gyro_bias_ + gyro_normal_noise(eng),
                                                imu_gyro_bias_ + gyro_normal_noise(eng));
            imu_data.timestamp_ = static_cast<uint64_t>(i * 0.01 * 1.0e6);
            imu_data.orientation_ = last_nav_state_data.R_;
            imu_data_vec.emplace_back(std::move(imu_data));

            // 生成GPS的测量数据
            if (i % 10 == 0) {
                GPSData gps_data;

                gps_data.local_xyz_true_ = last_nav_state_data.P_;
                gps_data.local_orientation_true_ = last_nav_state_data.R_;
                gps_data.local_xyz_velocity_true_ = last_nav_state_data.V_;

                gps_data.local_xyz_.x() = last_nav_state_data.P_.x() + gps_posi_normal_noise(eng);
                gps_data.local_xyz_.y() = last_nav_state_data.P_.y() + gps_posi_normal_noise(eng);
                gps_data.local_xyz_.z() = last_nav_state_data.P_.z() + gps_posi_normal_noise(eng);
                gps_data.local_xyz_velocity_.x() = last_nav_state_data.V_.x() + gps_vel_normal_noise(eng);
                gps_data.local_xyz_velocity_.y() = last_nav_state_data.V_.y() + gps_vel_normal_noise(eng);
                gps_data.local_xyz_velocity_.z() = last_nav_state_data.V_.z() + gps_vel_normal_noise(eng);

                Vec3d ori_noise(gps_ori_normal_noise(eng),
                                gps_ori_normal_noise(eng),
                                gps_ori_normal_noise(eng));

                gps_data.local_orientation = last_nav_state_data.R_ * SO3Exp(ori_noise);

                gps_data.timestamp_ = static_cast<uint64_t>(i * imu_dt_ * 1.0e6);

                gps_data_vec.emplace_back(std::move(gps_data));
            }
            last_nav_state_data.timestamp_ = static_cast<uint64_t>(i * imu_dt_ * 1.0e6);
            nav_state_gt_data_vec.push_back(last_nav_state_data);

            // 生成真实运动轨迹
            last_nav_state_data.P_ = last_nav_state_data.P_ + last_nav_state_data.V_ * imu_dt_
                                     + 0.5 * imu_dt_ * imu_dt_ * last_nav_state_data.R_ * acceleration;
            last_nav_state_data.V_ = last_nav_state_data.V_ + last_nav_state_data.R_ * acceleration * imu_dt_;
            last_nav_state_data.R_ = last_nav_state_data.R_ * SO3Exp(angular_velocity * imu_dt_);
        }
    }

public:
    double sum_time_ = 60.0; // 总仿真时间
    double imu_dt_ = 0.01; // imu采样时间

    double imu_acc_noise_std_ = 0.01;
    double imu_acc_bias_ = 0.2;

    double imu_gyro_noise_std_ = 0.01;
    double imu_gyro_bias_ = 0.1;

    double imu_bias_rw_noise_std_ = 0.001; // imu bias随机游走的标准差

    double gps_posi_noise_std_ = 1.0;
    double gps_velocity_noise_std_ = 0.1;
    double gps_orientation_noise_std_ = 1.0 * kDegree2Radian; // degree

    double velocity_true_ = 5.0; // velocity norm
    double omega_true_ = 5.0 * kDegree2Radian;
};

#endif //MATCH_TEST_GEN_SIM_DATA_H
