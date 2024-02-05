#include "type.h"
#include "gen_sim_data.h"
#include "pre_integration_data_preparer.h"

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/TangentPreintegration.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <boost/program_options.hpp>

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>

#include <Eigen/Dense>

#include <fstream>
#include <iostream>

using namespace gtsam;
using namespace std;

using symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)

namespace po = boost::program_options;

std::shared_ptr<PreintegratedCombinedMeasurements::Params>
ImuParams(const PreIntegrationOptimizationTest &pre_integration_optimization_test) {
    // We use the sensor specs to build the noise model for the IMU factor.
    double accel_noise_sigma = pre_integration_optimization_test.imu_acc_noise_std_;
    double gyro_noise_sigma = pre_integration_optimization_test.imu_gyro_noise_std_;
    double accel_bias_rw_sigma = pre_integration_optimization_test.imu_bias_rw_noise_std_;
    double gyro_bias_rw_sigma = pre_integration_optimization_test.imu_bias_rw_noise_std_;
    Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
    Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
    Matrix33 integration_error_cov = I_3x3 * 1e-8;  //由于速度积分位置时带来的积分误差
    Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
    Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
    Matrix66 bias_acc_omega_init = I_6x6 * 1e-5;  // error in the bias used for preintegration

    auto p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);

    // PreintegrationBase params:
    p->accelerometerCovariance = measured_acc_cov;     // acc white noise in continuous
    p->integrationCovariance = integration_error_cov;  // integration uncertainty continuous

    // should be using 2nd order integration
    // PreintegratedRotation params:
    p->gyroscopeCovariance = measured_omega_cov;  // gyro white noise in continuous

    // PreintegrationCombinedMeasurements params:
    p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
    p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
    p->biasAccOmegaInt = bias_acc_omega_init;

    return p;
}

int main() {
    PreIntegrationOptimizationTest pre_integration_optimization_test;

    std::vector<IMUData> imu_data_vec;
    std::vector<GPSData> gps_measures_vec;
    std::vector<NavStateDataMy> nav_state_gt_data_vec;
    pre_integration_optimization_test.GenImuData(imu_data_vec, gps_measures_vec, nav_state_gt_data_vec);

    // 设置运动状态的先验
    Rot3 prior_rotation(SO3::FromMatrix(nav_state_gt_data_vec[0].R_));
    Point3 prior_point(nav_state_gt_data_vec[0].P_);
    Pose3 prior_pose(prior_rotation, prior_point);
    Vector3 prior_velocity(nav_state_gt_data_vec[0].V_);
    imuBias::ConstantBias prior_imu_bias;

    // 设置先验状态的置信度噪声标准差
    noiseModel::Base::shared_ptr prior_pose_noise_model = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.00001, 0.00001, 0.00001, 0.0005, 0.0005, 0.0005).finished()
    );
    noiseModel::Base::shared_ptr prior_velocity_noise_model = noiseModel::Diagonal::Sigmas(
            (Vector(3) << 0.00001, 0.00001, 0.00001).finished()
    );
    noiseModel::Base::shared_ptr prior_bias_prior_noise_model = noiseModel::Diagonal::Sigmas(
            (Vector(6) << 0.2, 0.2, 0.2, 0.1, 0.1, 0.1).finished()
    );;

    // 初始化imu的参数和预积分
    auto p = ImuParams(pre_integration_optimization_test);
    std::shared_ptr<PreintegrationType> preintegrated
            = std::make_shared<PreintegratedImuMeasurements>(p, prior_imu_bias);

    NavState prev_state(prior_pose, prior_velocity);

    double dt = pre_integration_optimization_test.imu_dt_;

    // 生成imu数据准备器，便于根据时间点搜索数据段
    PreIntegrationDataPreparer pre_integration_data_preparer;
    for (const auto &i: imu_data_vec) {
        pre_integration_data_preparer.CacheData(i);
    }

    int correction_count = 0;

    std::ofstream gps_measure_file("../result/gps_measure.txt", std::ios::trunc);
    std::ofstream fuse_path_file("../result/fuse_path.txt", std::ios::trunc);
    std::ofstream gt_path_file("../result/gt_path.txt", std::ios::trunc);
    std::ofstream imu_bias_file("../result/imu_bias.txt", std::ios::trunc);

    imu_bias_file << pre_integration_optimization_test.imu_acc_bias_ << " "
                  << pre_integration_optimization_test.imu_acc_bias_ << " "
                  << pre_integration_optimization_test.imu_acc_bias_ << " "
                  << pre_integration_optimization_test.imu_gyro_bias_ << " "
                  << pre_integration_optimization_test.imu_gyro_bias_ << " "
                  << pre_integration_optimization_test.imu_gyro_bias_ << std::endl;

    // 保存真值轨迹
    for (const auto &nav_state: nav_state_gt_data_vec) {
        Eigen::Quaterniond q(nav_state.R_);
        gt_path_file << std::setprecision(15) << static_cast<double >(nav_state.timestamp_) / 1.0e6 << " "
                     << nav_state.P_.x() << " " << nav_state.P_.y() << " " << nav_state.P_.z() << " "
                     << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }

    for (const auto &gps_measure: gps_measures_vec) {
        Eigen::Quaterniond q(gps_measure.local_orientation);
        gps_measure_file << std::setprecision(15) << static_cast<double >(gps_measure.timestamp_) / 1.0e6 << " "
                         << gps_measure.local_xyz_.x() << " " << gps_measure.local_xyz_.y() << " "
                         << gps_measure.local_xyz_.z() << " "
                         << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }

    for (unsigned int i = 0; i < gps_measures_vec.size() - 1; ++i) {
        // 搜索相邻两个时间点内的IMU数据段
        const auto imu_data_segment = pre_integration_data_preparer.GetDataSegment(gps_measures_vec[i].timestamp_,
                                                                                   gps_measures_vec[i + 1].timestamp_);

        // 对IMU数据进行预积分
        for (unsigned int imu_data_index = 0; imu_data_index < imu_data_segment.size() - 1; ++imu_data_index) {
            preintegrated->integrateMeasurement(imu_data_segment[imu_data_index].linear_acceleration_,
                                                imu_data_segment[imu_data_index].angular_velocity_, dt);
        }

        auto preint_imu = dynamic_cast<const PreintegratedImuMeasurements &>(*preintegrated);

        // 设置待求解的随机变量
        Values initial_values;
        initial_values.insert(X(correction_count), prior_pose);
        initial_values.insert(V(correction_count), prior_velocity);
        initial_values.insert(B(correction_count), prior_imu_bias);

        // 新建非线性因子图
        auto *graph = new NonlinearFactorGraph();

        // 向因子图中增加随机状态量的先验测量和先验的置信度
        graph->addPrior(X(correction_count), prior_pose, prior_pose_noise_model);
        graph->addPrior(V(correction_count), prior_velocity, prior_velocity_noise_model);
        graph->addPrior(B(correction_count), prior_imu_bias, prior_bias_prior_noise_model);

        correction_count++;

        // IMU因子
        ImuFactor imu_factor(X(correction_count - 1),
                             V(correction_count - 1),
                             X(correction_count),
                             V(correction_count),
                             B(correction_count - 1),
                             preint_imu);
        graph->add(imu_factor);

        // MU bias随机游走的因子
        noiseModel::Base::shared_ptr bias_rw_noise_model
                = noiseModel::Isotropic::Sigma(6, pre_integration_optimization_test.imu_bias_rw_noise_std_);
        imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
        graph->add(BetweenFactor<imuBias::ConstantBias>(
                B(correction_count - 1), B(correction_count), zero_bias, bias_rw_noise_model
        ));

        // GPS测量的因子，包括位置和姿态
        double gps_posi_noise = pre_integration_optimization_test.gps_posi_noise_std_ / 5.0;
        double gps_rot_noise = pre_integration_optimization_test.gps_orientation_noise_std_;
        auto gps_measure_noise = noiseModel::Diagonal::Sigmas(
                (Vector(6) << gps_rot_noise, gps_rot_noise, gps_rot_noise,
                        gps_posi_noise, gps_posi_noise, gps_posi_noise).finished());
        Rot3 gps_prior_rotation(SO3::FromMatrix(gps_measures_vec[i].local_orientation));
        Point3 gps_prior_point(gps_measures_vec[i].local_xyz_);
        Pose3 gps_prior_pose(gps_prior_rotation, gps_prior_point);
        PriorFactor<Pose3> gps_pose(X(correction_count), gps_prior_pose, gps_measure_noise);
        graph->add(gps_pose);

        // 使用IMU的预测值作为后一个时刻的状态的初始值
        NavState prop_state = preintegrated->predict(prev_state, prior_imu_bias);
        initial_values.insert(X(correction_count), prop_state.pose());
        initial_values.insert(V(correction_count), prop_state.v());
        initial_values.insert(B(correction_count), prior_imu_bias);

        // 执行非线性优化
        GaussNewtonOptimizer optimizer(*graph, initial_values);
        Values result = optimizer.optimize();

        // 获取优化之后的结果
        prior_pose = result.at<Pose3>(X(correction_count));
        prior_velocity = result.at<Vector3>(V(correction_count));
        prior_imu_bias = result.at<imuBias::ConstantBias>(B(correction_count));
        preintegrated->resetIntegrationAndSetBias(prior_imu_bias);

        // 对优化状态执行边缘化操作，将当前时刻的状态变成下一次优化的先验
        // 通过雅克比矩阵，以及信息矩阵从联合概率中中获得某个状态边缘概率
        Marginals marginals(*graph, result);
        Matrix curr_pose_cov = marginals.marginalCovariance(X(correction_count));
        Matrix curr_velocity_cov = marginals.marginalCovariance(V(correction_count));
        Matrix curr_bias_cov = marginals.marginalCovariance(B(correction_count));

        // 更下下一个时刻先验的协方差
        prior_pose_noise_model = noiseModel::Gaussian::Covariance(curr_pose_cov);
        prior_velocity_noise_model = noiseModel::Gaussian::Covariance(curr_velocity_cov);
        prior_bias_prior_noise_model = noiseModel::Gaussian::Covariance(curr_bias_cov);

        prev_state = NavState(result.at<Pose3>(X(correction_count)),
                              result.at<Vector3>(V(correction_count)));

        Rot3 fusion_rot = prev_state.pose().rotation();
        Eigen::Quaterniond fusion_q = fusion_rot.toQuaternion();

        Vector3 fusion_position = prev_state.pose().translation();
        Vector3 fusion_velocity = prev_state.velocity();
        Vector3 position_error = fusion_position - gps_measures_vec[i].local_xyz_true_;
        Vector3 velocity_error = fusion_velocity - gps_measures_vec[i].local_xyz_velocity_true_;

        Vec3d acc_bias_true(1.0, 1.0, 1.0);
        Vec3d gyro_bias_true(1.0, 1.0, 1.0);
        acc_bias_true *= pre_integration_optimization_test.imu_acc_bias_;
        gyro_bias_true *= pre_integration_optimization_test.imu_gyro_bias_;

        const Vec3d acc_bias_fusion = prior_imu_bias.vector().head(3);
        const Vec3d gyro_bias_fusion = prior_imu_bias.vector().tail(3);

        std::cout << "timestamp: " << static_cast<double >(gps_measures_vec[i].timestamp_) / 1.0e6 << std::endl;
        std::cout << "Position error:" << position_error.norm() << std::endl;
        std::cout << "Velocity error:" << velocity_error.norm() << std::endl;
        std::cout << "acc gyro bias: " << prior_imu_bias.vector().transpose() << std::endl;
        std::cout << "acc bias error: " << (gyro_bias_fusion - gyro_bias_true).transpose() << std::endl;
        std::cout << "gyro bias error: " << (acc_bias_fusion - acc_bias_true).transpose() << std::endl << std::endl;

        fuse_path_file << std::setprecision(15) << static_cast<double >(gps_measures_vec[i].timestamp_) / 1.0e6 << " "
                       << fusion_position.x() << " " << fusion_position.y() << " " << fusion_position.z() << " "
                       << fusion_q.x() << " " << fusion_q.y() << " " << fusion_q.z() << " " << fusion_q.w()
                       << std::endl;

        imu_bias_file << acc_bias_fusion.x() << " "
                      << acc_bias_fusion.y() << " "
                      << acc_bias_fusion.z() << " "
                      << gyro_bias_fusion.x() << " "
                      << gyro_bias_fusion.y() << " "
                      << gyro_bias_fusion.z() << std::endl;
    }

    return 0;
}