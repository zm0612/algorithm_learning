//
// Created by meng on 24-1-17.
//

#ifndef GTSAM_TEST_TYPE_H
#define GTSAM_TEST_TYPE_H

#include <Eigen/Dense>

#define kDegree2Radian (M_PI / 180.0)

using Vec3d = Eigen::Vector3d;
using Mat3d = Eigen::Matrix3d;

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> Hat(const Eigen::MatrixBase<Derived> &v) {
    eigen_assert(v.size() == 3u);

    Eigen::Matrix<typename Derived::Scalar, 3, 3> skew_mat;
    skew_mat.setZero();
    skew_mat(0, 1) = -v(2);
    skew_mat(0, 2) = +v(1);
    skew_mat(1, 2) = -v(0);
    skew_mat(1, 0) = +v(2);
    skew_mat(2, 0) = -v(1);
    skew_mat(2, 1) = +v(0);
    return skew_mat;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SO3Exp(const Eigen::MatrixBase<Derived> &v) {
    eigen_assert(v.size() == 3u);

    Eigen::Matrix<typename Derived::Scalar, 3, 3> R = Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity();
    typename Derived::Scalar theta = v.norm();

    if (theta * theta > std::numeric_limits<typename Derived::Scalar>::epsilon()) {
        Eigen::Matrix<typename Derived::Scalar, 3, 1> v_normalized = v.normalized();
        R = std::cos(theta) * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity()
            + (typename Derived::Scalar(1.0) - std::cos(theta)) * v_normalized *
              v_normalized.transpose() + std::sin(theta) * Hat(v_normalized);
        return R;
    } else {
        return R;
    }
}

struct DataBase {
    uint64_t timestamp_ = 0u; // us
};

struct NavStateDataMy : public DataBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3d P_ = Vec3d::Zero();
    Vec3d V_ = Vec3d::Zero();
    Mat3d R_ = Mat3d::Identity();

    Vec3d bg_ = Vec3d::Zero(); // gyro bias
    Vec3d ba_ = Vec3d::Zero(); // accel bias
};

struct IMUData : public DataBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3d angular_velocity_ = Vec3d::Zero();
    Vec3d linear_acceleration_ = Vec3d::Zero();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
};

struct GPSData : public DataBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double longitude_{0.0};
    double latitude_{0.0};
    double altitude_{0.0};

    Vec3d local_xyz_ = Vec3d::Zero();
    Vec3d local_xyz_velocity_ = Vec3d::Zero();
    Mat3d local_orientation = Mat3d::Identity();

    Vec3d local_xyz_true_ = Vec3d::Zero();
    Vec3d local_xyz_velocity_true_ = Vec3d::Zero();
    Mat3d local_orientation_true_ = Mat3d::Identity();
};

#endif //GTSAM_TEST_TYPE_H
