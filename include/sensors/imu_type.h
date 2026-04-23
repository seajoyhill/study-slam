#pragma once
#include <Eigen/Dense>
namespace mloam {
using Vec3d = Eigen::Matrix<double, 3, 1>;
struct IMU {
    IMU() = default;
    IMU(double t, const Vec3d& gyro, const Vec3d& acc)
        : timestamp(t), gyro(gyro), acc(acc) {}

    double timestamp = 0.0;
    Vec3d gyro = Vec3d::Zero();
    Vec3d acc = Vec3d::Zero();
};
}  // namespace mloam