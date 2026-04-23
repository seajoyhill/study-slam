#include "sensors/imu_type.h"

namespace mloam {

class ImuIntegration {
    using Vec3d = mloam::Vec3d;

public:
    ImuIntegration(const Vec3d& gravity, const Vec3d& init_bg,
                   const Vec3d& init_ba)
        : g_(gravity), bg_(init_bg), ba_(init_ba) {}

    void AddIMU(const IMU& imu) {
        if (is_first_imu_) {
            last_timestamp_ = imu.timestamp;
            is_first_imu_ = false;
            return;
        }

        double dt = imu.timestamp - last_timestamp_;
        last_timestamp_ = imu.timestamp;

        Vec3d gyro_unbiased = imu.gyro - bg_;
        Vec3d acc_unbiased = imu.acc - ba_;

        // 这里可以添加积分计算，更新 R_curr_, v_curr_, p_curr_
    }

private:
    bool is_first_imu_ = true;
    double last_timestamp_ = 0.0;
    Vec3d g_ = Vec3d(0, 0, -9.81);
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();
    Vec3d R_curr_ = Vec3d::Zero();
    Vec3d v_curr_ = Vec3d::Zero();
    Vec3d p_curr_ = Vec3d::Zero();
};

}  // namespace mloam