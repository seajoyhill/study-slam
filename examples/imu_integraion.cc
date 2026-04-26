#include "imu_integration.h"


namespace mloam {

void ImuIntegration::AddIMU(const IMU& imu) {
    if (is_first_imu_) {
        last_timestamp_ = imu.timestamp;
        is_first_imu_ = false;
        return;
    }

    double dt = imu.timestamp - last_timestamp_;
    last_timestamp_ = imu.timestamp;

    Vec3d gyro_unbiased = imu.gyro - bg_;
    Vec3d acc_unbiased = imu.acc - ba_;

    if (dt > 0.0 && dt < 0.1) {
        R_curr_ = R_curr_ * SO3d::exp((gyro_unbiased) * dt);
        v_curr_ += R_curr_ * (acc_unbiased) * dt + g_ * dt; // v = v + at
        p_curr_ += v_curr_ * dt + 0.5 * (R_curr_ * (acc_unbiased) + g_) * dt * dt; // p = p + vt + 0.5at^2
    }
}

}  // namespace mloam