#include "sensors/imu_type.h"
#include "sophus/se2.hpp"
#include "sophus/se3.hpp"

namespace mloam {

class ImuIntegration {
    using Vec3d = mloam::Vec3d;
    using SO3d = Sophus::SO3d;
public:
    ImuIntegration(const Vec3d& gravity, const Vec3d& init_bg,
                   const Vec3d& init_ba)
        : g_(gravity), bg_(init_bg), ba_(init_ba) {}

    void AddIMU(const IMU& imu);

private:
    bool is_first_imu_ = true;
    double last_timestamp_ = 0.0;
    Vec3d g_ = Vec3d(0, 0, -9.81);
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();

    SO3d R_curr_;
    Vec3d v_curr_ = Vec3d::Zero();
    Vec3d p_curr_ = Vec3d::Zero();
};

}  // namespace mloam