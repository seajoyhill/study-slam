#pragma once
#include <fstream>
#include <functional>
#include <string>
#include <utility>

#include "sensors/imu_type.h"

namespace mloam {
class TxtIO {
   public:
    TxtIO(const std::string& file_path) : fin(file_path) {}

    /// 定义回调函数
    using IMUProcessFuncType = std::function<void(const IMU&)>;
    // using OdomProcessFuncType = std::function<void(const Odom&)>;
    // using GNSSProcessFuncType = std::function<void(const GNSS&)>;

    TxtIO& SetIMUProcessFunc(IMUProcessFuncType imu_proc) {
        imu_proc_ = std::move(imu_proc);
        return *this;
    }

    // TxtIO& SetOdomProcessFunc(OdomProcessFuncType odom_proc) {
    //     odom_proc_ = std::move(odom_proc);
    //     return *this;
    // }

    // TxtIO& SetGNSSProcessFunc(GNSSProcessFuncType gnss_proc) {
    //     gnss_proc_ = std::move(gnss_proc);
    //     return *this;
    // }

    // 遍历文件内容，调用回调函数
    void Go();

   private:
    std::ifstream fin;
    IMUProcessFuncType imu_proc_;
    // OdomProcessFuncType odom_proc_;
    // GNSSProcessFuncType gnss_proc_;
};

}  // namespace mloam