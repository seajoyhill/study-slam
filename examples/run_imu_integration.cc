#include <gflags/gflags.h>
#include <cstdlib>
#include <glog/logging.h>
#include "io_utils.h"
#include <iostream>

int main(int argc, char** argv) {
    FLAGS_colorlogtostderr = true;
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    if (!FLAGS_logtostderr) {
        FLAGS_logtostderr = 1;
    }

    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <data.txt>";
        google::ShutdownGoogleLogging();
        return 1;
    }

    mloam::TxtIO txt_io(argv[1]);
    txt_io.SetIMUProcessFunc([](const mloam::IMU& imu) {
        std::cout << "IMU time=" << std::setprecision(18) << imu.timestamp << " gyro=" << imu.gyro.transpose()
                  << " acc=" << imu.acc.transpose() << std::endl;
    });
    txt_io.Go();

    google::ShutdownGoogleLogging();
    return EXIT_SUCCESS;
}