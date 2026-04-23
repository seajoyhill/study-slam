#include <gflags/gflags.h>
#include <atomic>
#include <cstdlib>
#include <deque>
#include <exception>
#include <glog/logging.h>
#include "io_utils.h"
#include <iostream>
#include <mutex>
#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/gl/glinclude.h>
#include <pangolin/gl/opengl_render_state.h>
#include <pangolin/handler/handler.h>
#include <thread>
#include <vector>

namespace {

class ImuViewer {
public:
    ImuViewer()
        : camera_state_(pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240,
                                                   0.2, 100),
                        pangolin::ModelViewLookAt(0, -1, -3, 0, 0, 0,
                                                  pangolin::AxisY)) {
        pangolin::CreateWindowAndBind("IMU Visualization", 640, 480);
        glEnable(GL_DEPTH_TEST);

        view_ = &pangolin::CreateDisplay()
                     .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0,
                                -640.0f / 480.0f)
                     .SetHandler(new pangolin::Handler3D(camera_state_));
    }

    void PushImu(const mloam::IMU& imu) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (pending_imus_.size() >= kMaxPendingImus) {
            pending_imus_.pop_front();
        }
        pending_imus_.push_back(imu);
    }

    void Run(const std::atomic<bool>& reader_done) {
        while (!pangolin::ShouldQuit() || !reader_done.load()) {
            ConsumePendingImus();
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            view_->Activate(camera_state_);
            DrawImuPoints();
            pangolin::FinishFrame();
        }
    }

private:
    void ConsumePendingImus() {
        std::deque<mloam::IMU> local_pending;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (pending_imus_.empty()) {
                return;
            }
            local_pending.swap(pending_imus_);
        }

        for (const auto& imu : local_pending) {
            timestamps_.push_back(imu.timestamp);
            gyro_data_.push_back(imu.gyro);
            acc_data_.push_back(imu.acc);
        }
    }

    void DrawImuPoints() const {
        glPointSize(5.0);
        glBegin(GL_POINTS);
        glColor3f(1.0, 0.0, 0.0);
        for (const auto& gyro : gyro_data_) {
            glVertex3d(gyro.x(), gyro.y(), gyro.z());
        }
        glColor3f(0.0, 1.0, 0.0);
        for (const auto& acc : acc_data_) {
            glVertex3d(acc.x(), acc.y(), acc.z());
        }
        glEnd();
    }

    pangolin::OpenGlRenderState camera_state_;
    pangolin::View* view_ = nullptr;
    std::mutex queue_mutex_;
    std::deque<mloam::IMU> pending_imus_;
    std::vector<double> timestamps_;
    std::vector<mloam::Vec3d> gyro_data_;
    std::vector<mloam::Vec3d> acc_data_;

    static constexpr std::size_t kMaxPendingImus = 4096;
};

}  // namespace

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
    ImuViewer viewer;
    std::exception_ptr reader_error;
    std::atomic<bool> reader_done{false};

    txt_io.SetIMUProcessFunc([&](const mloam::IMU& imu) {
        viewer.PushImu(imu);
    });

    std::thread reader_thread([&]() {
        try {
            txt_io.Go();
        } catch (...) {
            reader_error = std::current_exception();
        }
        reader_done.store(true);
        std::cout << "Finished reading IMU data." << std::endl;
    });

    viewer.Run(reader_done);
    reader_thread.join();

    if (reader_error) {
        google::ShutdownGoogleLogging();
        std::rethrow_exception(reader_error);
    }

    google::ShutdownGoogleLogging();
    return EXIT_SUCCESS;
}