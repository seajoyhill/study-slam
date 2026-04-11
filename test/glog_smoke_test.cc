// 验证 thirdparty/glog 与 gflags 联用：命令行解析 + 日志输出。
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <cstdlib>

DEFINE_bool(log_smoke_extra, false, "若指定，则多打一行 INFO 日志");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  // 与 glog 内置 gflags 一致，也可用命令行 --logtostderr
  if (!FLAGS_logtostderr) {
    FLAGS_logtostderr = 1;
  }

  LOG(INFO) << "glog smoke test: INFO";
  LOG(WARNING) << "glog smoke test: WARNING";

  if (FLAGS_log_smoke_extra) {
    LOG(INFO) << "glog smoke test: extra line (--log_smoke_extra)";
  }

  CHECK_EQ(2, 1 + 1) << "CHECK_EQ should succeed";

  google::ShutdownGoogleLogging();
  return EXIT_SUCCESS;
}
