// 验证 thirdparty/glog 与 gflags 联用：命令行解析 + 日志输出。
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <cstdlib>
#include <thread>

DEFINE_bool(log_smoke_extra, false, "若指定，则多打一行 INFO 日志");

int main(int argc, char* argv[]) {
  // 默认开启 stderr 彩色日志；命令行 --colorlogtostderr=false 可关闭（须在 Parse 之前设默认）
  FLAGS_colorlogtostderr = true;
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

  // CHECK 失败时的日志前缀含发起线程的 std::thread::id（与主线程不同即可对比）；
  // 同一进程内 Linux/macOS 的 PID 相同，glog 默认前缀不单独打印 PID。
  std::thread fail_checker([] {
    CHECK_EQ(2, 3) << "CHECK_EQ should fail (worker thread)";
  });
  fail_checker.join();

  google::ShutdownGoogleLogging();
  return EXIT_SUCCESS;
}
