find_package(Threads REQUIRED)

add_executable(glog_smoke_test glog_smoke_test.cc)
target_link_libraries(glog_smoke_test PRIVATE glog::glog Threads::Threads)
