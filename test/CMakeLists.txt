find_package(Threads REQUIRED)

add_executable(glog_smoke_test glog_smoke_test.cc)
target_link_libraries(glog_smoke_test PRIVATE glog::glog Threads::Threads)

add_executable(pangolin_smoke_test pangolin_smoke_test.cc)
target_link_libraries(pangolin_smoke_test PRIVATE pango_display)
