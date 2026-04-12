find_package(Threads REQUIRED)

add_executable(glog_smoke_test glog_smoke_test.cc)
target_link_libraries(glog_smoke_test PRIVATE glog::glog Threads::Threads)

add_executable(pangolin_smoke_test pangolin_smoke_test.cc)
target_link_libraries(pangolin_smoke_test PRIVATE pango_display)

# ROS1 + PCL：系统已安装即可（勿用 catkin COMPONENTS 聚合库，非工作区下 catkin_LIBRARIES 常为空）。
# CMAKE_PREFIX_PATH 含 ROS 前缀，例如: /opt/ros/noetic 或 conda 的 ROS 环境。
option(STUDY_BUILD_ROSBAG_PCL_TEST "Build rosbag+PCL smoke test when deps are found" ON)
if(STUDY_BUILD_ROSBAG_PCL_TEST)
  find_package(rosbag QUIET CONFIG)
  find_package(pcl_conversions QUIET CONFIG)
  find_package(PCL QUIET COMPONENTS common io)
  if(rosbag_FOUND AND pcl_conversions_FOUND AND PCL_FOUND)
    add_executable(rosbag_pcl_smoke_test rosbag_pcl_smoke_test.cc)
    target_include_directories(rosbag_pcl_smoke_test SYSTEM PRIVATE
      ${rosbag_INCLUDE_DIRS}
      ${pcl_conversions_INCLUDE_DIRS}
      ${PCL_INCLUDE_DIRS}
    )
    target_compile_definitions(rosbag_pcl_smoke_test PRIVATE ${PCL_DEFINITIONS})
    target_link_directories(rosbag_pcl_smoke_test PRIVATE
      ${rosbag_LIBRARY_DIRS}
      ${pcl_conversions_LIBRARY_DIRS}
    )
    target_link_libraries(rosbag_pcl_smoke_test PRIVATE
      ${rosbag_LIBRARIES}
      ${pcl_conversions_LIBRARIES}
      ${PCL_LIBRARIES}
    )
    message(STATUS "rosbag_pcl_smoke_test: enabled (rosbag + pcl_conversions + PCL)")
  else()
    message(STATUS "rosbag_pcl_smoke_test: skipped (need rosbag, pcl_conversions, PCL in CMAKE_PREFIX_PATH)")
  endif()
endif()
