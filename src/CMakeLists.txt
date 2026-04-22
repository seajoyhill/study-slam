
add_library(mloam
  utils/io_utils.cc
)

target_include_directories(mloam
  PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(mloam
  PUBLIC
    Eigen3::Eigen
)

# 相邻帧 NDT 建图（可选，与 test/rosbag 目标相同依赖条件）
option(STUDY_BUILD_NDT_BAG_MAPPER "Build ndt_bag_mapper (ROS bag + PCL NDT)" ON)
if(STUDY_BUILD_NDT_BAG_MAPPER)
  find_package(rosbag QUIET CONFIG)
  find_package(pcl_conversions QUIET CONFIG)
  find_package(PCL QUIET COMPONENTS common io filters registration)
  if(rosbag_FOUND AND pcl_conversions_FOUND AND PCL_FOUND)
    add_executable(ndt_bag_mapper ndt_bag_mapper.cc)
    target_include_directories(ndt_bag_mapper SYSTEM PRIVATE
      ${rosbag_INCLUDE_DIRS}
      ${pcl_conversions_INCLUDE_DIRS}
      ${PCL_INCLUDE_DIRS}
    )
    target_compile_definitions(ndt_bag_mapper PRIVATE ${PCL_DEFINITIONS})
    target_link_directories(ndt_bag_mapper PRIVATE
      ${rosbag_LIBRARY_DIRS}
      ${pcl_conversions_LIBRARY_DIRS}
    )
    target_link_libraries(ndt_bag_mapper PRIVATE
      ${rosbag_LIBRARIES}
      ${pcl_conversions_LIBRARIES}
      ${PCL_LIBRARIES}
    )
    message(STATUS "ndt_bag_mapper: enabled")
  else()
    message(STATUS "ndt_bag_mapper: skipped (need rosbag, pcl_conversions, PCL registration in CMAKE_PREFIX_PATH)")
  endif()
endif()

