// 极简激光建图：从 rosbag 读 PointCloud2，用相邻两帧 NDT 估计位姿，累加并保存 PCD。
// 依赖：ROS1（rosbag、pcl_conversions）、PCL（common/io/filters/registration）。
// 用法: ndt_bag_mapper <file.bag> <out.pcd> [pointcloud_topic]
#include <iostream>
#include <string>

#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

static void Downsample(const PointCloud& in, float leaf, PointCloud* out) {
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(in.makeShared());
  vg.setLeafSize(leaf, leaf, leaf);
  vg.filter(*out);
}

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <file.bag> <out.pcd> [pointcloud_topic]\n";
    return 1;
  }

  const std::string bag_path = argv[1];
  const std::string out_pcd = argv[2];
  const std::string topic_filter = (argc >= 4) ? argv[3] : std::string();

  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& e) {
    std::cerr << "rosbag open failed: " << e.what() << '\n';
    return 1;
  }

  rosbag::View view(bag);

  PointCloud map_global;
  PointCloud prev_raw;
  bool have_prev = false;
  Eigen::Matrix4f T_global_curr = Eigen::Matrix4f::Identity();

  // 体素下采样：建图累积用稍大叶子，NDT 用较小叶子（简单分两套）
  const float voxel_map = 0.3f;
  const float voxel_ndt = 0.2f;
  const float ndt_res = 1.0f;

  size_t frame = 0;
  for (const rosbag::MessageInstance& m : view) {
    if (m.getDataType() != "sensor_msgs/PointCloud2") {
      continue;
    }
    if (!topic_filter.empty() && m.getTopic() != topic_filter) {
      continue;
    }

    sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (!cloud_msg) {
      continue;
    }

    PointCloud current;
    pcl::fromROSMsg(*cloud_msg, current);
    if (current.empty()) {
      continue;
    }

    Eigen::Matrix4f T_prev_curr = Eigen::Matrix4f::Identity();
    if (have_prev) {
      PointCloud target_ds;
      PointCloud source_ds;
      Downsample(prev_raw, voxel_ndt, &target_ds);
      Downsample(current, voxel_ndt, &source_ds);
      if (target_ds.size() < 50 || source_ds.size() < 50) {
        std::cerr << "frame " << frame << ": too few points after downsample, skip NDT\n";
      } else {
        pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
        ndt.setResolution(ndt_res);
        ndt.setInputTarget(target_ds.makeShared());
        ndt.setInputSource(source_ds.makeShared());
        ndt.setMaximumIterations(35);
        ndt.setTransformationEpsilon(0.01);
        ndt.setStepSize(0.1);

        PointCloud aligned;
        ndt.align(aligned);
        if (!ndt.hasConverged()) {
          std::cerr << "frame " << frame << ": NDT not converged, use identity\n";
        } else {
          T_prev_curr = ndt.getFinalTransformation();
        }
      }
    }

    T_global_curr = T_global_curr * T_prev_curr;

    PointCloud current_ds;
    Downsample(current, voxel_map, &current_ds);
    PointCloud transformed;
    pcl::transformPointCloud(current_ds, transformed, T_global_curr);
    map_global += transformed;

    prev_raw = current;
    have_prev = true;
    ++frame;
    if (frame <= 5 || frame % 50 == 0) {
      std::cout << "frame " << frame << " topic=" << m.getTopic() << " map_pts=" << map_global.size() << '\n';
    }
  }

  bag.close();

  if (map_global.empty()) {
    std::cerr << "no point clouds merged\n";
    return 2;
  }

  if (pcl::io::savePCDFileBinaryCompressed(out_pcd, map_global) != 0) {
    std::cerr << "save PCD failed: " << out_pcd << '\n';
    return 1;
  }

  std::cout << "saved " << out_pcd << " points=" << map_global.size() << " frames=" << frame << '\n';
  return 0;
}
