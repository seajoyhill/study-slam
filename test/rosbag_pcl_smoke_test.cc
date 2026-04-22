// 系统已安装 ROS1（rosbag、sensor_msgs、pcl_conversions）与 PCL 时由 CMake
// 编译。 用法: rosbag_pcl_smoke_test <file.bag> [pointcloud_topic] 未指定 topic
// 时处理 bag 内所有 sensor_msgs/PointCloud2。
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <file.bag> [pointcloud_topic]\n";
        return 1;
    }

    const std::string bag_path = argv[1];
    const std::string topic_filter = (argc >= 3) ? argv[2] : std::string();

    rosbag::Bag bag;
    try {
        bag.open(bag_path, rosbag::bagmode::Read);
    } catch (const rosbag::BagException& e) {
        std::cerr << "rosbag open failed: " << e.what() << '\n';
        return 1;
    }

    rosbag::View view(bag);

    size_t n_msg = 0;
    size_t total_pts = 0;
    for (const rosbag::MessageInstance& m : view) {
        if (m.getDataType() != "sensor_msgs/PointCloud2") {
            continue;
        }
        if (!topic_filter.empty() && m.getTopic() != topic_filter) {
            continue;
        }

        sensor_msgs::PointCloud2::ConstPtr cloud =
            m.instantiate<sensor_msgs::PointCloud2>();
        if (!cloud) {
            continue;
        }

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(*cloud, pcl_cloud);

        ++n_msg;
        total_pts += pcl_cloud.size();
        if (n_msg <= 5) {
            std::cout << "msg " << n_msg << " topic=" << m.getTopic()
                      << " width=" << cloud->width
                      << " height=" << cloud->height
                      << " pcl_points=" << pcl_cloud.size() << '\n';
        }
    }

    bag.close();

    std::cout << "rosbag+pcl smoke: PointCloud2 messages=" << n_msg
              << " total_points=" << total_pts << '\n';

    return n_msg > 0 ? 0 : 2;
}
