// Demo: read /odom translation from rosbag and visualize rslidar(frame from /velodyne_points)
// moving in map frame with Pangolin. Orientation is intentionally ignored.
//
// Usage:
//   rosbag_pangolin_loam_demo <file.bag> [/odom_topic] [/velodyne_points_topic]
//
// Supported odom topic message types:
//   nav_msgs/Odometry
//   geometry_msgs/PoseStamped
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pangolin/display/display.h>
#include <pangolin/display/view.h>
#include <pangolin/gl/gldraw.h>
#include <pangolin/gl/glinclude.h>
#include <pangolin/gl/opengl_render_state.h>
#include <pangolin/handler/handler.h>

namespace {

struct PoseSample {
  double t_sec;
  float x;
  float y;
  float z;
  float qx;
  float qy;
  float qz;
  float qw;
};

struct CloudSample {
  double t_sec;
  std::vector<float> xyz;
  std::vector<float> rgb;
};

constexpr std::size_t kMaxCloudPointsPerFrame = 20000;
constexpr std::size_t kCloudFrameStride = 3;
constexpr bool kEnableIntensityDebugLog = true;
constexpr bool kInvertRvizRainbow = false;
constexpr float kCameraTranslationScale = 0.02f;
constexpr float kCameraZoomFraction = 0.10f;
constexpr double kTargetFrameRate = 60.0;

void ApplyQuaternionRotation(float qx, float qy, float qz, float qw) {
  const float n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  if (n < 1e-8f) {
    return;
  }
  qx /= n;
  qy /= n;
  qz /= n;
  qw /= n;

  const float angle = 2.0f * std::acos(std::max(-1.0f, std::min(1.0f, qw)));
  const float s = std::sqrt(std::max(0.0f, 1.0f - qw * qw));
  if (s < 1e-6f || angle < 1e-6f) {
    return;
  }

  const float ax = qx / s;
  const float ay = qy / s;
  const float az = qz / s;
  glRotatef(angle * 180.0f / static_cast<float>(M_PI), ax, ay, az);
}

// ROS FLU convention:
//   +X forward (red), +Y left (green), +Z up (blue).
void DrawRosAxis(float s) {
  const float lines[] = {
      0.f, 0.f, 0.f, s,   0.f, 0.f,  // +X forward
      0.f, 0.f, 0.f, 0.f, s,   0.f,  // +Y left
      0.f, 0.f, 0.f, 0.f, 0.f, s     // +Z up
  };
  const float colors[] = {
      1.f, 0.f, 0.f, 1.f, 0.f, 0.f,  // X red (forward)
      0.f, 1.f, 0.f, 0.f, 1.f, 0.f,  // Y green (left)
      0.f, 0.f, 1.f, 0.f, 0.f, 1.f   // Z blue (up)
  };
  pangolin::glDrawColoredVertices<float, float>(6, lines, colors, GL_LINES, 3, 3);
}

std::vector<float> BuildTrajectoryVertices(const std::vector<PoseSample>& poses) {
  std::vector<float> vertices;
  vertices.reserve(poses.size() * 3);
  for (const PoseSample& pose : poses) {
    vertices.push_back(pose.x);
    vertices.push_back(pose.y);
    vertices.push_back(pose.z);
  }
  return vertices;
}

void DrawTrajectory(const std::vector<float>& vertices, std::size_t end_idx) {
  const std::size_t point_count = vertices.size() / 3;
  if (point_count < 2 || end_idx < 1) {
    return;
  }
  const std::size_t draw_points = std::min(end_idx + 1, point_count);
  glColor3f(1.f, 0.85f, 0.2f);
  pangolin::glDrawVertices<float>(static_cast<GLsizei>(draw_points), vertices.data(), GL_LINE_STRIP,
                                  3);
}

void DrawPointCloud(const std::vector<float>& xyz, const std::vector<float>& rgb) {
  if (xyz.empty() || rgb.empty() || xyz.size() != rgb.size()) {
    return;
  }
  glPointSize(1.5f);
  pangolin::glDrawColoredVertices<float, float>(static_cast<GLsizei>(xyz.size() / 3), xyz.data(),
                                                rgb.data(), GL_POINTS, 3, 3);
}

const sensor_msgs::PointField* FindIntensityField(const sensor_msgs::PointCloud2& cloud) {
  for (const auto& f : cloud.fields) {
    if (f.name == "intensity" || f.name == "intensities") {
      return &f;
    }
  }
  return nullptr;
}

float ReadFieldAsFloat(const uint8_t* p, uint8_t datatype) {
  switch (datatype) {
    case sensor_msgs::PointField::INT8:
      return static_cast<float>(*reinterpret_cast<const int8_t*>(p));
    case sensor_msgs::PointField::UINT8:
      return static_cast<float>(*reinterpret_cast<const uint8_t*>(p));
    case sensor_msgs::PointField::INT16:
      return static_cast<float>(*reinterpret_cast<const int16_t*>(p));
    case sensor_msgs::PointField::UINT16:
      return static_cast<float>(*reinterpret_cast<const uint16_t*>(p));
    case sensor_msgs::PointField::INT32:
      return static_cast<float>(*reinterpret_cast<const int32_t*>(p));
    case sensor_msgs::PointField::UINT32:
      return static_cast<float>(*reinterpret_cast<const uint32_t*>(p));
    case sensor_msgs::PointField::FLOAT32:
      return *reinterpret_cast<const float*>(p);
    case sensor_msgs::PointField::FLOAT64:
      return static_cast<float>(*reinterpret_cast<const double*>(p));
    default:
      return std::numeric_limits<float>::quiet_NaN();
  }
}

void IntensityToRvizRainbow(float value, float* r, float* g, float* b) {
  value = std::max(0.0f, std::min(1.0f, value));

  const float band = value * 5.0f + 1.0f;
  const int idx = static_cast<int>(std::floor(band));
  float frac = band - static_cast<float>(idx);
  if ((idx & 1) == 0) {
    frac = 1.0f - frac;
  }
  const float inv = 1.0f - frac;

  if (idx <= 1) {
    *r = inv;
    *g = 0.0f;
    *b = 1.0f;
  } else if (idx == 2) {
    *r = 0.0f;
    *g = inv;
    *b = 1.0f;
  } else if (idx == 3) {
    *r = 0.0f;
    *g = 1.0f;
    *b = inv;
  } else if (idx == 4) {
    *r = inv;
    *g = 1.0f;
    *b = 0.0f;
  } else {
    *r = 1.0f;
    *g = inv;
    *b = 0.0f;
  }
}

CloudSample BuildCloudSample(const sensor_msgs::PointCloud2& cloud, std::size_t cloud_seq) {
  CloudSample sample;
  sample.t_sec = cloud.header.stamp.toSec();

  const std::size_t total_pts = static_cast<std::size_t>(cloud.width) * cloud.height;
  const std::size_t keep_pts = std::min(total_pts, kMaxCloudPointsPerFrame);
  const std::size_t step = std::max<std::size_t>(1, total_pts / std::max<std::size_t>(1, keep_pts));
  sample.xyz.reserve(keep_pts * 3);
  sample.rgb.reserve(keep_pts * 3);
  std::vector<float> intensities;
  intensities.reserve(keep_pts);

  const sensor_msgs::PointField* intensity_field = FindIntensityField(cloud);
  const bool has_intensity = (intensity_field != nullptr);
  const std::size_t intensity_offset = has_intensity ? intensity_field->offset : 0;
  const uint8_t intensity_datatype = has_intensity ? intensity_field->datatype : 0;
  float i_min = std::numeric_limits<float>::infinity();
  float i_max = -std::numeric_limits<float>::infinity();
  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");
  for (std::size_t i = 0; i < total_pts; ++i, ++it_x, ++it_y, ++it_z) {
    if (i % step != 0) {
      continue;
    }
    const float x = *it_x;
    const float y = *it_y;
    const float z = *it_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    sample.xyz.push_back(x);
    sample.xyz.push_back(y);
    sample.xyz.push_back(z);
    float inten = 0.0f;
    if (has_intensity && (i * cloud.point_step + intensity_offset) < cloud.data.size()) {
      const uint8_t* ptr = &cloud.data[i * cloud.point_step + intensity_offset];
      inten = ReadFieldAsFloat(ptr, intensity_datatype);
      if (!std::isfinite(inten)) {
        inten = 0.0f;
      }
    }
    intensities.push_back(inten);
    i_min = std::min(i_min, inten);
    i_max = std::max(i_max, inten);
    if (sample.xyz.size() / 3 >= kMaxCloudPointsPerFrame) {
      break;
    }
  }

  if (!sample.xyz.empty()) {
    const float norm_min = i_min;
    const float norm_max = i_max;
    const float denom = (norm_max > norm_min) ? (norm_max - norm_min) : 1.0f;

    std::size_t n_blue = 0;
    std::size_t n_red = 0;
    for (float inten : intensities) {
      float r = 0.85f, g = 0.9f, b = 0.95f;
      if (has_intensity) {
        float normalized = (inten - norm_min) / denom;
        normalized = std::max(0.0f, std::min(1.0f, normalized));
        float rviz_value = 1.0f - normalized;
        if (kInvertRvizRainbow) {
          rviz_value = 1.0f - rviz_value;
        }
        IntensityToRvizRainbow(rviz_value, &r, &g, &b);
        if (rviz_value < 0.33f) {
          ++n_blue;
        }
        if (rviz_value > 0.66f) {
          ++n_red;
        }
      }
      sample.rgb.push_back(r);
      sample.rgb.push_back(g);
      sample.rgb.push_back(b);
    }

    // if (kEnableIntensityDebugLog && has_intensity && (cloud_seq <= 3 || cloud_seq % 100 == 0)) {
    //   const float blue_ratio = static_cast<float>(n_blue) / static_cast<float>(intensities.size());
    //   const float red_ratio = static_cast<float>(n_red) / static_cast<float>(intensities.size());
    //   std::cout << "[intensity dbg] cloud=" << cloud_seq << " n=" << intensities.size()
    //             << " raw[min,max]=(" << i_min << ", " << i_max << ")"
    //             << " norm[min,max]=(" << norm_min << ", " << norm_max << ")"
    //             << " blue_ratio=" << blue_ratio << " red_ratio=" << red_ratio << '\n';
    // }
  }
  return sample;
}

bool ReadTrajectory(const std::string& bag_path, const std::string& odom_topic,
                    const std::string& velodyne_topic, std::vector<PoseSample>* poses,
                    std::vector<CloudSample>* clouds, std::string* rslidar_frame_id) {
  poses->clear();
  clouds->clear();
  rslidar_frame_id->clear();

  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (const rosbag::BagException& e) {
    std::cerr << "rosbag open failed: " << e.what() << '\n';
    return false;
  }

  rosbag::View view(bag);
  std::size_t n_odom = 0;
  std::size_t n_pose = 0;
  std::size_t n_cloud = 0;
  std::size_t n_cloud_kept = 0;
  std::string first_odom_child_frame;

  for (const rosbag::MessageInstance& m : view) {
    if (m.getTopic() == velodyne_topic && m.getDataType() == "sensor_msgs/PointCloud2") {
      sensor_msgs::PointCloud2::ConstPtr cloud = m.instantiate<sensor_msgs::PointCloud2>();
      if (cloud) {
        ++n_cloud;
        if (rslidar_frame_id->empty() && !cloud->header.frame_id.empty()) {
          *rslidar_frame_id = cloud->header.frame_id;
        }
        if ((n_cloud - 1) % kCloudFrameStride == 0) {
          clouds->push_back(BuildCloudSample(*cloud, n_cloud));
          ++n_cloud_kept;
        }
      }
    }

    if (m.getTopic() != odom_topic) {
      continue;
    }

    if (m.getDataType() == "nav_msgs/Odometry") {
      nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();
      if (!odom) {
        continue;
      }
      if (first_odom_child_frame.empty()) {
        first_odom_child_frame = odom->child_frame_id;
      }
      const auto& p = odom->pose.pose.position;
      const auto& q = odom->pose.pose.orientation;
      poses->push_back(PoseSample{odom->header.stamp.toSec(), static_cast<float>(p.x),
                                  static_cast<float>(p.y), static_cast<float>(p.z),
                                  static_cast<float>(q.x), static_cast<float>(q.y),
                                  static_cast<float>(q.z), static_cast<float>(q.w)});
      ++n_odom;
      continue;
    }

    if (m.getDataType() == "geometry_msgs/PoseStamped") {
      geometry_msgs::PoseStamped::ConstPtr pose = m.instantiate<geometry_msgs::PoseStamped>();
      if (!pose) {
        continue;
      }
      const auto& p = pose->pose.position;
      const auto& q = pose->pose.orientation;
      poses->push_back(PoseSample{pose->header.stamp.toSec(), static_cast<float>(p.x),
                                  static_cast<float>(p.y), static_cast<float>(p.z),
                                  static_cast<float>(q.x), static_cast<float>(q.y),
                                  static_cast<float>(q.z), static_cast<float>(q.w)});
      ++n_pose;
      continue;
    }
  }

  bag.close();

  std::cout << "Loaded odom samples from " << odom_topic << ": " << poses->size()
            << " (Odometry=" << n_odom
            << ", PoseStamped=" << n_pose << ")\n";
  std::cout << "Loaded point clouds from " << velodyne_topic << ": total=" << n_cloud
            << ", kept=" << n_cloud_kept << ", stride=" << kCloudFrameStride
            << ", max_points/frame=" << kMaxCloudPointsPerFrame << "\n";
  if (!first_odom_child_frame.empty()) {
    std::cout << "Detected odom child_frame_id: " << first_odom_child_frame << '\n';
  }
  if (!rslidar_frame_id->empty()) {
    std::cout << "Detected " << velodyne_topic << " frame_id: " << *rslidar_frame_id << '\n';
  } else {
    std::cout << "No frame_id detected from " << velodyne_topic
              << "; using logical frame name rslidar.\n";
    *rslidar_frame_id = "rslidar";
  }
  std::cout << "Visualization is map <- " << *rslidar_frame_id
            << " pose (translation + orientation) from " << odom_topic << ".\n";

  if (poses->empty()) {
    return false;
  }

  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0]
              << " <file.bag> [/odom_topic] [/velodyne_points_topic]\n";
    return 1;
  }

  const std::string bag_path = argv[1];
  const std::string odom_topic = (argc >= 3) ? argv[2] : std::string("/odom");
  const std::string velodyne_topic = (argc >= 4) ? argv[3] : std::string("/velodyne_points");

  std::vector<PoseSample> poses;
  std::vector<CloudSample> clouds;
  std::string rslidar_frame_id;
  if (!ReadTrajectory(bag_path, odom_topic, velodyne_topic, &poses, &clouds, &rslidar_frame_id)) {
    std::cerr << "No valid odom translation samples found on topic " << odom_topic << ".\n";
    return 2;
  }
  const std::vector<float> trajectory_vertices = BuildTrajectoryVertices(poses);

  constexpr int w = 1280;
  constexpr int h = 720;
  pangolin::CreateWindowAndBind("rosbag_odom_pangolin_demo", w, h);
  glEnable(GL_DEPTH_TEST);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(w, h, 700, 700, w / 2.0, h / 2.0, 0.1, 10000),
      pangolin::ModelViewLookAt(-20, -20, 12, 0, 0, 0, pangolin::AxisZ));

  pangolin::Handler3D handler(s_cam, pangolin::AxisNone, kCameraTranslationScale,
                              kCameraZoomFraction);
  pangolin::View& d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, 0.0, 1.0, -static_cast<float>(w) / h)
                              .SetHandler(&handler);

  const double t0 = poses.front().t_sec;
  const double tN = poses.back().t_sec;
  const double duration = (tN > t0) ? (tN - t0) : 0.0;

  std::size_t idx = 0;
  std::size_t cloud_idx = 0;
  const double speed = 1.0;
  const auto frame_interval = std::chrono::duration<double>(1.0 / kTargetFrameRate);
  const auto play_start = std::chrono::steady_clock::now();
  auto next_frame_time = play_start;

  std::cout << "Playing map <- " << rslidar_frame_id
            << " translation from " << odom_topic
            << ". Controls: mouse drag/zoom, ESC to exit.\n";

  while (!pangolin::ShouldQuit()) {
    const auto now = std::chrono::steady_clock::now();
    double play_sec = std::chrono::duration<double>(now - play_start).count() * speed;
    if (duration > 0.0 && play_sec > duration) {
      play_sec = duration;
    }

    while (idx + 1 < poses.size() && (poses[idx + 1].t_sec - t0) <= play_sec) {
      ++idx;
    }
    while (cloud_idx + 1 < clouds.size() && clouds[cloud_idx + 1].t_sec <= poses[idx].t_sec) {
      ++cloud_idx;
    }

    glClearColor(0.06f, 0.08f, 0.11f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    d_cam.Activate(s_cam);

    // map frame axis at origin, using ROS FLU convention.
    DrawRosAxis(2.0f);

    // trajectory from odom translation
    DrawTrajectory(trajectory_vertices, idx);

    // rslidar frame transformed by odom pose (translation + orientation)
    glPushMatrix();
    glTranslatef(poses[idx].x, poses[idx].y, poses[idx].z);
    ApplyQuaternionRotation(poses[idx].qx, poses[idx].qy, poses[idx].qz, poses[idx].qw);
    if (!clouds.empty()) {
      DrawPointCloud(clouds[cloud_idx].xyz, clouds[cloud_idx].rgb);
    }
    DrawRosAxis(1.0f);
    glPopMatrix();

    pangolin::FinishFrame();

    next_frame_time += std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_interval);
    const auto sleep_until = next_frame_time;
    const auto after_render = std::chrono::steady_clock::now();
    if (sleep_until > after_render) {
      std::this_thread::sleep_until(sleep_until);
    } else {
      next_frame_time = after_render;
    }

    if (duration > 0.0 && play_sec >= duration) {
      // Keep final frame visible; user can close window manually.
    }
  }

  return 0;
}
