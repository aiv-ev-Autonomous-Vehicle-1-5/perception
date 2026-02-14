#include "lidar_clustering/clustering_component.hpp"
#include "lidar_clustering/dbscan.hpp"

#include <cstring>
#include <unordered_map>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace lidar_clustering
{

// 클러스터 라벨 → RGB 색상 변환 (최대 20색 팔레트, 이후 순환)
static float labelToRgb(int32_t label)
{
  // 노이즈는 어두운 회색
  if (label < 0) {
    uint32_t packed = (50u << 16) | (50u << 8) | 50u;
    float rgb;
    std::memcpy(&rgb, &packed, sizeof(float));
    return rgb;
  }

  static const uint32_t palette[] = {
    0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00, 0xFF00FF,
    0x00FFFF, 0xFF8000, 0x8000FF, 0x00FF80, 0xFF0080,
    0x80FF00, 0x0080FF, 0xFF4040, 0x40FF40, 0x4040FF,
    0xFFAA00, 0xAA00FF, 0x00FFAA, 0xFF00AA, 0xAAFF00,
  };
  static const int palette_size = sizeof(palette) / sizeof(palette[0]);

  uint32_t packed = palette[label % palette_size];
  float rgb;
  std::memcpy(&rgb, &packed, sizeof(float));
  return rgb;
}

ClusteringComponent::ClusteringComponent(const rclcpp::NodeOptions & options)
: Node("clustering_component", options)
{
  eps_ = this->declare_parameter("eps", 0.5);
  min_pts_ = this->declare_parameter("min_pts", 5);
  use_gpu_ = this->declare_parameter("use_gpu", true);
  min_cluster_size_ = this->declare_parameter("min_cluster_size", 5);
  max_cluster_size_ = this->declare_parameter("max_cluster_size", 5000);

  if (use_gpu_) {
    try {
      dbscan_gpu_ = std::make_unique<DBSCANGpu>(50000);
      RCLCPP_INFO(this->get_logger(), "GPU DBSCAN initialized (max 50000 points)");
    } catch (const std::runtime_error & e) {
      RCLCPP_FATAL(this->get_logger(),
        "GPU DBSCAN init failed: %s. Shutting down.", e.what());
      rclcpp::shutdown();
      return;
    }
  }

  auto qos = rclcpp::SensorDataQoS();
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", qos,
    std::bind(&ClusteringComponent::pointCloudCallback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", qos);

  RCLCPP_INFO(this->get_logger(),
    "DBSCAN Clustering initialized: eps=%.2f, min_pts=%d, gpu=%s, cluster_size=[%d,%d]",
    eps_, min_pts_, use_gpu_ ? "true" : "false",
    min_cluster_size_, max_cluster_size_);
}

void ClusteringComponent::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const uint32_t num_points = msg->width * msg->height;
  if (num_points == 0) {
    publisher_->publish(*msg);
    return;
  }

  // Extract x, y, z from input PointCloud2
  std::vector<Point3D> points;
  points.reserve(num_points);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (uint32_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
    points.push_back({*iter_x, *iter_y, *iter_z});
  }

  // Run DBSCAN (GPU or CPU)
  std::vector<int32_t> labels;
  if (use_gpu_ && dbscan_gpu_) {
    labels = dbscan_gpu_->cluster(points, eps_, min_pts_);
  } else {
    labels = DBSCAN::cluster(points, eps_, min_pts_);
  }

  // Count points per cluster and filter by size
  std::unordered_map<int32_t, int> cluster_sizes;
  int32_t noise_count = 0;
  for (uint32_t i = 0; i < num_points; ++i) {
    if (labels[i] == -1) {
      ++noise_count;
    } else {
      ++cluster_sizes[labels[i]];
    }
  }

  // Determine which clusters pass the size filter
  std::unordered_map<int32_t, bool> valid_clusters;
  int valid_cluster_count = 0;
  int filtered_cluster_count = 0;
  for (const auto & [label, size] : cluster_sizes) {
    if (size >= min_cluster_size_ && size <= max_cluster_size_) {
      valid_clusters[label] = true;
      ++valid_cluster_count;
    } else {
      ++filtered_cluster_count;
    }
  }

  // Count valid output points
  uint32_t out_count = 0;
  for (uint32_t i = 0; i < num_points; ++i) {
    if (labels[i] >= 0 && valid_clusters.count(labels[i])) {
      ++out_count;
    }
  }

  // Build output PointCloud2 with x, y, z (FLOAT32) + label (INT32) + rgb (FLOAT32)
  sensor_msgs::msg::PointCloud2 output_msg;
  output_msg.header = msg->header;
  output_msg.height = 1;
  output_msg.width = out_count;
  output_msg.is_bigendian = false;
  output_msg.is_dense = true;

  // Define fields: x, y, z, label, rgb
  sensor_msgs::msg::PointField field;
  field.count = 1;

  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  output_msg.fields.push_back(field);

  field.name = "y";
  field.offset = 4;
  output_msg.fields.push_back(field);

  field.name = "z";
  field.offset = 8;
  output_msg.fields.push_back(field);

  field.name = "label";
  field.offset = 12;
  field.datatype = sensor_msgs::msg::PointField::INT32;
  output_msg.fields.push_back(field);

  field.name = "rgb";
  field.offset = 16;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  output_msg.fields.push_back(field);

  output_msg.point_step = 20;  // 5 * 4 bytes
  output_msg.row_step = output_msg.point_step * out_count;
  output_msg.data.resize(output_msg.row_step);

  // Fill data (skip noise and filtered clusters)
  uint32_t out_idx = 0;
  for (uint32_t i = 0; i < num_points; ++i) {
    if (labels[i] < 0 || !valid_clusters.count(labels[i])) {
      continue;
    }
    const size_t offset = static_cast<size_t>(out_idx) * 20;
    float rgb = labelToRgb(labels[i]);
    std::memcpy(&output_msg.data[offset + 0], &points[i].x, 4);
    std::memcpy(&output_msg.data[offset + 4], &points[i].y, 4);
    std::memcpy(&output_msg.data[offset + 8], &points[i].z, 4);
    std::memcpy(&output_msg.data[offset + 12], &labels[i], 4);
    std::memcpy(&output_msg.data[offset + 16], &rgb, 4);
    ++out_idx;
  }

  publisher_->publish(output_msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "DBSCAN[%s]: %u pts -> %d clusters (%d filtered), %d noise, %u output",
    use_gpu_ ? "GPU" : "CPU",
    num_points, valid_cluster_count, filtered_cluster_count,
    noise_count, out_count);
}

}  // namespace lidar_clustering

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_clustering::ClusteringComponent)
