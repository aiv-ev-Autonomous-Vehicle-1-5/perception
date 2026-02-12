#include "lidar_clustering/clustering_component.hpp"
#include "lidar_clustering/dbscan.hpp"

#include <cstring>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace lidar_clustering
{

ClusteringComponent::ClusteringComponent(const rclcpp::NodeOptions & options)
: Node("clustering_component", options)
{
  eps_ = this->declare_parameter("eps", 0.5);
  min_pts_ = this->declare_parameter("min_pts", 5);
  use_gpu_ = this->declare_parameter("use_gpu", true);

  if (use_gpu_) {
    try {
      dbscan_gpu_ = std::make_unique<DBSCANGpu>(50000);
      RCLCPP_INFO(this->get_logger(), "GPU DBSCAN initialized (max 50000 points)");
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN(this->get_logger(),
        "GPU DBSCAN init failed: %s. Falling back to CPU.", e.what());
      use_gpu_ = false;
    }
  }

  auto qos = rclcpp::SensorDataQoS();
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", qos,
    std::bind(&ClusteringComponent::pointCloudCallback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", qos);

  RCLCPP_INFO(this->get_logger(),
    "DBSCAN Clustering initialized: eps=%.2f, min_pts=%d, gpu=%s",
    eps_, min_pts_, use_gpu_ ? "true" : "false");
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

  // Build output PointCloud2 with x, y, z (FLOAT32) + label (INT32)
  sensor_msgs::msg::PointCloud2 output_msg;
  output_msg.header = msg->header;
  output_msg.height = 1;
  output_msg.width = num_points;
  output_msg.is_bigendian = false;
  output_msg.is_dense = true;

  // Define fields: x, y, z, label
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

  output_msg.point_step = 16;  // 4 * 4 bytes
  output_msg.row_step = output_msg.point_step * num_points;
  output_msg.data.resize(output_msg.row_step);

  // Fill data
  for (uint32_t i = 0; i < num_points; ++i) {
    const size_t offset = static_cast<size_t>(i) * 16;
    std::memcpy(&output_msg.data[offset + 0], &points[i].x, 4);
    std::memcpy(&output_msg.data[offset + 4], &points[i].y, 4);
    std::memcpy(&output_msg.data[offset + 8], &points[i].z, 4);
    std::memcpy(&output_msg.data[offset + 12], &labels[i], 4);
  }

  publisher_->publish(output_msg);

  // Count clusters for logging
  int32_t max_label = -1;
  int32_t noise_count = 0;
  for (const auto & l : labels) {
    if (l == -1) {
      ++noise_count;
    } else if (l > max_label) {
      max_label = l;
    }
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "DBSCAN[%s]: %u pts -> %d clusters, %d noise",
    use_gpu_ ? "GPU" : "CPU",
    num_points, max_label + 1, noise_count);
}

}  // namespace lidar_clustering

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_clustering::ClusteringComponent)
