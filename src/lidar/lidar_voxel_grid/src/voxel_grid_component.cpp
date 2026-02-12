#include "lidar_voxel_grid/voxel_grid_component.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace lidar_voxel_grid
{

VoxelGridComponent::VoxelGridComponent(const rclcpp::NodeOptions & options)
: Node("voxel_grid_component", options)
{
  leaf_size_x_ = this->declare_parameter("leaf_size_x", 0.05);
  leaf_size_y_ = this->declare_parameter("leaf_size_y", 0.05);
  leaf_size_z_ = this->declare_parameter("leaf_size_z", 0.05);

  auto qos = rclcpp::SensorDataQoS();
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", qos,
    std::bind(&VoxelGridComponent::pointCloudCallback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", qos);

  RCLCPP_INFO(this->get_logger(),
    "VoxelGrid initialized with leaf size: %.2f x %.2f x %.2f",
    leaf_size_x_, leaf_size_y_, leaf_size_z_);
}

void VoxelGridComponent::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  if (publisher_->get_subscription_count() == 0 && this->count_subscribers("output") == 0) {
    // No subscribers, skip processing (optional optimization)
    // But for pipeline, we usually process.
  }

  // Convert ROS2 msg -> PCL cloud
  pcl::PCLPointCloud2::Ptr pcl_pc2(new pcl::PCLPointCloud2);
  pcl_conversions::toPCL(*msg, *pcl_pc2);

  // Filter
  pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(pcl_pc2);
  sor.setLeafSize((float)leaf_size_x_, (float)leaf_size_y_, (float)leaf_size_z_);
  sor.filter(*cloud_filtered);

  // Convert PCL cloud -> ROS2 msg
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl_conversions::moveFromPCL(*cloud_filtered, output_msg);
  output_msg.header = msg->header; // Preserve timestamp and frame_id

  publisher_->publish(output_msg);
}

}  // namespace lidar_voxel_grid

RCLCPP_COMPONENTS_REGISTER_NODE(lidar_voxel_grid::VoxelGridComponent)
