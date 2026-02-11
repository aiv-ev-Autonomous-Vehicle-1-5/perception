#ifndef VELODYNE_GROUND_SEGMENTATION__GROUND_SEGMENTATION_COMPONENT_HPP_
#define VELODYNE_GROUND_SEGMENTATION__GROUND_SEGMENTATION_COMPONENT_HPP_

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace velodyne_ground_segmentation
{

struct PointInfo
{
  float x, y, z, intensity;
  uint16_t ring;
  float time;
  float distance;
  uint32_t original_index;
};

class GroundSegmentationComponent : public rclcpp::Node
{
public:
  explicit GroundSegmentationComponent(const rclcpp::NodeOptions & options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  // Parameters
  int num_rings_;
  int num_sectors_;
  double sensor_height_;
  double ground_threshold_;
  double initial_ground_height_;
};

}  // namespace velodyne_ground_segmentation

#endif  // VELODYNE_GROUND_SEGMENTATION__GROUND_SEGMENTATION_COMPONENT_HPP_
