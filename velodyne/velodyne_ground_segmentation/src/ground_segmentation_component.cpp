#include "velodyne_ground_segmentation/ground_segmentation_component.hpp"

#include <cmath>
#include <algorithm>
#include <cstring>
#include <numeric>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace velodyne_ground_segmentation
{

GroundSegmentationComponent::GroundSegmentationComponent(const rclcpp::NodeOptions & options)
: Node("ground_segmentation_component", options)
{
  num_rings_ = this->declare_parameter("num_rings", 16);
  num_sectors_ = this->declare_parameter("num_sectors", 120);
  sensor_height_ = this->declare_parameter("sensor_height", 1.8);
  ground_threshold_ = this->declare_parameter("ground_threshold", 0.3);
  initial_ground_height_ = this->declare_parameter("initial_ground_height", -1.5);

  auto qos = rclcpp::SensorDataQoS();
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", qos,
    std::bind(&GroundSegmentationComponent::pointCloudCallback, this, std::placeholders::_1));

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", qos);

  RCLCPP_INFO(this->get_logger(),
    "GroundSegmentation initialized: rings=%d, sectors=%d, sensor_height=%.2f, "
    "ground_threshold=%.2f, initial_ground_height=%.2f",
    num_rings_, num_sectors_, sensor_height_, ground_threshold_, initial_ground_height_);
}

void GroundSegmentationComponent::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const uint32_t num_points = msg->width * msg->height;
  if (num_points == 0) {
    publisher_->publish(*msg);
    return;
  }

  // Check if ring field exists
  bool has_ring = false;
  int ring_offset = -1;
  int ring_datatype = 0;
  for (const auto & field : msg->fields) {
    if (field.name == "ring") {
      has_ring = true;
      ring_offset = field.offset;
      ring_datatype = field.datatype;
    }
  }

  // Read all points using iterators
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  // Build sector-ring bins
  struct BinPoint {
    float z;
    float distance;
    uint32_t index;
  };

  std::vector<std::vector<std::vector<BinPoint>>> bins(
    num_sectors_, std::vector<std::vector<BinPoint>>(num_rings_));

  const double sector_size = 2.0 * M_PI / num_sectors_;

  for (uint32_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;

    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
      continue;
    }

    // Get ring index
    int ring = 0;
    if (has_ring) {
      const uint8_t * point_data = &msg->data[i * msg->point_step + ring_offset];
      if (ring_datatype == sensor_msgs::msg::PointField::UINT16) {
        uint16_t ring_val;
        std::memcpy(&ring_val, point_data, sizeof(uint16_t));
        ring = static_cast<int>(ring_val);
      } else if (ring_datatype == sensor_msgs::msg::PointField::UINT8) {
        ring = static_cast<int>(*point_data);
      }
    } else {
      float distance_xy = std::sqrt(x * x + y * y);
      float vertical_angle = std::atan2(z, distance_xy) * 180.0 / M_PI;
      ring = static_cast<int>(std::round((vertical_angle + 15.0) / 2.0));
      ring = std::clamp(ring, 0, num_rings_ - 1);
    }

    if (ring < 0 || ring >= num_rings_) {
      continue;
    }

    double azimuth = std::atan2(y, x);
    if (azimuth < 0) {
      azimuth += 2.0 * M_PI;
    }
    int sector = static_cast<int>(azimuth / sector_size);
    sector = std::clamp(sector, 0, num_sectors_ - 1);

    float distance = std::sqrt(x * x + y * y);
    bins[sector][ring].push_back({z, distance, i});
  }

  // Ground segmentation
  std::vector<bool> is_ground(num_points, false);

  for (int s = 0; s < num_sectors_; ++s) {
    // Compute representative z per ring (median of lowest points)
    struct RingInfo {
      float z_min;
      float z_mean;
      int ring_idx;
      bool has_points;
    };
    std::vector<RingInfo> ring_info(num_rings_);

    for (int r = 0; r < num_rings_; ++r) {
      ring_info[r].ring_idx = r;
      ring_info[r].has_points = !bins[s][r].empty();
      if (ring_info[r].has_points) {
        // Sort points by z to get robust estimate
        auto & pts = bins[s][r];
        std::sort(pts.begin(), pts.end(),
          [](const BinPoint & a, const BinPoint & b) { return a.z < b.z; });

        // Use lower quartile as representative z (robust to outliers)
        size_t q_idx = pts.size() / 4;
        ring_info[r].z_min = pts[q_idx].z;

        // Compute mean z
        float sum = 0.0f;
        for (const auto & p : pts) { sum += p.z; }
        ring_info[r].z_mean = sum / static_cast<float>(pts.size());
      } else {
        ring_info[r].z_min = std::numeric_limits<float>::max();
        ring_info[r].z_mean = std::numeric_limits<float>::max();
      }
    }

    // Find the ring with the lowest representative z in this sector → ground seed
    // Only consider lower rings (0 ~ num_rings/2) as ground seed candidates
    int seed_ring = -1;
    float seed_z = std::numeric_limits<float>::max();
    for (int r = 0; r < num_rings_; ++r) {
      if (ring_info[r].has_points && ring_info[r].z_min < seed_z) {
        seed_z = ring_info[r].z_min;
        seed_ring = r;
      }
    }

    if (seed_ring < 0) {
      continue;  // No points in this sector
    }

    // Mark ground rings: propagate from seed ring outward
    std::vector<bool> ring_is_ground(num_rings_, false);
    ring_is_ground[seed_ring] = true;
    float ground_z = ring_info[seed_ring].z_min;

    // Propagate upward (seed_ring+1 → num_rings-1)
    float prev_z = ground_z;
    for (int r = seed_ring + 1; r < num_rings_; ++r) {
      if (!ring_info[r].has_points) {
        continue;
      }
      float diff = ring_info[r].z_min - prev_z;
      if (std::abs(diff) < ground_threshold_) {
        ring_is_ground[r] = true;
        prev_z = ring_info[r].z_min;
      } else {
        break;  // Non-ground encountered, stop propagation
      }
    }

    // Propagate downward (seed_ring-1 → 0)
    prev_z = ground_z;
    for (int r = seed_ring - 1; r >= 0; --r) {
      if (!ring_info[r].has_points) {
        continue;
      }
      float diff = ring_info[r].z_min - prev_z;
      if (std::abs(diff) < ground_threshold_) {
        ring_is_ground[r] = true;
        prev_z = ring_info[r].z_min;
      } else {
        break;
      }
    }

    // Mark individual points as ground
    for (int r = 0; r < num_rings_; ++r) {
      if (!ring_is_ground[r]) {
        continue;
      }
      float ref_z = ring_info[r].z_min;
      for (const auto & p : bins[s][r]) {
        // Point is ground if its z is close to the ring's ground estimate
        if (std::abs(p.z - ref_z) < ground_threshold_) {
          is_ground[p.index] = true;
        }
      }
    }
  }

  // Count non-ground points
  uint32_t non_ground_count = 0;
  for (uint32_t i = 0; i < num_points; ++i) {
    if (!is_ground[i]) {
      ++non_ground_count;
    }
  }

  // Build output message preserving all fields
  sensor_msgs::msg::PointCloud2 output_msg;
  output_msg.header = msg->header;
  output_msg.fields = msg->fields;
  output_msg.point_step = msg->point_step;
  output_msg.height = 1;
  output_msg.width = non_ground_count;
  output_msg.row_step = non_ground_count * msg->point_step;
  output_msg.is_bigendian = msg->is_bigendian;
  output_msg.is_dense = msg->is_dense;
  output_msg.data.resize(output_msg.row_step);

  uint32_t out_idx = 0;
  for (uint32_t i = 0; i < num_points; ++i) {
    if (!is_ground[i]) {
      std::memcpy(
        &output_msg.data[out_idx * msg->point_step],
        &msg->data[i * msg->point_step],
        msg->point_step);
      ++out_idx;
    }
  }

  publisher_->publish(output_msg);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "GroundSegmentation: Input %u pts -> Non-ground %u pts (removed %.1f%% ground)",
    num_points, non_ground_count,
    num_points > 0 ? ((num_points - non_ground_count) * 100.0 / num_points) : 0.0);
}

}  // namespace velodyne_ground_segmentation

RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_ground_segmentation::GroundSegmentationComponent)
