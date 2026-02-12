#ifndef LIDAR_CLUSTERING__DBSCAN_HPP_
#define LIDAR_CLUSTERING__DBSCAN_HPP_

#include <cstdint>
#include <vector>

namespace lidar_clustering
{

struct Point3D
{
  float x, y, z;
};

class DBSCAN
{
public:
  /// Run DBSCAN clustering on the given points.
  /// @param points  Input point cloud
  /// @param eps     Neighborhood search radius (meters)
  /// @param min_pts Minimum neighbors for a core point
  /// @return labels: >= 0 cluster ID, -1 noise
  static std::vector<int32_t> cluster(
    const std::vector<Point3D> & points, double eps, int min_pts);
};

}  // namespace lidar_clustering

#endif  // LIDAR_CLUSTERING__DBSCAN_HPP_
