#include "lidar_clustering/dbscan.hpp"
#include "lidar_clustering/nanoflann.hpp"

#include <queue>

namespace lidar_clustering
{

/// nanoflann adaptor — zero-copy reference to std::vector<Point3D>
struct PointCloudAdaptor
{
  const std::vector<Point3D> & pts;

  explicit PointCloudAdaptor(const std::vector<Point3D> & points) : pts(points) {}

  inline size_t kdtree_get_point_count() const { return pts.size(); }

  inline float kdtree_get_pt(const size_t idx, const size_t dim) const
  {
    if (dim == 0) return pts[idx].x;
    if (dim == 1) return pts[idx].y;
    return pts[idx].z;
  }

  template<class BBOX>
  bool kdtree_get_bbox(BBOX &) const { return false; }
};

using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
  nanoflann::L2_Simple_Adaptor<float, PointCloudAdaptor>,
  PointCloudAdaptor,
  3,     // dimensionality
  size_t // index type
>;

std::vector<int32_t> DBSCAN::cluster(
  const std::vector<Point3D> & points, double eps, int min_pts)
{
  const size_t n = points.size();
  std::vector<int32_t> labels(n, -1);  // -1 = unvisited / noise

  if (n == 0) {
    return labels;
  }

  // Build KD-tree
  PointCloudAdaptor adaptor(points);
  KDTree index(3, adaptor, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  index.buildIndex();

  const float eps_sq = static_cast<float>(eps * eps);
  int32_t cluster_id = 0;

  // Reusable containers
  nanoflann::SearchParameters search_params;
  search_params.sorted = false;

  std::vector<nanoflann::ResultItem<size_t, float>> neighbors;
  std::vector<nanoflann::ResultItem<size_t, float>> sub_neighbors;
  std::queue<size_t> expand_queue;

  for (size_t i = 0; i < n; ++i) {
    if (labels[i] != -1) {
      continue;  // already processed
    }

    // Range query around point i
    const float query_pt[3] = {points[i].x, points[i].y, points[i].z};
    neighbors.clear();
    index.radiusSearch(query_pt, eps_sq, neighbors, search_params);

    if (static_cast<int>(neighbors.size()) < min_pts) {
      continue;  // remains noise (-1)
    }

    // Start a new cluster
    labels[i] = cluster_id;

    // Seed the expansion queue
    while (!expand_queue.empty()) expand_queue.pop();
    for (const auto & nb : neighbors) {
      if (nb.first != i && labels[nb.first] == -1) {
        expand_queue.push(nb.first);
      }
    }

    while (!expand_queue.empty()) {
      size_t q = expand_queue.front();
      expand_queue.pop();

      if (labels[q] != -1 && labels[q] != cluster_id) {
        continue;  // already assigned to another cluster
      }
      if (labels[q] == cluster_id) {
        continue;  // already in this cluster
      }

      labels[q] = cluster_id;

      // Expand if q is also a core point
      const float q_pt[3] = {points[q].x, points[q].y, points[q].z};
      sub_neighbors.clear();
      index.radiusSearch(q_pt, eps_sq, sub_neighbors, search_params);

      if (static_cast<int>(sub_neighbors.size()) >= min_pts) {
        for (const auto & nb : sub_neighbors) {
          if (labels[nb.first] == -1) {
            expand_queue.push(nb.first);
          }
        }
      }
    }

    ++cluster_id;
  }

  return labels;
}

}  // namespace lidar_clustering
