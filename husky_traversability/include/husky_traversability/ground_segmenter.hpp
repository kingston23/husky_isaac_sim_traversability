#ifndef HUSKY_TRAVERSABILITY__GROUND_SEGMENTER_HPP_
#define HUSKY_TRAVERSABILITY__GROUND_SEGMENTER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <Eigen/Core>

namespace husky_traversability
{

enum class PointLabel : std::uint8_t
{
  Ground = 0,
  Obstacle = 1
};

/// All lengths in metres, all angles in degrees.
struct GroundSegmenterConfig
{
  /// Edge length of the 2D grid cells.
  double cell_size{0.5};
  /// Only points within this height above the lowest point of a cell are used for the ground fit.
  double low_band{0.35};
  /// Minimum number of low-band points required to fit a cell.
  int min_points_per_cell{5};
  /// Steepest plane (or line) that is still accepted as ground.
  double max_slope_deg{25.0};
  /// Upper bound on lambda_min / sum(lambda). Larger values mean the cell is not a surface.
  double max_planarity{0.05};
  /// If lambda_mid / lambda_max is below this, the points form a line (e.g. a single scan ring).
  double min_linearity_ratio{0.05};
  /// Height tolerance between neighbouring ground cells, independent of distance.
  double max_step{0.15};
  /// Slope change allowed when a neighbour's height is predicted from the current cell's plane.
  double max_slope_change_deg{10.0};
  /// Neighbour search radius during region growing. Bridges gaps between scan rings.
  double search_radius{1.5};
  /// Radius around the robot in which seed cells are searched. Must exceed the lidar blind zone.
  double seed_radius{4.0};
  /// Maximum height difference between a seed cell and the robot's ground plane.
  double seed_height_tolerance{0.25};
  /// Maximum vertical distance between a point and the local ground plane for a ground label.
  double ground_inlier_threshold{0.10};
};

struct SegmentationStats
{
  std::size_t occupied_cells{0};
  std::size_t candidate_cells{0};
  std::size_t seed_cells{0};
  std::size_t ground_cells{0};
  std::size_t ground_points{0};
};

/// ROS-independent ground segmentation inspired by GSeg3D:
/// per-cell PCA classification, slope-aware region growing from a seed under the robot,
/// and a point-wise height check against the nearest ground plane.
class GroundSegmenter
{
public:
  explicit GroundSegmenter(const GroundSegmenterConfig & config);

  /// @param points      Points expressed in a gravity-aligned frame (z points up).
  /// @param seed_point  A point on the ground directly under the robot, same frame.
  /// @param seed_normal The robot's up axis in the same frame (approximates the local ground normal).
  /// @param labels      Output, one label per input point.
  SegmentationStats segment(
    const std::vector<Eigen::Vector3f> & points,
    const Eigen::Vector3d & seed_point,
    const Eigen::Vector3d & seed_normal,
    std::vector<PointLabel> & labels) const;

  const GroundSegmenterConfig & config() const {return config_;}

private:
  GroundSegmenterConfig config_;
  double cos_max_slope_;
  double sin_max_slope_;
  double tan_max_slope_;
  double tan_max_slope_change_;
};

}  // namespace husky_traversability

#endif  // HUSKY_TRAVERSABILITY__GROUND_SEGMENTER_HPP_
