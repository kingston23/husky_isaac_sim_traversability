#ifndef HUSKY_TRAVERSABILITY__TRAVERSABILITY_GRID_HPP_
#define HUSKY_TRAVERSABILITY__TRAVERSABILITY_GRID_HPP_

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace husky_traversability
{

/// Cost values, identical to nav2_costmap_2d (checked with static_assert in the layer).
namespace cost
{
constexpr unsigned char kFree = 0;
constexpr unsigned char kMaxNonLethal = 252;
constexpr unsigned char kLethal = 254;
constexpr unsigned char kNoInformation = 255;
}  // namespace cost

/// All lengths in metres, all angles in degrees.
struct TraversabilityConfig
{
  /// Edge length of the analysis cells (use 2-5x the costmap resolution).
  double cell_size{0.25};
  /// A cell needs at least this many ground points to have a ground height.
  int min_ground_points{3};
  /// Radius of the neighbourhood used to fit the local ground plane (slope).
  double slope_radius{0.75};
  /// Below this lambda_mid / lambda_max ratio the neighbourhood is a line and gives no slope.
  double min_linearity_ratio{0.05};
  /// Slopes up to this angle cost nothing.
  double slope_safe_deg{10.0};
  /// Slopes from this angle on are lethal. Between safe and lethal the cost rises linearly.
  double slope_lethal_deg{25.0};
  /// Cost at the upper end of the linear slope ramp (max 252, below "inscribed").
  int max_slope_cost{200};
  /// Height jump between neighbouring cells, not explained by the local slope, that is lethal.
  double max_step_height{0.15};
  /// Obstacle points closer to the ground than this are ignored (grass, noise).
  double min_obstacle_height{0.15};
  /// Obstacle points higher above the ground than this are ignored (canopy, overhangs).
  double max_obstacle_height{1.0};
  /// Obstacle cells without own ground look for ground within this radius.
  double ground_search_radius{1.0};
  /// What to do with obstacle points that have no ground reference nearby.
  bool mark_obstacles_without_ground{true};
};

/// Axis-aligned world rectangle of one analysis cell and its cost.
struct CellResult
{
  double min_x;
  double min_y;
  double max_x;
  double max_y;
  unsigned char cost;
};

/// Debug view of one cell.
struct CellInfo
{
  bool has_ground{false};
  bool has_slope{false};
  double ground_height{0.0};
  double slope_deg{0.0};
  double step_height{0.0};
  bool lethal_obstacle{false};
  unsigned char cost{cost::kNoInformation};
};

/// Turns ground / obstacle point clouds (gravity-aligned frame) into traversability costs:
/// slope from a local plane fit, step height between neighbouring cells, and obstacle height
/// measured relative to the local ground instead of an absolute z threshold.
class TraversabilityGrid
{
public:
  explicit TraversabilityGrid(const TraversabilityConfig & config);

  void clear();
  void addGroundPoints(const std::vector<Eigen::Vector3f> & points);
  void addObstaclePoints(const std::vector<Eigen::Vector3f> & points);

  /// Evaluates all cells. Call after adding points.
  void compute();

  /// All cells with a cost other than kNoInformation.
  std::vector<CellResult> results() const;

  /// Debug information for the cell containing (x, y).
  CellInfo info(double x, double y) const;

  const TraversabilityConfig & config() const {return config_;}

private:
  struct Cell
  {
    int ix{0};
    int iy{0};
    // Ground statistics
    std::size_t n_ground{0};
    Eigen::Vector3d sum{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d outer{Eigen::Matrix3d::Zero()};
    std::vector<float> obstacle_z;
    // Derived
    bool has_ground{false};
    bool has_slope{false};
    Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
    Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
    double slope_deg{0.0};
    double step_height{0.0};
    bool lethal_obstacle{false};
    unsigned char cost{cost::kNoInformation};
  };

  using Key = std::uint64_t;

  static Key makeKey(int ix, int iy);
  Cell & cellFor(float x, float y);
  const Cell * find(int ix, int iy) const;
  const Cell * nearestGround(const Cell & cell) const;
  double groundHeightAt(const Cell & ground, double x, double y) const;
  unsigned char slopeCost(double slope_deg) const;

  TraversabilityConfig config_;
  std::unordered_map<Key, Cell> cells_;
};

}  // namespace husky_traversability

#endif  // HUSKY_TRAVERSABILITY__TRAVERSABILITY_GRID_HPP_
