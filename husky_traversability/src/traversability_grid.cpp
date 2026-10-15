#include "husky_traversability/traversability_grid.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Eigenvalues>

namespace husky_traversability
{
namespace
{

double deg2rad(double deg) {return deg * M_PI / 180.0;}
double rad2deg(double rad) {return rad * 180.0 / M_PI;}

// Planes steeper than this (normal almost horizontal) are not used for height prediction.
constexpr double kMinNormalZ = 0.1;

}  // namespace

TraversabilityGrid::TraversabilityGrid(const TraversabilityConfig & config)
: config_(config)
{
  config_.min_ground_points = std::max(config_.min_ground_points, 1);
  config_.max_slope_cost = std::clamp(config_.max_slope_cost, 1, static_cast<int>(cost::kMaxNonLethal));
}

TraversabilityGrid::Key TraversabilityGrid::makeKey(int ix, int iy)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32) |
         static_cast<std::uint64_t>(static_cast<std::uint32_t>(iy));
}

void TraversabilityGrid::clear()
{
  cells_.clear();
}

TraversabilityGrid::Cell & TraversabilityGrid::cellFor(float x, float y)
{
  const int ix = static_cast<int>(std::floor(x / config_.cell_size));
  const int iy = static_cast<int>(std::floor(y / config_.cell_size));
  Cell & cell = cells_[makeKey(ix, iy)];
  cell.ix = ix;
  cell.iy = iy;
  return cell;
}

const TraversabilityGrid::Cell * TraversabilityGrid::find(int ix, int iy) const
{
  const auto it = cells_.find(makeKey(ix, iy));
  return it == cells_.end() ? nullptr : &it->second;
}

void TraversabilityGrid::addGroundPoints(const std::vector<Eigen::Vector3f> & points)
{
  for (const auto & p : points) {
    if (!p.allFinite()) {
      continue;
    }
    Cell & cell = cellFor(p.x(), p.y());
    const Eigen::Vector3d q = p.cast<double>();
    ++cell.n_ground;
    cell.sum += q;
    cell.outer += q * q.transpose();
  }
}

void TraversabilityGrid::addObstaclePoints(const std::vector<Eigen::Vector3f> & points)
{
  for (const auto & p : points) {
    if (!p.allFinite()) {
      continue;
    }
    cellFor(p.x(), p.y()).obstacle_z.push_back(p.z());
  }
}

double TraversabilityGrid::groundHeightAt(const Cell & ground, double x, double y) const
{
  if (!ground.has_slope || ground.normal.z() < kMinNormalZ) {
    return ground.centroid.z();
  }
  const Eigen::Vector3d & n = ground.normal;
  return ground.centroid.z() -
         (n.x() * (x - ground.centroid.x()) + n.y() * (y - ground.centroid.y())) / n.z();
}

const TraversabilityGrid::Cell * TraversabilityGrid::nearestGround(const Cell & cell) const
{
  const double cs = config_.cell_size;
  const double cx = (cell.ix + 0.5) * cs;
  const double cy = (cell.iy + 0.5) * cs;
  const int range = static_cast<int>(std::ceil(config_.ground_search_radius / cs));

  const Cell * best = nullptr;
  double best_distance = config_.ground_search_radius;
  for (int dx = -range; dx <= range; ++dx) {
    for (int dy = -range; dy <= range; ++dy) {
      const Cell * candidate = find(cell.ix + dx, cell.iy + dy);
      if (candidate == nullptr || !candidate->has_ground) {
        continue;
      }
      const double d = std::hypot(candidate->centroid.x() - cx, candidate->centroid.y() - cy);
      if (d <= best_distance) {
        best_distance = d;
        best = candidate;
      }
    }
  }
  return best;
}

unsigned char TraversabilityGrid::slopeCost(double slope_deg) const
{
  if (slope_deg <= config_.slope_safe_deg) {
    return cost::kFree;
  }
  if (slope_deg >= config_.slope_lethal_deg) {
    return cost::kLethal;
  }
  const double t = (slope_deg - config_.slope_safe_deg) /
    (config_.slope_lethal_deg - config_.slope_safe_deg);
  const double value = 1.0 + t * (config_.max_slope_cost - 1.0);
  return static_cast<unsigned char>(std::clamp(value, 1.0, double(config_.max_slope_cost)));
}

void TraversabilityGrid::compute()
{
  const double cs = config_.cell_size;
  const double tan_lethal = std::tan(deg2rad(config_.slope_lethal_deg));

  // ---------------------------------------------------------------------------
  // 1) Ground height per cell
  // ---------------------------------------------------------------------------
  for (auto & kv : cells_) {
    Cell & cell = kv.second;
    cell.has_ground = cell.n_ground >= static_cast<std::size_t>(config_.min_ground_points);
    if (cell.n_ground > 0) {
      cell.centroid = cell.sum / static_cast<double>(cell.n_ground);
    }
  }

  // ---------------------------------------------------------------------------
  // 2) Slope: PCA plane over all ground points within slope_radius
  // ---------------------------------------------------------------------------
  const int slope_range = static_cast<int>(std::ceil(config_.slope_radius / cs));
  for (auto & kv : cells_) {
    Cell & cell = kv.second;
    if (!cell.has_ground) {
      continue;
    }
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d outer = Eigen::Matrix3d::Zero();
    std::size_t n = 0;
    for (int dx = -slope_range; dx <= slope_range; ++dx) {
      for (int dy = -slope_range; dy <= slope_range; ++dy) {
        if (std::hypot(dx, dy) * cs > config_.slope_radius) {
          continue;
        }
        const Cell * nb = find(cell.ix + dx, cell.iy + dy);
        if (nb == nullptr || !nb->has_ground) {
          continue;
        }
        sum += nb->sum;
        outer += nb->outer;
        n += nb->n_ground;
      }
    }
    if (n < 3) {
      continue;
    }
    const Eigen::Vector3d mean = sum / static_cast<double>(n);
    const Eigen::Matrix3d cov = outer / static_cast<double>(n) - mean * mean.transpose();
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    const Eigen::Vector3d & ev = solver.eigenvalues();  // ascending
    if (ev(2) <= 1e-12 || ev(1) < config_.min_linearity_ratio * ev(2)) {
      continue;  // points on a line: slope across the line is unknown
    }
    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    if (normal.z() < 0.0) {
      normal = -normal;
    }
    cell.normal = normal;
    cell.has_slope = true;
    cell.slope_deg = rad2deg(std::acos(std::clamp(normal.z(), -1.0, 1.0)));
  }

  // ---------------------------------------------------------------------------
  // 3) Step height: jump to the 8 neighbours that the local plane does not explain
  // ---------------------------------------------------------------------------
  for (auto & kv : cells_) {
    Cell & cell = kv.second;
    if (!cell.has_ground) {
      continue;
    }
    const bool use_plane = cell.has_slope && cell.normal.z() >= kMinNormalZ;
    double step = 0.0;
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const Cell * nb = find(cell.ix + dx, cell.iy + dy);
        if (nb == nullptr || !nb->has_ground) {
          continue;
        }
        double residual;
        if (use_plane) {
          residual = std::abs(
            nb->centroid.z() - groundHeightAt(cell, nb->centroid.x(), nb->centroid.y()));
        } else {
          // No plane: only count what even the steepest lethal slope cannot explain.
          const double d = std::hypot(
            nb->centroid.x() - cell.centroid.x(), nb->centroid.y() - cell.centroid.y());
          residual = std::max(0.0, std::abs(nb->centroid.z() - cell.centroid.z()) - d * tan_lethal);
        }
        step = std::max(step, residual);
      }
    }
    cell.step_height = step;
  }

  // ---------------------------------------------------------------------------
  // 4) Obstacles: height above the local ground instead of an absolute threshold
  // ---------------------------------------------------------------------------
  for (auto & kv : cells_) {
    Cell & cell = kv.second;
    if (cell.obstacle_z.empty()) {
      continue;
    }
    const Cell * reference = cell.has_ground ? &cell : nearestGround(cell);
    if (reference == nullptr) {
      cell.lethal_obstacle = config_.mark_obstacles_without_ground;
      continue;
    }
    const double h = groundHeightAt(
      *reference, (cell.ix + 0.5) * cs, (cell.iy + 0.5) * cs);
    for (const float z : cell.obstacle_z) {
      const double above = z - h;
      if (above >= config_.min_obstacle_height && above <= config_.max_obstacle_height) {
        cell.lethal_obstacle = true;
        break;
      }
    }
  }

  // ---------------------------------------------------------------------------
  // 5) Final cost
  // ---------------------------------------------------------------------------
  for (auto & kv : cells_) {
    Cell & cell = kv.second;
    if (cell.lethal_obstacle) {
      cell.cost = cost::kLethal;
    } else if (cell.has_ground) {
      cell.cost = slopeCost(cell.has_slope ? cell.slope_deg : 0.0);
      if (cell.step_height > config_.max_step_height) {
        cell.cost = cost::kLethal;
      }
    } else {
      cell.cost = cost::kNoInformation;
    }
  }
}

std::vector<CellResult> TraversabilityGrid::results() const
{
  const double cs = config_.cell_size;
  std::vector<CellResult> out;
  out.reserve(cells_.size());
  for (const auto & kv : cells_) {
    const Cell & cell = kv.second;
    if (cell.cost == cost::kNoInformation) {
      continue;
    }
    out.push_back(
      {cell.ix * cs, cell.iy * cs, (cell.ix + 1) * cs, (cell.iy + 1) * cs, cell.cost});
  }
  return out;
}

CellInfo TraversabilityGrid::info(double x, double y) const
{
  CellInfo out;
  const Cell * cell = find(
    static_cast<int>(std::floor(x / config_.cell_size)),
    static_cast<int>(std::floor(y / config_.cell_size)));
  if (cell == nullptr) {
    return out;
  }
  out.has_ground = cell->has_ground;
  out.has_slope = cell->has_slope;
  out.ground_height = cell->centroid.z();
  out.slope_deg = cell->slope_deg;
  out.step_height = cell->step_height;
  out.lethal_obstacle = cell->lethal_obstacle;
  out.cost = cell->cost;
  return out;
}

}  // namespace husky_traversability
