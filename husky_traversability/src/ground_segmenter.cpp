#include "husky_traversability/ground_segmenter.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <unordered_map>

#include <Eigen/Eigenvalues>

namespace husky_traversability
{
namespace
{

using CellKey = std::uint64_t;

CellKey makeKey(int ix, int iy)
{
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32) |
         static_cast<std::uint64_t>(static_cast<std::uint32_t>(iy));
}

struct Cell
{
  int ix{0};
  int iy{0};
  std::vector<std::size_t> indices;
  bool candidate{false};   // passed the local geometric test
  bool has_normal{false};  // false for line cells: only the height is known
  bool ground{false};      // reached by region growing
  Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitZ()};
};

/// Height of a ground plane at the horizontal position (x, y).
double predictHeight(
  const Eigen::Vector3d & centroid, const Eigen::Vector3d & normal, bool has_normal,
  double x, double y)
{
  if (!has_normal) {
    return centroid.z();
  }
  return centroid.z() -
         (normal.x() * (x - centroid.x()) + normal.y() * (y - centroid.y())) / normal.z();
}

double deg2rad(double deg) {return deg * M_PI / 180.0;}

}  // namespace

GroundSegmenter::GroundSegmenter(const GroundSegmenterConfig & config)
: config_(config),
  cos_max_slope_(std::cos(deg2rad(config.max_slope_deg))),
  sin_max_slope_(std::sin(deg2rad(config.max_slope_deg))),
  tan_max_slope_(std::tan(deg2rad(config.max_slope_deg))),
  tan_max_slope_change_(std::tan(deg2rad(config.max_slope_change_deg)))
{
}

SegmentationStats GroundSegmenter::segment(
  const std::vector<Eigen::Vector3f> & points,
  const Eigen::Vector3d & seed_point,
  const Eigen::Vector3d & seed_normal_in,
  std::vector<PointLabel> & labels) const
{
  SegmentationStats stats;
  labels.assign(points.size(), PointLabel::Obstacle);
  if (points.empty()) {
    return stats;
  }

  const double cs = config_.cell_size;

  // ---------------------------------------------------------------------------
  // 1) Grid representation: bin points into 2D cells
  // ---------------------------------------------------------------------------
  std::unordered_map<CellKey, Cell> cells;
  cells.reserve(points.size() / 4 + 1);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto & p = points[i];
    if (!p.allFinite()) {
      continue;
    }
    const int ix = static_cast<int>(std::floor(p.x() / cs));
    const int iy = static_cast<int>(std::floor(p.y() / cs));
    Cell & cell = cells[makeKey(ix, iy)];
    if (cell.indices.empty()) {
      cell.ix = ix;
      cell.iy = iy;
    }
    cell.indices.push_back(i);
  }
  stats.occupied_cells = cells.size();

  // ---------------------------------------------------------------------------
  // 2) Local eigen classification of the lowest points in each cell
  // ---------------------------------------------------------------------------
  for (auto & kv : cells) {
    Cell & cell = kv.second;

    float z_min = std::numeric_limits<float>::max();
    for (const auto idx : cell.indices) {
      z_min = std::min(z_min, points[idx].z());
    }

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d outer = Eigen::Matrix3d::Zero();
    std::size_t n = 0;
    for (const auto idx : cell.indices) {
      const auto & p = points[idx];
      if (p.z() > z_min + config_.low_band) {
        continue;
      }
      const Eigen::Vector3d q = p.cast<double>();
      sum += q;
      outer += q * q.transpose();
      ++n;
    }
    if (n < static_cast<std::size_t>(config_.min_points_per_cell)) {
      continue;
    }

    cell.centroid = sum / static_cast<double>(n);
    const Eigen::Matrix3d cov =
      outer / static_cast<double>(n) - cell.centroid * cell.centroid.transpose();

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    const Eigen::Vector3d & ev = solver.eigenvalues();  // ascending order
    const double total = ev.sum();

    if (total < 1e-12) {
      // All points coincide: only the height is meaningful.
      cell.candidate = true;
      cell.has_normal = false;
    } else if (ev(1) < config_.min_linearity_ratio * ev(2)) {
      // Line cell. Accept only near-horizontal lines (a scan ring on the ground).
      const Eigen::Vector3d direction = solver.eigenvectors().col(2);
      cell.candidate = std::abs(direction.z()) <= sin_max_slope_;
      cell.has_normal = false;
    } else if (ev(0) / total <= config_.max_planarity) {
      // Planar cell: the normal gives the slope.
      Eigen::Vector3d normal = solver.eigenvectors().col(0);
      if (normal.z() < 0.0) {
        normal = -normal;
      }
      cell.normal = normal;
      cell.has_normal = true;
      cell.candidate = normal.z() >= cos_max_slope_;
    }
    // Otherwise non-planar (vegetation, clutter, object bases): not a candidate.

    if (cell.candidate) {
      ++stats.candidate_cells;
    }
  }

  // ---------------------------------------------------------------------------
  // 3) Seed cells around the robot
  // ---------------------------------------------------------------------------
  Eigen::Vector3d seed_normal = seed_normal_in.normalized();
  if (seed_normal.z() < 0.0) {
    seed_normal = -seed_normal;
  }
  const bool seed_has_normal = seed_normal.allFinite() && seed_normal.z() > 0.5;

  std::deque<Cell *> queue;
  const int seed_range = static_cast<int>(std::ceil(config_.seed_radius / cs));
  const int seed_ix = static_cast<int>(std::floor(seed_point.x() / cs));
  const int seed_iy = static_cast<int>(std::floor(seed_point.y() / cs));
  for (int dx = -seed_range; dx <= seed_range; ++dx) {
    for (int dy = -seed_range; dy <= seed_range; ++dy) {
      const auto it = cells.find(makeKey(seed_ix + dx, seed_iy + dy));
      if (it == cells.end() || !it->second.candidate) {
        continue;
      }
      Cell & cell = it->second;
      const double d = std::hypot(
        cell.centroid.x() - seed_point.x(), cell.centroid.y() - seed_point.y());
      if (d > config_.seed_radius) {
        continue;
      }
      const double z_pred = predictHeight(
        seed_point, seed_normal, seed_has_normal, cell.centroid.x(), cell.centroid.y());
      if (std::abs(cell.centroid.z() - z_pred) > config_.seed_height_tolerance) {
        continue;
      }
      cell.ground = true;
      queue.push_back(&cell);
      ++stats.seed_cells;
    }
  }

  // ---------------------------------------------------------------------------
  // 4) Slope-aware region growing (radius search bridges gaps between rings)
  // ---------------------------------------------------------------------------
  const int search_range = static_cast<int>(std::ceil(config_.search_radius / cs));
  while (!queue.empty()) {
    const Cell * current = queue.front();
    queue.pop_front();

    for (int dx = -search_range; dx <= search_range; ++dx) {
      for (int dy = -search_range; dy <= search_range; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const auto it = cells.find(makeKey(current->ix + dx, current->iy + dy));
        if (it == cells.end()) {
          continue;
        }
        Cell & neighbour = it->second;
        if (!neighbour.candidate || neighbour.ground) {
          continue;
        }
        const double d = std::hypot(
          neighbour.centroid.x() - current->centroid.x(),
          neighbour.centroid.y() - current->centroid.y());
        if (d > config_.search_radius) {
          continue;
        }
        // With a known plane we extrapolate it and only allow a small slope change.
        // Without one (line cell) we can only bound the height by the maximum slope.
        const double z_pred = predictHeight(
          current->centroid, current->normal, current->has_normal,
          neighbour.centroid.x(), neighbour.centroid.y());
        const double tolerance = config_.max_step +
          d * (current->has_normal ? tan_max_slope_change_ : tan_max_slope_);
        if (std::abs(neighbour.centroid.z() - z_pred) > tolerance) {
          continue;
        }
        neighbour.ground = true;
        queue.push_back(&neighbour);
      }
    }
  }

  // ---------------------------------------------------------------------------
  // 5) Point-wise labelling against the own or nearest ground plane
  // ---------------------------------------------------------------------------
  for (const auto & kv : cells) {
    const Cell & cell = kv.second;
    const Cell * reference = nullptr;

    if (cell.ground) {
      reference = &cell;
      ++stats.ground_cells;
    } else {
      const double cx = (cell.ix + 0.5) * cs;
      const double cy = (cell.iy + 0.5) * cs;
      double best = config_.search_radius;
      for (int dx = -search_range; dx <= search_range; ++dx) {
        for (int dy = -search_range; dy <= search_range; ++dy) {
          const auto it = cells.find(makeKey(cell.ix + dx, cell.iy + dy));
          if (it == cells.end() || !it->second.ground) {
            continue;
          }
          const double d =
            std::hypot(it->second.centroid.x() - cx, it->second.centroid.y() - cy);
          if (d <= best) {
            best = d;
            reference = &it->second;
          }
        }
      }
    }

    if (reference == nullptr) {
      continue;  // no ground nearby: everything stays an obstacle
    }

    for (const auto idx : cell.indices) {
      const auto & p = points[idx];
      const double z_ground = predictHeight(
        reference->centroid, reference->normal, reference->has_normal, p.x(), p.y());
      if (std::abs(p.z() - z_ground) <= config_.ground_inlier_threshold) {
        labels[idx] = PointLabel::Ground;
        ++stats.ground_points;
      }
    }
  }

  return stats;
}

}  // namespace husky_traversability
