#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <random>
#include <vector>

#include "husky_traversability/traversability_grid.hpp"

using husky_traversability::TraversabilityConfig;
using husky_traversability::TraversabilityGrid;
namespace cost = husky_traversability::cost;

namespace
{

using HeightFn = std::function<double(double, double)>;

// Samples a height field on a 5 cm grid with 1 cm noise.
std::vector<Eigen::Vector3f> sampleGround(
  double x0, double x1, double y0, double y1, const HeightFn & height,
  const std::function<bool(double, double)> & skip = nullptr)
{
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.01);
  std::vector<Eigen::Vector3f> points;
  const int nx = static_cast<int>(std::round((x1 - x0) / 0.05));
  const int ny = static_cast<int>(std::round((y1 - y0) / 0.05));
  for (int i = 0; i < nx; ++i) {
    for (int j = 0; j < ny; ++j) {
      const double x = x0 + i * 0.05 + 0.025;
      const double y = y0 + j * 0.05 + 0.025;
      if (skip && skip(x, y)) {continue;}
      points.emplace_back(x, y, height(x, y) + noise(rng));
    }
  }
  return points;
}

HeightFn ramp(double angle_deg)
{
  const double t = std::tan(angle_deg * M_PI / 180.0);
  return [t](double x, double) {return x * t;};
}

}  // namespace

TEST(TraversabilityGrid, FlatGroundIsFree)
{
  TraversabilityGrid grid(TraversabilityConfig{});
  grid.addGroundPoints(sampleGround(-2, 2, -2, 2, [](double, double) {return 0.0;}));
  grid.compute();

  const auto info = grid.info(0.1, 0.1);
  EXPECT_TRUE(info.has_ground);
  EXPECT_TRUE(info.has_slope);
  EXPECT_LT(info.slope_deg, 2.0);
  EXPECT_LT(info.step_height, 0.05);
  EXPECT_EQ(info.cost, cost::kFree);
}

TEST(TraversabilityGrid, ModerateRampHasIntermediateCost)
{
  TraversabilityGrid grid(TraversabilityConfig{});  // safe 10 deg, lethal 25 deg
  grid.addGroundPoints(sampleGround(-2, 2, -2, 2, ramp(17.5)));
  grid.compute();

  const auto info = grid.info(0.1, 0.1);
  EXPECT_NEAR(info.slope_deg, 17.5, 1.5);
  EXPECT_LT(info.step_height, 0.05);
  EXPECT_GT(info.cost, cost::kFree);
  EXPECT_LT(info.cost, cost::kLethal);
}

TEST(TraversabilityGrid, SteepRampIsLethal)
{
  TraversabilityGrid grid(TraversabilityConfig{});
  grid.addGroundPoints(sampleGround(-2, 2, -2, 2, ramp(30.0)));
  grid.compute();
  EXPECT_EQ(grid.info(0.1, 0.1).cost, cost::kLethal);
}

TEST(TraversabilityGrid, HighStepIsLethalLowStepIsNot)
{
  auto run = [](double step) {
      TraversabilityGrid grid(TraversabilityConfig{});
      grid.addGroundPoints(
        sampleGround(-2, 2, -2, 2, [step](double x, double) {return x < 0.0 ? 0.0 : step;}));
      grid.compute();
      const bool edge_lethal =
        grid.info(-0.1, 0.1).cost == cost::kLethal || grid.info(0.1, 0.1).cost == cost::kLethal;
      const bool far_free = grid.info(-1.6, 0.1).cost == cost::kFree;
      return std::make_pair(edge_lethal, far_free);
    };

  const auto high = run(0.30);
  EXPECT_TRUE(high.first);
  EXPECT_TRUE(high.second);

  const auto low = run(0.05);
  EXPECT_FALSE(low.first);
  EXPECT_TRUE(low.second);
}

TEST(TraversabilityGrid, ObstacleAboveGroundIsLethal)
{
  TraversabilityGrid grid(TraversabilityConfig{});
  grid.addGroundPoints(sampleGround(-2, 2, -2, 2, [](double, double) {return 0.0;}));
  grid.addObstaclePoints({Eigen::Vector3f(0.6f, 0.6f, 0.4f), Eigen::Vector3f(0.6f, 0.6f, 0.5f)});
  grid.compute();

  EXPECT_EQ(grid.info(0.6, 0.6).cost, cost::kLethal);
  EXPECT_EQ(grid.info(-0.6, -0.6).cost, cost::kFree);
}

TEST(TraversabilityGrid, MisclassifiedPointsOnRampAreIgnored)
{
  // Ground is missing in one cell, and that cell's points arrive as "obstacles"
  // lying exactly on the ramp surface, 0.3 - 0.4 m above the flat reference z = 0.
  const auto on_hole = [](double x, double y) {
      return x >= 1.0 && x < 1.25 && y >= 0.0 && y < 0.25;
    };
  const auto height = ramp(17.5);

  TraversabilityGrid grid(TraversabilityConfig{});
  grid.addGroundPoints(sampleGround(-2, 2, -2, 2, height, on_hole));
  std::vector<Eigen::Vector3f> fake_obstacles;
  for (double x = 1.025; x < 1.25; x += 0.05) {
    for (double y = 0.025; y < 0.25; y += 0.05) {
      fake_obstacles.emplace_back(x, y, height(x, y));
    }
  }
  grid.addObstaclePoints(fake_obstacles);
  grid.compute();

  const auto info = grid.info(1.1, 0.1);
  EXPECT_FALSE(info.lethal_obstacle);
  EXPECT_NE(info.cost, cost::kLethal);
}

TEST(TraversabilityGrid, OverhangIsIgnored)
{
  TraversabilityGrid grid(TraversabilityConfig{});  // max_obstacle_height 1.0
  grid.addGroundPoints(sampleGround(-2, 2, -2, 2, [](double, double) {return 0.0;}));
  grid.addObstaclePoints({Eigen::Vector3f(0.1f, 0.1f, 2.0f)});
  grid.compute();

  EXPECT_FALSE(grid.info(0.1, 0.1).lethal_obstacle);
  EXPECT_EQ(grid.info(0.1, 0.1).cost, cost::kFree);
}

TEST(TraversabilityGrid, ObstaclesWithoutGroundFollowPolicy)
{
  const std::vector<Eigen::Vector3f> far_points{Eigen::Vector3f(10.1f, 10.1f, 0.5f)};

  TraversabilityConfig mark;
  mark.mark_obstacles_without_ground = true;
  TraversabilityGrid grid_mark(mark);
  grid_mark.addObstaclePoints(far_points);
  grid_mark.compute();
  EXPECT_EQ(grid_mark.info(10.1, 10.1).cost, cost::kLethal);

  TraversabilityConfig ignore;
  ignore.mark_obstacles_without_ground = false;
  TraversabilityGrid grid_ignore(ignore);
  grid_ignore.addObstaclePoints(far_points);
  grid_ignore.compute();
  EXPECT_EQ(grid_ignore.info(10.1, 10.1).cost, cost::kNoInformation);
  EXPECT_TRUE(grid_ignore.results().empty());
}
