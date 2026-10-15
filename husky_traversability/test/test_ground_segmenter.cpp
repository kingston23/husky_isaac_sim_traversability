#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "husky_traversability/ground_segmenter.hpp"

using husky_traversability::GroundSegmenter;
using husky_traversability::GroundSegmenterConfig;
using husky_traversability::PointLabel;

namespace
{

enum Tag : int { FLAT, RAMP, PLATEAU, BOX, WALL, NUM_TAGS };

const double kRampAngle = 20.0 * M_PI / 180.0;

// Flat for x < 0, a 20 deg ramp for 0 <= x < 6, plateau beyond.
double terrainHeight(double x)
{
  if (x < 0.0) {return 0.0;}
  if (x < 6.0) {return x * std::tan(kRampAngle);}
  return 6.0 * std::tan(kRampAngle);
}

struct Scene
{
  std::vector<Eigen::Vector3f> points;
  std::vector<int> tags;
};

Scene makeScene(double robot_x, double robot_y, double blind_radius)
{
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, 0.01);
  Scene scene;
  auto add = [&](double x, double y, double z, int tag) {
      if (std::hypot(x - robot_x, y - robot_y) < blind_radius) {return;}
      scene.points.emplace_back(x, y, z + noise(rng));
      scene.tags.push_back(tag);
    };

  // Terrain on a 10 cm grid, with the box footprint left out.
  for (int i = -100; i < 120; ++i) {
    for (int j = -60; j < 60; ++j) {
      const double x = i * 0.1;
      const double y = j * 0.1;
      if (x >= -6.0 && x < -5.0 && y >= 2.0 && y < 3.0) {continue;}
      add(x, y, terrainHeight(x), x < 0.0 ? FLAT : (x < 6.0 ? RAMP : PLATEAU));
    }
  }

  // Box 1 x 1 x 0.5 m on flat ground: top and the four sides (above 0.15 m).
  for (int i = 0; i <= 20; ++i) {
    for (int j = 0; j <= 20; ++j) {
      add(-6.0 + i * 0.05, 2.0 + j * 0.05, 0.5, BOX);
    }
    for (int k = 3; k <= 10; ++k) {
      const double z = k * 0.05;
      add(-6.0 + i * 0.05, 2.0, z, BOX);
      add(-6.0 + i * 0.05, 3.0, z, BOX);
      add(-6.0, 2.0 + i * 0.05, z, BOX);
      add(-5.0, 2.0 + i * 0.05, z, BOX);
    }
  }

  // Wall, 1 m high, along y = -5.5 on the flat part.
  for (int i = -100; i < -20; ++i) {
    for (int k = 3; k <= 20; ++k) {
      add(i * 0.1, -5.5, k * 0.05, WALL);
    }
  }
  return scene;
}

// Fraction of points per tag that received the given label.
std::vector<double> fractions(
  const Scene & scene, const std::vector<PointLabel> & labels, PointLabel label)
{
  std::vector<double> hit(NUM_TAGS, 0.0), total(NUM_TAGS, 0.0);
  for (std::size_t i = 0; i < labels.size(); ++i) {
    total[scene.tags[i]] += 1.0;
    if (labels[i] == label) {hit[scene.tags[i]] += 1.0;}
  }
  std::vector<double> out(NUM_TAGS, 0.0);
  for (int t = 0; t < NUM_TAGS; ++t) {
    out[t] = total[t] > 0.0 ? hit[t] / total[t] : 0.0;
  }
  return out;
}

}  // namespace

TEST(GroundSegmenter, RobotOnFlatGroundSeesRampAsGround)
{
  const Scene scene = makeScene(-3.0, 0.0, 2.0);
  GroundSegmenter segmenter(GroundSegmenterConfig{});
  std::vector<PointLabel> labels;
  const auto stats = segmenter.segment(
    scene.points, Eigen::Vector3d(-3.0, 0.0, 0.0), Eigen::Vector3d::UnitZ(), labels);

  ASSERT_GT(stats.seed_cells, 0u);
  const auto ground = fractions(scene, labels, PointLabel::Ground);
  const auto obstacle = fractions(scene, labels, PointLabel::Obstacle);
  EXPECT_GT(ground[FLAT], 0.97);
  EXPECT_GT(ground[RAMP], 0.95);
  EXPECT_GT(ground[PLATEAU], 0.95);
  EXPECT_GT(obstacle[BOX], 0.99);
  EXPECT_GT(obstacle[WALL], 0.99);
}

TEST(GroundSegmenter, RobotOnRampWithTiltedSeed)
{
  const double x = 3.0;
  const Scene scene = makeScene(x, 0.0, 2.0);
  GroundSegmenter segmenter(GroundSegmenterConfig{});
  std::vector<PointLabel> labels;
  const Eigen::Vector3d seed(x, 0.0, terrainHeight(x));
  const Eigen::Vector3d normal(-std::sin(kRampAngle), 0.0, std::cos(kRampAngle));
  const auto stats = segmenter.segment(scene.points, seed, normal, labels);

  ASSERT_GT(stats.seed_cells, 0u);
  const auto ground = fractions(scene, labels, PointLabel::Ground);
  const auto obstacle = fractions(scene, labels, PointLabel::Obstacle);
  EXPECT_GT(ground[FLAT], 0.95);
  EXPECT_GT(ground[RAMP], 0.95);
  EXPECT_GT(ground[PLATEAU], 0.95);
  EXPECT_GT(obstacle[BOX], 0.99);
  EXPECT_GT(obstacle[WALL], 0.99);
}

TEST(GroundSegmenter, WrongSeedHeightFallsBackToObstacles)
{
  const Scene scene = makeScene(-3.0, 0.0, 2.0);
  GroundSegmenter segmenter(GroundSegmenterConfig{});
  std::vector<PointLabel> labels;
  // Seed 2 m above the terrain, e.g. a wrong seed_z_offset.
  const auto stats = segmenter.segment(
    scene.points, Eigen::Vector3d(-3.0, 0.0, 2.0), Eigen::Vector3d::UnitZ(), labels);

  EXPECT_EQ(stats.seed_cells, 0u);
  EXPECT_EQ(stats.ground_points, 0u);
}
