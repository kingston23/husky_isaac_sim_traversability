#include "husky_traversability/traversability_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>

namespace husky_traversability
{

// The grid uses its own constants so it can be tested without Nav2.
static_assert(cost::kLethal == nav2_costmap_2d::LETHAL_OBSTACLE, "cost mismatch");
static_assert(cost::kNoInformation == nav2_costmap_2d::NO_INFORMATION, "cost mismatch");
static_assert(cost::kFree == nav2_costmap_2d::FREE_SPACE, "cost mismatch");

void TraversabilityLayer::WorldBounds::expand(double x, double y)
{
  if (!valid) {
    min_x = max_x = x;
    min_y = max_y = y;
    valid = true;
    return;
  }
  min_x = std::min(min_x, x);
  min_y = std::min(min_y, y);
  max_x = std::max(max_x, x);
  max_y = std::max(max_y, y);
}

template<typename T>
T TraversabilityLayer::getParam(const std::string & name, const T & default_value)
{
  declareParameter(name, rclcpp::ParameterValue(default_value));
  T value = default_value;
  node_.lock()->get_parameter(name_ + "." + name, value);
  return value;
}

// -----------------------------------------------------------------------------
// Initialisation
// -----------------------------------------------------------------------------
void TraversabilityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TraversabilityLayer: failed to lock node");
  }
  layer_logger_ = node->get_logger().get_child(name_);
  layer_clock_ = node->get_clock();

  enabled_ = getParam<bool>("enabled", true);
  const auto ground_topic =
    getParam<std::string>("ground_topic", "/ground_segmentation/ground");
  const auto obstacles_topic =
    getParam<std::string>("obstacles_topic", "/ground_segmentation/obstacles");
  observation_persistence_ = getParam<double>("observation_persistence", 0.0);
  data_timeout_ = getParam<double>("data_timeout", 0.0);
  transform_tolerance_ = getParam<double>("transform_tolerance", 0.2);

  TraversabilityConfig cfg;
  cfg.cell_size = getParam<double>("cell_size", cfg.cell_size);
  cfg.min_ground_points = getParam<int>("min_ground_points", cfg.min_ground_points);
  cfg.slope_radius = getParam<double>("slope_radius", cfg.slope_radius);
  cfg.min_linearity_ratio = getParam<double>("min_linearity_ratio", cfg.min_linearity_ratio);
  cfg.slope_safe_deg = getParam<double>("slope_safe_deg", cfg.slope_safe_deg);
  cfg.slope_lethal_deg = getParam<double>("slope_lethal_deg", cfg.slope_lethal_deg);
  cfg.max_slope_cost = getParam<int>("max_slope_cost", cfg.max_slope_cost);
  cfg.max_step_height = getParam<double>("max_step_height", cfg.max_step_height);
  cfg.min_obstacle_height = getParam<double>("min_obstacle_height", cfg.min_obstacle_height);
  cfg.max_obstacle_height = getParam<double>("max_obstacle_height", cfg.max_obstacle_height);
  cfg.ground_search_radius = getParam<double>("ground_search_radius", cfg.ground_search_radius);
  cfg.mark_obstacles_without_ground =
    getParam<bool>("mark_obstacles_without_ground", cfg.mark_obstacles_without_ground);
  grid_ = std::make_unique<TraversabilityGrid>(cfg);

  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();
  default_value_ = nav2_costmap_2d::NO_INFORMATION;
  matchSize();
  current_ = true;

  // Subscriptions run in the costmap's callback group (its own executor thread).
  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  ground_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    ground_topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      cloudCallback(*msg, ground_buffer_);
    },
    options);
  obstacles_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    obstacles_topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      cloudCallback(*msg, obstacle_buffer_);
    },
    options);

  RCLCPP_INFO(
    layer_logger_,
    "TraversabilityLayer: ground '%s', obstacles '%s', frame '%s', %s, cell %.2f m, "
    "slope safe/lethal %.1f/%.1f deg, step %.2f m",
    ground_topic.c_str(), obstacles_topic.c_str(), global_frame_.c_str(),
    rolling_window_ ? "rolling" : "static", cfg.cell_size,
    cfg.slope_safe_deg, cfg.slope_lethal_deg, cfg.max_step_height);
}

// -----------------------------------------------------------------------------
// Data input
// -----------------------------------------------------------------------------
bool TraversabilityLayer::readCloud(
  const sensor_msgs::msg::PointCloud2 & msg, std::vector<Eigen::Vector3f> & points)
{
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  if (msg.header.frame_id != global_frame_) {
    try {
      T = tf2::transformToEigen(
        tf_->lookupTransform(
          global_frame_, msg.header.frame_id, rclcpp::Time(msg.header.stamp, RCL_ROS_TIME),
          rclcpp::Duration::from_seconds(transform_tolerance_)));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        layer_logger_, *layer_clock_, 2000, "TF %s -> %s failed: %s",
        msg.header.frame_id.c_str(), global_frame_.c_str(), ex.what());
      return false;
    }
  }

  try {
    const std::size_t n = static_cast<std::size_t>(msg.width) * msg.height;
    points.reserve(n);
    sensor_msgs::PointCloud2ConstIterator<float> it_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(msg, "z");
    for (std::size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z) {
      const Eigen::Vector3d p(*it_x, *it_y, *it_z);
      if (p.allFinite()) {
        points.emplace_back((T * p).cast<float>());
      }
    }
  } catch (const std::runtime_error & ex) {
    RCLCPP_ERROR_THROTTLE(layer_logger_, *layer_clock_, 5000, "Bad point cloud: %s", ex.what());
    return false;
  }
  return true;
}

void TraversabilityLayer::cloudCallback(
  const sensor_msgs::msg::PointCloud2 & msg, std::deque<Observation> & buffer)
{
  Observation observation;
  observation.stamp = rclcpp::Time(msg.header.stamp, RCL_ROS_TIME);
  if (!readCloud(msg, observation.points)) {
    return;
  }

  std::lock_guard<std::mutex> lock(data_mutex_);
  // A jump back in time (simulation reset) invalidates older observations.
  if (!buffer.empty() && observation.stamp < buffer.back().stamp) {
    buffer.clear();
  }
  const rclcpp::Time newest = observation.stamp;
  buffer.push_back(std::move(observation));
  while (buffer.size() > 1 &&
    (observation_persistence_ <= 0.0 ||
    (newest - buffer.front().stamp).seconds() > observation_persistence_))
  {
    buffer.pop_front();
  }

  last_receive_time_ = layer_clock_->now();
  has_data_ = true;
  new_data_ = true;
}

// -----------------------------------------------------------------------------
// Costmap update
// -----------------------------------------------------------------------------
void TraversabilityLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2.0, robot_y - getSizeInMetersY() / 2.0);
  }
  if (!enabled_) {
    return;
  }

  std::vector<Eigen::Vector3f> ground;
  std::vector<Eigen::Vector3f> obstacles;
  bool recompute = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (has_data_ && data_timeout_ > 0.0 &&
      (layer_clock_->now() - last_receive_time_).seconds() > data_timeout_)
    {
      RCLCPP_WARN(
        layer_logger_, "No segmented clouds for %.1f s, clearing the layer", data_timeout_);
      ground_buffer_.clear();
      obstacle_buffer_.clear();
      has_data_ = false;
      recompute = true;
    }

    if (new_data_) {
      for (const auto & obs : ground_buffer_) {
        ground.insert(ground.end(), obs.points.begin(), obs.points.end());
      }
      for (const auto & obs : obstacle_buffer_) {
        obstacles.insert(obstacles.end(), obs.points.begin(), obs.points.end());
      }
      new_data_ = false;
      recompute = true;
    }

    current_ = has_data_ || data_timeout_ <= 0.0;
  }

  if (recompute) {
    grid_->clear();
    grid_->addGroundPoints(ground);
    grid_->addObstaclePoints(obstacles);
    grid_->compute();

    // Clear what was painted last time, paint the new result and report both areas.
    const WorldBounds previous = painted_bounds_;
    resetWorldBounds(previous);
    painted_bounds_ = paintGrid();
    reportBounds(previous, min_x, min_y, max_x, max_y);
  }
  reportBounds(painted_bounds_, min_x, min_y, max_x, max_y);
}

void TraversabilityLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  // Keep the highest cost of all layers; NO_INFORMATION cells of this layer are skipped.
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void TraversabilityLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  ground_buffer_.clear();
  obstacle_buffer_.clear();
  new_data_ = false;
  has_data_ = false;
  resetMaps();
  painted_bounds_ = WorldBounds{};
  current_ = data_timeout_ <= 0.0;
}

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
void TraversabilityLayer::resetWorldBounds(const WorldBounds & bounds)
{
  if (!bounds.valid) {
    return;
  }
  int x0, y0, x1, y1;
  worldToMapEnforceBounds(bounds.min_x, bounds.min_y, x0, y0);
  worldToMapEnforceBounds(bounds.max_x, bounds.max_y, x1, y1);
  resetMap(x0, y0, x1 + 1, y1 + 1);  // upper bounds are exclusive
}

TraversabilityLayer::WorldBounds TraversabilityLayer::paintGrid()
{
  WorldBounds painted;
  const double resolution = getResolution();
  const double origin_x = getOriginX();
  const double origin_y = getOriginY();
  const int size_x = static_cast<int>(getSizeInCellsX());
  const int size_y = static_cast<int>(getSizeInCellsY());

  for (const auto & cell : grid_->results()) {
    // Costmap cells whose centre lies inside the analysis cell [min, max).
    const int mx0 = std::max(
      0, static_cast<int>(std::ceil((cell.min_x - origin_x) / resolution - 0.5)));
    const int mx1 = std::min(
      size_x - 1, static_cast<int>(std::ceil((cell.max_x - origin_x) / resolution - 0.5)) - 1);
    const int my0 = std::max(
      0, static_cast<int>(std::ceil((cell.min_y - origin_y) / resolution - 0.5)));
    const int my1 = std::min(
      size_y - 1, static_cast<int>(std::ceil((cell.max_y - origin_y) / resolution - 0.5)) - 1);
    if (mx0 > mx1 || my0 > my1) {
      continue;
    }
    for (int my = my0; my <= my1; ++my) {
      for (int mx = mx0; mx <= mx1; ++mx) {
        setCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), cell.cost);
      }
    }
    painted.expand(cell.min_x, cell.min_y);
    painted.expand(cell.max_x, cell.max_y);
  }
  return painted;
}

void TraversabilityLayer::reportBounds(
  const WorldBounds & bounds, double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!bounds.valid) {
    return;
  }
  *min_x = std::min(*min_x, bounds.min_x);
  *min_y = std::min(*min_y, bounds.min_y);
  *max_x = std::max(*max_x, bounds.max_x);
  *max_y = std::max(*max_y, bounds.max_y);
}

}  // namespace husky_traversability

PLUGINLIB_EXPORT_CLASS(husky_traversability::TraversabilityLayer, nav2_costmap_2d::Layer)
