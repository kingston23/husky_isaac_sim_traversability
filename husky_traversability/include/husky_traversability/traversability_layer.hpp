#ifndef HUSKY_TRAVERSABILITY__TRAVERSABILITY_LAYER_HPP_
#define HUSKY_TRAVERSABILITY__TRAVERSABILITY_LAYER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "husky_traversability/traversability_grid.hpp"

namespace husky_traversability
{

/// Nav2 costmap layer that consumes segmented ground / obstacle clouds and marks
/// slope, step and obstacle costs relative to the local ground.
class TraversabilityLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TraversabilityLayer() = default;
  ~TraversabilityLayer() override = default;

  void onInitialize() override;
  void reset() override;
  bool isClearable() override {return true;}

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  struct Observation
  {
    rclcpp::Time stamp;
    std::vector<Eigen::Vector3f> points;
  };

  struct WorldBounds
  {
    double min_x{0.0};
    double min_y{0.0};
    double max_x{0.0};
    double max_y{0.0};
    bool valid{false};

    void expand(double x, double y);
  };

  template<typename T>
  T getParam(const std::string & name, const T & default_value);

  void cloudCallback(const sensor_msgs::msg::PointCloud2 & msg, std::deque<Observation> & buffer);
  bool readCloud(const sensor_msgs::msg::PointCloud2 & msg, std::vector<Eigen::Vector3f> & points);
  void resetWorldBounds(const WorldBounds & bounds);
  WorldBounds paintGrid();
  static void reportBounds(
    const WorldBounds & bounds, double * min_x, double * min_y, double * max_x, double * max_y);

  // Parameters
  std::string global_frame_;
  bool rolling_window_{false};
  double observation_persistence_{0.0};
  double data_timeout_{0.0};
  double transform_tolerance_{0.2};

  // ROS interfaces
  rclcpp::Logger layer_logger_{rclcpp::get_logger("traversability_layer")};
  rclcpp::Clock::SharedPtr layer_clock_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr ground_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_sub_;

  // Shared between subscription callbacks and the costmap update thread
  std::mutex data_mutex_;
  std::deque<Observation> ground_buffer_;
  std::deque<Observation> obstacle_buffer_;
  bool new_data_{false};
  bool has_data_{false};
  rclcpp::Time last_receive_time_{0, 0, RCL_ROS_TIME};

  // Only used in the update thread
  std::unique_ptr<TraversabilityGrid> grid_;
  WorldBounds painted_bounds_;
};

}  // namespace husky_traversability

#endif  // HUSKY_TRAVERSABILITY__TRAVERSABILITY_LAYER_HPP_
