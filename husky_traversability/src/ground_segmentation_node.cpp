#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "husky_traversability/ground_segmenter.hpp"

namespace husky_traversability
{

class GroundSegmentationNode : public rclcpp::Node
{
public:
  explicit GroundSegmentationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("ground_segmentation", options)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar/point_cloud");
    target_frame_ = declare_parameter<std::string>("target_frame", "odom");
    seed_frame_ = declare_parameter<std::string>("seed_frame", "base_link");
    seed_z_offset_ = declare_parameter<double>("seed_z_offset", -0.13);
    min_range_ = declare_parameter<double>("min_range", 0.8);
    max_range_ = declare_parameter<double>("max_range", 30.0);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.1);

    GroundSegmenterConfig cfg;
    cfg.cell_size = declare_parameter<double>("cell_size", cfg.cell_size);
    cfg.low_band = declare_parameter<double>("low_band", cfg.low_band);
    cfg.min_points_per_cell = static_cast<int>(
      declare_parameter<int64_t>("min_points_per_cell", cfg.min_points_per_cell));
    cfg.max_slope_deg = declare_parameter<double>("max_slope_deg", cfg.max_slope_deg);
    cfg.max_planarity = declare_parameter<double>("max_planarity", cfg.max_planarity);
    cfg.min_linearity_ratio =
      declare_parameter<double>("min_linearity_ratio", cfg.min_linearity_ratio);
    cfg.max_step = declare_parameter<double>("max_step", cfg.max_step);
    cfg.max_slope_change_deg =
      declare_parameter<double>("max_slope_change_deg", cfg.max_slope_change_deg);
    cfg.search_radius = declare_parameter<double>("search_radius", cfg.search_radius);
    cfg.seed_radius = declare_parameter<double>("seed_radius", cfg.seed_radius);
    cfg.seed_height_tolerance =
      declare_parameter<double>("seed_height_tolerance", cfg.seed_height_tolerance);
    cfg.ground_inlier_threshold =
      declare_parameter<double>("ground_inlier_threshold", cfg.ground_inlier_threshold);
    segmenter_ = std::make_unique<GroundSegmenter>(cfg);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/ground", rclcpp::QoS(5));
    obstacles_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("~/obstacles", rclcpp::QoS(5));

    // SensorDataQoS (best effort) also connects to reliable publishers such as Isaac Sim.
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&GroundSegmentationNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Segmenting '%s' in frame '%s', seed frame '%s'",
      input_topic_.c_str(), target_frame_.c_str(), seed_frame_.c_str());
  }

private:
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const auto t_start = std::chrono::steady_clock::now();

    // Both transforms at the cloud's timestamp: the cloud must end up in a gravity-aligned frame.
    Eigen::Isometry3d T_target_sensor;
    Eigen::Isometry3d T_target_seed;
    try {
      const rclcpp::Time stamp(msg->header.stamp);
      const auto timeout = rclcpp::Duration::from_seconds(tf_timeout_);
      T_target_sensor = tf2::transformToEigen(
        tf_buffer_->lookupTransform(target_frame_, msg->header.frame_id, stamp, timeout));
      T_target_seed = tf2::transformToEigen(
        tf_buffer_->lookupTransform(target_frame_, seed_frame_, stamp, timeout));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return;
    }

    // Read, range-filter and transform the points.
    std::vector<Eigen::Vector3f> points;
    try {
      const std::size_t n = static_cast<std::size_t>(msg->width) * msg->height;
      points.reserve(n);
      sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
      const double min_r2 = min_range_ * min_range_;
      const double max_r2 = max_range_ * max_range_;
      for (std::size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z) {
        const Eigen::Vector3d p_sensor(*it_x, *it_y, *it_z);
        if (!p_sensor.allFinite()) {
          continue;
        }
        const double r2 = p_sensor.squaredNorm();
        if (r2 < min_r2 || r2 > max_r2) {
          continue;
        }
        points.emplace_back((T_target_sensor * p_sensor).cast<float>());
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Bad point cloud: %s", ex.what());
      return;
    }

    // Seed: ground point under the robot and the robot's up axis.
    const Eigen::Vector3d seed_point = T_target_seed * Eigen::Vector3d(0.0, 0.0, seed_z_offset_);
    const Eigen::Vector3d seed_normal = T_target_seed.linear() * Eigen::Vector3d::UnitZ();

    std::vector<PointLabel> labels;
    const SegmentationStats stats = segmenter_->segment(points, seed_point, seed_normal, labels);

    std::vector<Eigen::Vector3f> ground;
    std::vector<Eigen::Vector3f> obstacles;
    ground.reserve(stats.ground_points);
    obstacles.reserve(points.size() - stats.ground_points);
    for (std::size_t i = 0; i < points.size(); ++i) {
      (labels[i] == PointLabel::Ground ? ground : obstacles).push_back(points[i]);
    }

    std_msgs::msg::Header header;
    header.stamp = msg->header.stamp;
    header.frame_id = target_frame_;
    ground_pub_->publish(toCloud(header, ground));
    obstacles_pub_->publish(toCloud(header, obstacles));

    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_start).count();

    if (!points.empty() && stats.seed_cells == 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No ground found around the robot, all points are published as obstacles. "
        "Check seed_frame, seed_z_offset, seed_radius and the TF tree.");
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "points %zu | ground %zu | cells %zu (candidate %zu, seed %zu, ground %zu) | %.1f ms",
      points.size(), stats.ground_points, stats.occupied_cells, stats.candidate_cells,
      stats.seed_cells, stats.ground_cells, elapsed_ms);
  }

  static sensor_msgs::msg::PointCloud2 toCloud(
    const std_msgs::msg::Header & header, const std::vector<Eigen::Vector3f> & points)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    for (const auto & p : points) {
      *it_x = p.x();
      *it_y = p.y();
      *it_z = p.z();
      ++it_x;
      ++it_y;
      ++it_z;
    }
    cloud.is_dense = true;
    return cloud;
  }

  std::string input_topic_;
  std::string target_frame_;
  std::string seed_frame_;
  double seed_z_offset_{0.0};
  double min_range_{0.0};
  double max_range_{0.0};
  double tf_timeout_{0.1};

  std::unique_ptr<GroundSegmenter> segmenter_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

}  // namespace husky_traversability

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<husky_traversability::GroundSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
