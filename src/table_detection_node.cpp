#include <memory>
#include <vector>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"  // for tf2::durationFromSec

using std::placeholders::_1;

class TableDetectionNode : public rclcpp::Node
{
public:
  TableDetectionNode()
  : Node("table_detection_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&TableDetectionNode::scanCallback, this, _1));

    tables_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/tables_odom", 10);

    RCLCPP_INFO(this->get_logger(), "table_detection_node started.");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr tables_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto & ranges = msg->ranges;
    const int N = static_cast<int>(ranges.size());

    const float max_gap = 0.1f;
    const int min_points = 5;
    const int max_points = 200;

    std::vector<std::vector<int>> clusters;
    std::vector<int> current;

    auto flush_cluster = [&]() {
      if ((int)current.size() >= min_points && (int)current.size() <= max_points) {
        clusters.push_back(current);
      }
      current.clear();
    };

    float prev_r = std::numeric_limits<float>::quiet_NaN();

    for (int i = 0; i < N; ++i) {
      float r = ranges[i];
      if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
        if (!current.empty()) {
          flush_cluster();
        }
        prev_r = std::numeric_limits<float>::quiet_NaN();
        continue;
      }

      if (std::isfinite(prev_r) && std::fabs(r - prev_r) > max_gap) {
        if (!current.empty()) {
          flush_cluster();
        }
      }

      current.push_back(i);
      prev_r = r;
    }
    if (!current.empty()) {
      flush_cluster();
    }

    geometry_msgs::msg::PoseArray tables_odom;
    tables_odom.header.stamp = this->now();
    tables_odom.header.frame_id = "odom";

    for (const auto & cl : clusters) {
      int mid = cl[cl.size() / 2];
      float r = ranges[mid];
      float angle_mid = msg->angle_min + mid * msg->angle_increment;

      geometry_msgs::msg::PoseStamped p_base;
      p_base.header = msg->header;
      p_base.pose.position.x = r * std::cos(angle_mid);
      p_base.pose.position.y = r * std::sin(angle_mid);
      p_base.pose.position.z = 0.0;
      p_base.pose.orientation.w = 1.0;
      p_base.pose.orientation.x = 0.0;
      p_base.pose.orientation.y = 0.0;
      p_base.pose.orientation.z = 0.0;

      geometry_msgs::msg::PoseStamped p_odom;
      try {
        p_odom = tf_buffer_.transform(
          p_base, "odom", tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(),
          "TF transform to odom failed: %s", ex.what());
        continue;
      }

      tables_odom.poses.push_back(p_odom.pose);
    }

    tables_pub_->publish(tables_odom);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Detected %zu potential tables.", tables_odom.poses.size());

    for (size_t i = 0; i < tables_odom.poses.size(); ++i) {
      const auto & p = tables_odom.poses[i];
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Table %zu in odom: (%.2f, %.2f)", i,
        p.position.x, p.position.y);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TableDetectionNode>());
  rclcpp::shutdown();
  return 0;
}

