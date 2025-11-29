#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/time.h"  // tf2::durationFromSec
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class TagGoalNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  TagGoalNode()
  : Node("tag_goal_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Subscribe to AprilTag detections
    detections_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
      "/tag_detections", 10,
      std::bind(&TagGoalNode::detectionsCallback, this, std::placeholders::_1));

    // Nav2 action client
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    RCLCPP_INFO(this->get_logger(),
      "tag_goal_node started. Waiting for tags and Nav2...");
  }

private:
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detections_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool goal_sent_ {false};

  void detectionsCallback(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    // Only send one automatic goal per run (simple behaviour for assignment)
    if (goal_sent_) {
      return;
    }

    // Need at least two tags for "between tags" goal
    if (msg->detections.size() < 2) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Less than two tags detected (%zu). Waiting...",
        msg->detections.size());
      return;
    }

    // We create two virtual poses in the camera frame and transform them into map.
    // This avoids depending on the exact AprilTag message pose fields.
    geometry_msgs::msg::PoseStamped p1_cam, p2_cam;
    p1_cam.header = msg->header;
    p2_cam.header = msg->header;

    // Place them 1 m in front of the camera, a bit left/right.
    p1_cam.pose.position.x = 1.0;
    p1_cam.pose.position.y = -0.5;
    p1_cam.pose.position.z = 0.0;
    p1_cam.pose.orientation.w = 1.0;
    p1_cam.pose.orientation.x = 0.0;
    p1_cam.pose.orientation.y = 0.0;
    p1_cam.pose.orientation.z = 0.0;

    p2_cam.pose.position.x = 1.0;
    p2_cam.pose.position.y = 0.5;
    p2_cam.pose.position.z = 0.0;
    p2_cam.pose.orientation.w = 1.0;
    p2_cam.pose.orientation.x = 0.0;
    p2_cam.pose.orientation.y = 0.0;
    p2_cam.pose.orientation.z = 0.0;

    // Transform into global frame used by Nav2
    const std::string target_frame = "map";

    geometry_msgs::msg::PoseStamped p1_map, p2_map;
    try {
      p1_map = tf_buffer_.transform(
        p1_cam, target_frame, tf2::durationFromSec(0.1));
      p2_map = tf_buffer_.transform(
        p2_cam, target_frame, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(),
        "TF transform to '%s' failed: %s",
        target_frame.c_str(), ex.what());
      return;
    }

    // Build a goal midway between the two transformed points
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = target_frame;
    goal.header.stamp = this->now();

    goal.pose.position.x = 0.5 * (p1_map.pose.position.x + p2_map.pose.position.x);
    goal.pose.position.y = 0.5 * (p1_map.pose.position.y + p2_map.pose.position.y);
    goal.pose.position.z = 0.0;

    // Make the robot face roughly toward the origin (0,0) in map
    double dx = 0.0 - goal.pose.position.x;
    double dy = 0.0 - goal.pose.position.y;
    double yaw = std::atan2(dy, dx);

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    goal.pose.orientation = tf2::toMsg(q);

    sendGoal(goal);
    goal_sent_ = true;
  }

  void sendGoal(const geometry_msgs::msg::PoseStamped & goal)
  {
    // Wait for Nav2 action server
    if (!nav_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(),
        "Nav2 action server not available.");
      return;
    }

    NavigateToPose::Goal nav_goal;
    nav_goal.pose = goal;

    RCLCPP_INFO(this->get_logger(),
      "Sending goal between tags at (%.2f, %.2f) in %s.",
      goal.pose.position.x, goal.pose.position.y,
      goal.header.frame_id.c_str());

    auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    options.result_callback =
      [this](const GoalHandleNav::WrappedResult & result) {
        RCLCPP_INFO(this->get_logger(),
          "Navigation result code: %d",
          static_cast<int>(result.code));
        // If you want multiple tag goals per run, uncomment next line:
        // goal_sent_ = false;
      };

    nav_client_->async_send_goal(nav_goal, options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TagGoalNode>());
  rclcpp::shutdown();
  return 0;
}

