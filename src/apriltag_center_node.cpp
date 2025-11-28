#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class AprilTagCenterNode : public rclcpp::Node
{
public:
  AprilTagCenterNode() : rclcpp::Node("apriltag_center_node"), has_valid_center_pose_(false)
  {
    RCLCPP_INFO(this->get_logger(), "AprilTag Center Node started."); // Log info

    center_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/apriltag_center_pose", 10);

    apriltag_sub = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/apriltag_detections", 10,
        std::bind(&AprilTagCenterNode::detectionsCallback, this, std::placeholders::_1));

    this->declare_parameter<std::string>("frame_id", "map");
    this->get_parameter("frame_id", frame_id_);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AprilTagCenterNode::timerCallback, this));
  }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr center_pose_pub_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string frame_id_;
    geometry_msgs::msg::PoseStamped last_center_pose_;
    bool has_valid_center_pose_;
    
    
    void detectionsCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
      if (msg->detections.empty())
      {
        has_valid_center_pose_ = false;
        return;
      }

      double sum_x = 0.0, sum_y = 0.0;

      for (const auto &detection : msg->detections)
      {
        sum_x += detection.centre.x;
        sum_y += detection.centre.y;
      }

      size_t tag_num = msg->detections.size();

      last_center_pose_.header.stamp = this->now();
      last_center_pose_.header.frame_id = frame_id_;

      last_center_pose_.pose.position.x = sum_x / tag_num;
      last_center_pose_.pose.position.y = sum_y / tag_num;

    RCLCPP_INFO(
        this->get_logger(),
        "CENTER POINT → x: %.3f   y: %.3f   (from %zu detections)",
        last_center_pose_.pose.position.x,
        last_center_pose_.pose.position.y,
        tag_num
    );

      last_center_pose_.pose.orientation.w = 1.0;

      has_valid_center_pose_ = true;

      center_pose_pub_->publish(last_center_pose_);
    }

    void timerCallback()
    {
      if (has_valid_center_pose_ && center_pose_pub_)
      {
        center_pose_pub_->publish(last_center_pose_);
      }
    }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);                               // Initialize ROS2
  rclcpp::spin(std::make_shared<AprilTagCenterNode>());  // Keep the node running
  rclcpp::shutdown();                                     // Shutdown ROS2
  return 0;                                               // Exit program
}