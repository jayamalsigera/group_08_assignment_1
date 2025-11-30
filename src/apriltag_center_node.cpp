#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"

#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


class AprilTagCenterNode : public rclcpp::Node
{
public:
  AprilTagCenterNode() : rclcpp::Node("apriltag_center_node"), has_valid_center_pose_(false)
  {
    // Log node start
    RCLCPP_INFO(this->get_logger(), "AprilTag Center Node started.");

    // Publisher for AprilTag center pose
    center_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/apriltag_center_pose", 10);

    // Subscriber for AprilTag detections
    apriltag_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/apriltag/detections", 10,
        std::bind(&AprilTagCenterNode::detectionsCallback, this, std::placeholders::_1));

    // Subscriber for depth image
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/world/default/model/external_camera/link/link/sensor/depth_camera/depth_image", 10,
        std::bind(&AprilTagCenterNode::depthCallback, this, std::placeholders::_1));

    // Subscriber for depth camera info
    depth_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/depth_camera/camera_info", 10,
        std::bind(&AprilTagCenterNode::depthInfoCallback, this, std::placeholders::_1));

    // TF2 buffer and listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Get frame_id parameter
    this->declare_parameter<std::string>("frame_id", "map");
    this->get_parameter("frame_id", frame_id_);

    // Timer to periodically publish last known center pose
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AprilTagCenterNode::timerCallback, this));
  }

private:
  bool depth_ready_ = false;
  bool intrinsics_ready_ = false;
  bool has_valid_center_pose_;

  cv::Mat depth_img_;

  // Subscribers and Publishers for AprilTag detections, depth image, and depth camera info
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr center_pose_pub_;
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped last_center_pose_;
  sensor_msgs::msg::CameraInfo depth_cam_info_;
  
  std::string frame_id_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;\

  // Callback for depth image
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      auto cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
      depth_img_ = cv_ptr->image.clone();
      depth_ready_ = true;
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Failed to convert depth image.");
    }
  }

  // Callback for depth camera info
  void depthInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    depth_cam_info_ = *msg;
    intrinsics_ready_ = true;
    depth_cam_info_.header.frame_id = "external_camera/link/rgb_camera";
  }

  // Callback for AprilTag detections
  void detectionsCallback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
  {
    // If no detections, mark center pose as invalid
    if (msg->detections.empty()) {
      has_valid_center_pose_ = false;
      return;
    }

    // Ensure depth image and intrinsics are ready
    if (!depth_ready_ || !intrinsics_ready_) {
      RCLCPP_WARN(this->get_logger(), "Depth or intrinsics not ready.");
      return;
    }

    // Get camera intrinsics
    double fx = depth_cam_info_.k[0];
    double fy = depth_cam_info_.k[4];
    double cx = depth_cam_info_.k[2];
    double cy = depth_cam_info_.k[5];
    // Camera frame
    std::string cam_frame = "external_camera/link/rgb_camera";

    double sum_x = 0.0, sum_y = 0.0;
    size_t tag_num = 0;

    // Process each detection
    for (const auto &detection : msg->detections)
    {
      // Extract pixel coordinates of the detection center
      int u = static_cast<int>(detection.centre.x);
      int v = static_cast<int>(detection.centre.y);

      // Check if the pixel coordinates are within the depth image bounds
      if (u < 0 || v < 0 || u >= depth_img_.cols || v >= depth_img_.rows)
        continue;

      // Get depth value at the detection center
      float Z = depth_img_.at<float>(v, u);
      if (Z <= 0.0f || Z > 10.0f)
        continue;

      // Back-project to 3D camera coordinates
      double X = (u - cx) * Z / fx;
      double Y = (v - cy) * Z / fy;

      // Transform to world frame
      geometry_msgs::msg::PointStamped p_cam, p_world;
      p_cam.header.stamp = this->now();
      p_cam.header.frame_id = cam_frame;
      p_cam.point.x = X;
      p_cam.point.y = Y;
      p_cam.point.z = Z;
      
      try {
        tf_buffer_->transform(p_cam, p_world, frame_id_);
      } catch (...) {
        continue;
      }

      // Accumulate world coordinates
      sum_x += p_world.point.x;
      sum_y += p_world.point.y;
      tag_num++;
    }

    if (tag_num == 0)
      return;

    // Compute average center pose
    last_center_pose_.header.stamp = this->now();
    last_center_pose_.header.frame_id = frame_id_;
    last_center_pose_.pose.position.x = sum_x / tag_num;
    last_center_pose_.pose.position.y = sum_y / tag_num;
    last_center_pose_.pose.position.z = 0.0;
    last_center_pose_.pose.orientation.w = 1.0;

    // Publish the center pose
    center_pose_pub_->publish(last_center_pose_);
    has_valid_center_pose_ = true;
  }

  // Timer callback to periodically publish last known center pose
  void timerCallback()
  {
    if (has_valid_center_pose_) {
      center_pose_pub_->publish(last_center_pose_);
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AprilTagCenterNode>());
  rclcpp::shutdown();
  return 0;
}
