#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

class CameraRelayNode : public rclcpp::Node
{
public:
    CameraRelayNode() : Node("camera_relay_node")
    {   
        // Subscribers and Publishers for camera image and camera info
        img_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/rgb_camera/image", 10,
            [this](sensor_msgs::msg::Image::SharedPtr msg) {
                image_pub_->publish(*msg);
            });

        // Camera info subscriber and publisher
        caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/rgb_camera/camera_info", 10,
            [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
                caminfo_pub_->publish(*msg);
            });

        // Publisher for relayed camera image
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/camera/image_rect", 10);

        // Publisher for camera info
        caminfo_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/color/camera_info", 10);

        // Log node start
        RCLCPP_INFO(this->get_logger(), "Camera Relay Node has started.");
    }

private:
    // Subscribers and Publishers for camera image and camera info
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraRelayNode>());
    rclcpp::shutdown();
    return 0;
}
