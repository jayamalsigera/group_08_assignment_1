#include "rclcpp/rclcpp.hpp"
#include <rclcpp/subscription_options.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"



class TurtlebotNode : public rclcpp::Node
{
public:
    TurtlebotNode() : Node("turtlebot_node")
    {
        RCLCPP_INFO(this->get_logger(), "Turtlebot node started.");

        // Subscription to LIDAR data
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;

        lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::QoS(10),
            std::bind(&TurtlebotNode::lidar_callback, this, std::placeholders::_1),
            rclcpp::SubscriptionOptions{}
        );

    }


private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {

    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotNode>());
    rclcpp::shutdown();
    return 0;
}
