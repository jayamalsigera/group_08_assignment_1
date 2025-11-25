#include "rclcpp/rclcpp.hpp"

class TurtlebotNode : public rclcpp::Node
{
public:
    TurtlebotNode() : Node("turtlebot_node")
    {
        RCLCPP_INFO(this->get_logger(), "Turtlebot node started.");
    }
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlebotNode>());
    rclcpp::shutdown();
    return 0;
}
