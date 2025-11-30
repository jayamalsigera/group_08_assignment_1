#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class NavigationGoalNode : public rclcpp::Node
{
public:
    NavigationGoalNode()
    : Node("navigation_goal_node")
    {
        // Declare and get parameters
        publish_rate_ = this->declare_parameter<double>("publish_rate", 10.0);

        // Subscriber to AprilTag center pose
        sub_tag_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/apriltag_center_pose",
            10,
            std::bind(&NavigationGoalNode::tagPoseCallback, this, std::placeholders::_1)
        );

        // Publisher for navigation goal
        pub_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/goal_pose",
            10
        );

        // Timer to publish goal at specified rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
            std::bind(&NavigationGoalNode::publishGoal, this)
        );

        // Log node start
        RCLCPP_INFO(this->get_logger(), "Navigation Goal Node has started.");
    }

private:
    bool received_tag_pose_ = false;
    double publish_rate_;

    geometry_msgs::msg::PoseStamped latest_tag_pose_;
    geometry_msgs::msg::PoseStamped tag_goal_pose_;

    // Subscriber and Publisher
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_tag_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_pose_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback for receiving AprilTag pose
    void tagPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            tag_goal_pose_ = *msg;
            received_tag_pose_ = true;
        }

    // Publish navigation goal based on latest AprilTag pose
    void publishGoal()
    {
        // Check if we have received an AprilTag pose
        if (!received_tag_pose_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 4000,
                                 "Waiting for AprilTag center pose…");
            return;
        }

        // Prepare and publish the goal message
        geometry_msgs::msg::PoseStamped goal_msg = tag_goal_pose_;
        goal_msg.header.stamp = this->get_clock()->now();
        goal_msg.header.frame_id = "map";

        pub_goal_pose_->publish(goal_msg);

        // Log the published goal
        RCLCPP_INFO(this->get_logger(),
                    "GOAL SENT -> (%.3f, %.3f)",
                    goal_msg.pose.position.x,
                    goal_msg.pose.position.y);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationGoalNode>());
    rclcpp::shutdown();
    return 0;
}
