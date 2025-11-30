#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class InitialPoseNode : public rclcpp::Node
{
public:
    InitialPoseNode() : Node("initial_pose_node"), pose_sent_(false)
    {
        // Publisher for initial pose
        initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // Timer to check if AMCL is ready
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&InitialPoseNode::checkAmclReady, this)
        );

        // Log node start
        RCLCPP_INFO(this->get_logger(), "Initial Pose Node has started.");
    }

private:

    // Check if AMCL is ready by verifying the existence of the /amcl_pose topic
    void checkAmclReady()
    {
        auto topics = this->get_topic_names_and_types();

        // Check for /amcl_pose topic
        bool amcl_ready = false;
        for (const auto &t : topics)
        {
            if (t.first == "/amcl_pose")
                amcl_ready = true;
        }

        // If AMCL is not ready, log a warning
        if (!amcl_ready)
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for AMCL to be ready...");
            return;
        }

        // If AMCL is ready and initial pose not yet sent, publish it
        if (!pose_sent_)
        {
            publishInitialPose();
            pose_sent_ = true;
            timer_->cancel();
        }
    }

    // Publish the initial pose to /initialpose topic
    void publishInitialPose()
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;

        msg.header.stamp = this->now();
        msg.header.frame_id = "map";

        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.position.z = 0.0;

        msg.pose.pose.orientation.z = 0.0;
        msg.pose.pose.orientation.w = 1.0;

        for (int i = 0; i < 36; i++)
            msg.pose.covariance[i] = 0.0;

        initial_pose_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Initial pose published.");
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    bool pose_sent_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InitialPoseNode>());
    rclcpp::shutdown();
    return 0;
}
