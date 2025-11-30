#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <memory>
#include <vector>
#include <cmath>

class TurtlebotServer : public rclcpp::Node
{
public:
    TurtlebotServer()
    : Node("turtlebot_server")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscription to LIDAR data
        lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&TurtlebotServer::lidarTableDetection, this, std::placeholders::_1)
        );

        sub_goal_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::SharedPtr msg){
                current_goal_ = *msg;
                got_goal_ = true;
            }
        );

        sub_amcl_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
                current_amcl_ = *msg;
                got_amcl_ = true;
            }
        );

        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&TurtlebotServer::checkGoalReached, this)
        );

    }

private:

    geometry_msgs::msg::PoseStamped current_goal_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_amcl_;
    bool got_goal_ = false;
    bool got_amcl_ = false;
    bool reached_goal = false;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_amcl_pose_;
    rclcpp::TimerBase::SharedPtr check_timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<std::pair<double, double>> table_positions;
    std::vector<std::pair<double, double>> table_positions_odom;

    //Lidar callback to process scan data and detect tables
    void checkGoalReached()
    {
        if (!got_goal_ || !got_amcl_)
            return;

        double gx = current_goal_.pose.position.x;
        double gy = current_goal_.pose.position.y;

        double rx = current_amcl_.pose.pose.position.x;
        double ry = current_amcl_.pose.pose.position.y;

        double d = std::hypot(gx - rx, gy - ry);

        if (d < 0.10) {
            reached_goal = true;
        }
    }

    void lidarTableDetection(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!reached_goal)
        {
            return;
        }

        table_positions.clear();

        const auto &ranges = msg->ranges;
        double angle = msg->angle_min;
        double angle_increment = msg->angle_increment;

        // Detect jumps in range to identify clusters
        std::vector<std::pair<double,double>> current_cluster;
        const double jump_thresh = 0.55;  // meters - threshold to detect a jump

        // Process each point in the scan
        for (size_t i = 1; i < ranges.size(); i++, angle += angle_increment) {
            double r_prev = ranges[i-1];
            double r_curr = ranges[i];

            if (!std::isfinite(r_curr)) continue;

            // Detect jumps in range to identify clusters
            if (std::abs(r_curr - r_prev) > jump_thresh) {
                if (current_cluster.size() > 5 && current_cluster.size() < 30) {
                    double rmin = 1e9, rmax = 0.0;
                    for (auto &p : current_cluster)
                    {
                        double r = std::hypot(p.first, p.second);
                        rmin = std::min(rmin, r);
                        rmax = std::max(rmax, r);
                    }
                    double curvature = std::abs(rmax - rmin);

                    if (curvature < 0.12 || curvature > 0.6) {
                        current_cluster.clear();
                        continue;
                    }
                    
                    double sum_x = 0, sum_y = 0;
                    for (auto &p : current_cluster) {
                        sum_x += p.first;
                        sum_y += p.second;
                    }
                    table_positions.push_back({sum_x / current_cluster.size(), sum_y / current_cluster.size()});
                }
                current_cluster.clear();
            }
            // Calculate x, y coordinates of the current point
            double x = r_curr * std::cos(msg->angle_min + i * angle_increment);
            double y = r_curr * std::sin(msg->angle_min + i * angle_increment);

            current_cluster.push_back({x,y}); // Add point to current cluster
        }

        for (auto &p : table_positions)
        {
            geometry_msgs::msg::PointStamped p_base, p_odom;
            p_base.header.stamp = this->now();
            p_base.header.frame_id = "base_link";
            p_base.point.x = p.first;
            p_base.point.y = p.second;

            try {
                tf_buffer_->transform(p_base, p_odom, "odom", tf2::durationFromSec(0.05));
                table_positions_odom.push_back({p_odom.point.x, p_odom.point.y});
            }
            catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(),
                            "TF transform failed for table: %s", ex.what());
                table_positions_odom.push_back({p.first, p.second}); // fallback
            }
        }
        
        int found_tables = table_positions_odom.size(); // Number of detected tables

        // ### LOG [TURTLEBOT] ### detected tables
        RCLCPP_INFO(this->get_logger(),
        "[TURTLEBOT] Detected %d Tables", found_tables);

        for (size_t i = 0; i < table_positions_odom.size(); i++)
            {
                RCLCPP_INFO(this->get_logger(),
                            "  Table %ld: x=%.2f  y=%.2f",
                            i + 1,
                            table_positions_odom[i].first,
                            table_positions_odom[i].second);
            }
    
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtlebotServer>());
  rclcpp::shutdown();
  return 0;
}