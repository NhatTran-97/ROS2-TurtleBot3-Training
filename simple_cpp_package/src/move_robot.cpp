#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "chrono"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rmw/qos_profiles.h"
#include "rclcpp/qos.hpp"
#include <queue>
using std::placeholders::_1;

class MovingRobot : public rclcpp::Node
{
public:
    MovingRobot() : Node("publisher_node")
    {
        rclcpp::QoS qos_cmd(rclcpp::KeepLast(1));
        qos_cmd.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_cmd.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MovingRobot::timer_callback, this));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&MovingRobot::odom_callback, this, _1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&MovingRobot::scan_callback, this, _1));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    geometry_msgs::msg::Twist twist_msg;  
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    void timer_callback()
    {
        twist_msg.linear.x = 0.0;  
        twist_msg.angular.z = 0.0; 

        pub_cmd_->publish(twist_msg);

        // Log a message for debugging
        //RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%.2f, angular.z=%.2f", twist_msg.linear.x, twist_msg.angular.z);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
       RCLCPP_INFO(this->get_logger(), "Odom sub: '%f'",msg->pose.pose.position.x);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Laser_Scan: '%f'", msg->ranges[20]);
    }
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovingRobot>());
    rclcpp::shutdown();
    return 0;
}
