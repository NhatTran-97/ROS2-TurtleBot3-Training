#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "rosi_msgs/srv/get_object_location.hpp"
#include <cmath>
#include <chrono>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class TurtleBotAruco : public rclcpp::Node
{
public:
    TurtleBotAruco() : Node("final_assessment")
    {
        distance_threshold_ = 1.0;
        max_linear_velocity_ = 0.1;
        max_angular_velocity_ = 1.0;
        too_close_distance_ = 0.15;
        alignment_angle_threshold_ = M_PI / 9.0; // 20 degrees

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        twist_msg_ = geometry_msgs::msg::Twist();

        service_client_ = this->create_client<rosi_msgs::srv::GetObjectLocation>("/find_objects");

        while (!service_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        }

        timer_ = this->create_wall_timer(100ms, std::bind(&TurtleBotAruco::send_request, this));
    }

private:
    void send_request()
    {
        auto request = std::make_shared<rosi_msgs::srv::GetObjectLocation::Request>();
        auto future = service_client_->async_send_request(request,
                    std::bind(&TurtleBotAruco::response_callback, this, std::placeholders::_1));
    }

    void response_callback(rclcpp::Client<rosi_msgs::srv::GetObjectLocation>::SharedFuture future)
    {
        try
        {
            auto result = future.get();
            auto pose = result->object_pose;

            RCLCPP_INFO(this->get_logger(), "Object Pose - x: %.2f, y: %.2f, z: %.2f",
                        pose.position.x, pose.position.y, pose.position.z);

            check_tag(result, pose);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }

    void check_tag(const rosi_msgs::srv::GetObjectLocation::Response::SharedPtr &result,
                   const geometry_msgs::msg::Pose &pose)
    {
        if (result->result)
        {
            RCLCPP_INFO(this->get_logger(), "robot can run");

            double current_distance = get_tag_distance(pose.position.x, pose.position.y);
            RCLCPP_INFO(this->get_logger(), "current_distance: %.2f", current_distance);

            double angle_to_tag = get_tag_angle(pose.position.x, pose.position.y);
            RCLCPP_INFO(this->get_logger(), "angle_to_tag (degree): %.2f", angle_to_tag * 180 / M_PI);

            if (std::abs(angle_to_tag) > alignment_angle_threshold_)
            {
                double angular_vel = angle_to_tag;
                if (angular_vel > max_angular_velocity_)
                    angular_vel = max_angular_velocity_;
                move_turtlebot(0.0, angular_vel);
                cmd_vel_pub_->publish(twist_msg_);
                return;
            }

            if (current_distance >= distance_threshold_)
            {
                RCLCPP_INFO(this->get_logger(), "\u2705 Reached target \u2192 stop");
                move_turtlebot(0.0, 0.0);
            }
            else if (current_distance <= too_close_distance_)
            {
                move_turtlebot(-max_linear_velocity_, 0.0);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "\u27A1\uFE0F Moving toward ArUco");
                move_turtlebot(max_linear_velocity_, 0.0);
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "robot can not run");
            move_turtlebot(0.0, 0.0);
        }

        cmd_vel_pub_->publish(twist_msg_);
        
    }

    double get_tag_distance(double x, double y)
    {
        return std::sqrt(x * x + y * y);
    }

    double get_tag_angle(double x, double y)
    {
        return std::atan2(y, x);
    }

    void move_turtlebot(double linear_velocity, double angular_velocity)
    {
        twist_msg_.linear.x = linear_velocity;
        twist_msg_.angular.z = angular_velocity;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    geometry_msgs::msg::Twist twist_msg_;
    rclcpp::Client<rosi_msgs::srv::GetObjectLocation>::SharedPtr service_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    double distance_threshold_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double too_close_distance_;
    double alignment_angle_threshold_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleBotAruco>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
