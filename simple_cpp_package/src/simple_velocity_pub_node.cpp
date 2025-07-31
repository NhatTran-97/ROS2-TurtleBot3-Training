#include "rclcpp/rclcpp.hpp"
#include "rosi_msgs/msg/simple_velocity.hpp"
#include "geometry_msgs/msg/twist.hpp"

class SimpleVelocity : public rclcpp::Node
{
public:
    SimpleVelocity() : Node("simple_velocity_pub_node")
    {
        pub_velocity_ = this->create_publisher<rosi_msgs::msg::SimpleVelocity>("/simple_vel", 10);
        sub_vel_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&SimpleVelocity::listener_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&SimpleVelocity::timer_callback, this));
        twist_msg_ = geometry_msgs::msg::Twist();
    }

private:
    rclcpp::Publisher<rosi_msgs::msg::SimpleVelocity>::SharedPtr pub_velocity_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist twist_msg_;

    void listener_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        twist_msg_ = *msg;
    }

    void timer_callback()
    {
        auto msg = rosi_msgs::msg::SimpleVelocity();
        msg.linear_velocity = twist_msg_.linear.x;
        msg.angular_velocity = twist_msg_.angular.z;

        pub_velocity_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Publishing linear velocity: '%f' m/s, angular velocity: '%f' rad/s",
                    msg.linear_velocity, msg.angular_velocity);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto simple_velocity = std::make_shared<SimpleVelocity>();
    rclcpp::spin(simple_velocity);
    rclcpp::shutdown();
    return 0;
}



