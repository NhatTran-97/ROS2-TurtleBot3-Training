#include "rclcpp/rclcpp.hpp"
#include "chrono"
#include "geometry_msgs/msg/twist.hpp"
#include "simple_velocity_msg/msg/simple_velocity.hpp"  
using std::placeholders::_1;

class CustomControlRobot : public rclcpp::Node
{
    
public:
    CustomControlRobot() : Node("custom_control_robot")
    {
        pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        sub_velocity_ = this->create_subscription<simple_velocity_msg::msg::SimpleVelocity>("/simple_velocity", 10, std::bind(&CustomControlRobot::velocity_callback, this, _1));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Subscription<simple_velocity_msg::msg::SimpleVelocity>::SharedPtr sub_velocity_;
    geometry_msgs::msg::Twist twist_msg;  

    void velocity_callback(const simple_velocity_msg::msg::SimpleVelocity::SharedPtr msg)
    {
        twist_msg.linear.x = msg->linear_velocity;  
        twist_msg.angular.z = msg->angular_velocity; 
        pub_cmd_->publish(twist_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CustomControlRobot>());
    rclcpp::shutdown();
    return 0;
}
