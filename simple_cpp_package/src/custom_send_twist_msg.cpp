#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "simple_velocity_msg/msg/simple_velocity.hpp"  
#include "chrono"
#include <queue>
using std::placeholders::_1;

class CustomSendTwistMsg : public rclcpp::Node
{
public:
    CustomSendTwistMsg() : Node("custom_send_twist_msg")
    {
        pub_cmd_ = this->create_publisher<simple_velocity_msg::msg::SimpleVelocity>("/simple_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CustomSendTwistMsg::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<simple_velocity_msg::msg::SimpleVelocity>::SharedPtr pub_cmd_;
    simple_velocity_msg::msg::SimpleVelocity simple_velocity;  
  

    void timer_callback()
    {
        simple_velocity.linear_velocity = 1.2;  
        simple_velocity.angular_velocity = 0.0; 
        pub_cmd_->publish(simple_velocity);

    }  
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CustomSendTwistMsg>());
    rclcpp::shutdown();
    return 0;
}
