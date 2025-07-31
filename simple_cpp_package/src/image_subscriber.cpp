#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "rclcpp/executors.hpp"
#include <unistd.h>
#include <memory>
#include <cstdio>
#include <string>

using namespace std;

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber() : Node("image_publisher_node")
    {
        // Khởi tạo bridge_
       // bridge_ = std::make_shared<cv_bridge::CvBridge>(); 

       callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

       rclcpp::SubscriptionOptions options;        options.callback_group = callback_group_;
        

       img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 10, std::bind(&ImageSubscriber::image_callback, this, std::placeholders::_1), options);


    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_subscription_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            // cv::Mat frame = bridge_->imgmsg_to_cv2(*msg, "bgr8");
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Hiển thị hình ảnh sử dụng OpenCV
            cv::imshow("frame", frame);
            cv::waitKey(1);  // Cập nhật cửa sổ OpenCV
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in converting image: %s", e.what());
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto image_subscriber = std::make_shared<ImageSubscriber>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(image_subscriber); 
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
