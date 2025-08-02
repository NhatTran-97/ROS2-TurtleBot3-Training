#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <marker_msgs/msg/marker_point.hpp>
#include <marker_msgs/msg/marker_point_array.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class USBCameraArucoNode : public rclcpp::Node {
public:
    USBCameraArucoNode();

private:
    void process_frame();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    rclcpp::Publisher<marker_msgs::msg::MarkerPointArray>::SharedPtr point_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    cv::VideoCapture cap_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    float marker_length_;
    bool show_gui_;

    std::map<int, std::string> aruco_locations_ = {
        {1, "station 1"}, {2, "station 1"},
        {3, "station 2"}, {4, "station 3"},
        {5, "station 4"}, {6, "station 4"}
    };
};


USBCameraArucoNode::USBCameraArucoNode(): Node("usb_aruco_node"),
  tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
    this->declare_parameter("show_gui", true);
    this->get_parameter("show_gui", show_gui_);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("aruco_poses", 10);
    point_pub_ = this->create_publisher<marker_msgs::msg::MarkerPointArray>("aruco_ids_poses", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("Rviz_marker", 10);

    cap_.open(2,cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open USB camera");
        throw std::runtime_error("Camera not found");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Opened USB camera");
    }

    camera_matrix_ = (cv::Mat1d(3, 3) <<
        600, 0, 320,
        0, 600, 240,
        0, 0, 1);
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);

    dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    marker_length_ = 0.04;

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
        this->process_frame();
    });
}

void USBCameraArucoNode::process_frame() {
    
    // RCLCPP_ERROR(this->get_logger(), "Process Frame");
    cv::Mat image;
    cap_ >> image;
    if (image.empty()) return;

    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    cv::aruco::detectMarkers(image, dictionary_, corners, ids);

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = "base_link";  // transformed frame
    pose_array.header.stamp = this->now();

    marker_msgs::msg::MarkerPointArray point_array;
    point_array.header = pose_array.header;

    visualization_msgs::msg::MarkerArray marker_array;

    // geometry_msgs::msg::TransformStamped transform_stamped;
    // try {
    //     transform_stamped = tf_buffer_.lookupTransform("base_link", "camera_link", tf2::TimePointZero);
    // } catch (tf2::TransformException &ex) {
    //     RCLCPP_WARN_ONCE(this->get_logger(), "Transform not available: %s", ex.what());
    //     return;
    // }

    if (!ids.empty()) {
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, camera_matrix_, dist_coeffs_, rvecs, tvecs);
        cv::aruco::drawDetectedMarkers(image, corners, ids);

        for (size_t i = 0; i < ids.size(); ++i) {
            geometry_msgs::msg::Pose pose_cam;
            pose_cam.position.x = tvecs[i][0];
            pose_cam.position.y = tvecs[i][1];
            pose_cam.position.z = tvecs[i][2];

            // Convert rotation
            cv::Mat R;
            cv::Rodrigues(rvecs[i], R);
            tf2::Matrix3x3 tf3d(
                R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2)
            );
            tf2::Quaternion q;
            tf3d.getRotation(q);
            pose_cam.orientation.x = q.x();
            pose_cam.orientation.y = q.y();
            pose_cam.orientation.z = q.z();
            pose_cam.orientation.w = q.w();

            // // Transform pose to base_link
            geometry_msgs::msg::PoseStamped pose_in, pose_out;
            pose_in.header.frame_id = "camera_link";
            pose_in.header.stamp = this->now();
            pose_in.pose = pose_cam;
            // tf2::doTransform(pose_in, pose_out, transform_stamped);

            pose_array.poses.push_back(pose_out.pose);

            marker_msgs::msg::MarkerPoint mp;
            mp.id = ids[i];
            mp.location = aruco_locations_.count(ids[i]) ? aruco_locations_[ids[i]] : "unknown";
            mp.point = pose_out.pose.position;
            point_array.markers.push_back(mp);

            visualization_msgs::msg::Marker text_marker;
            text_marker.header = pose_array.header;
            text_marker.ns = "aruco_text";
            text_marker.id = ids[i];
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position = pose_out.pose.position;
            text_marker.pose.position.z += 0.05;
            text_marker.scale.z = 0.04;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;
            text_marker.text = "ID: " + std::to_string(ids[i]);
            text_marker.lifetime = rclcpp::Duration::from_seconds(1.0);
            marker_array.markers.push_back(text_marker);
        }
    }

    // pose_pub_->publish(pose_array);
    point_pub_->publish(point_array);
    marker_pub_->publish(marker_array);

    if (true) {
        // RCLCPP_ERROR(this->get_logger(), "window openned");
        cv::imshow("Aruco Detection", image);
        cv::waitKey(1);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<USBCameraArucoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
