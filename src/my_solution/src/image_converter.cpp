#include <stdlib.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // include everything about OpenCV

//much thanks: https://gist.github.com/nightduck/a07c185faad82aeaacbfa87298d035c0
class ImageConverter : public rclcpp::Node {
public:
    ImageConverter() : Node("image_converter") {
        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        pub_ = image_transport::create_publisher(this, "modified_cam", custom_qos);
        sub_ = image_transport::create_subscription(this, "robotcam",
                                                    std::bind(&ImageConverter::imageCallback, this, std::placeholders::_1), "raw", custom_qos);
    }

private:
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat dstImage = cv::Mat(640, 640, CV_8UC3, cv::Scalar(255,255,255));
        inRange(cv_ptr->image, cv::Scalar(0, 0, 100), cv::Scalar(50, 50, 255), dstImage);
        cv::waitKey(30);
        sensor_msgs::msg::Image::SharedPtr outputMsg =
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", dstImage).toImageMsg();

        pub_.publish(*outputMsg.get());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConverter>());
    rclcpp::shutdown();
    return 0;
}