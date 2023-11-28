#include <stdlib.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // include everything about OpenCV

//much thanks: https://gist.github.com/nightduck/a07c185faad82aeaacbfa87298d035c0
class ImageConverter : public rclcpp::Node {
public:
    ImageConverter() : Node("image_converter") {
        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        pub_ = this->create_publisher<std_msgs::msg::Float32>("/desired_angle", 10);
        sub_ = image_transport::create_subscription(
                this, "robotcam",std::bind(&ImageConverter::imageCallback,
                                           this, std::placeholders::_1), "raw", custom_qos);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    image_transport::Subscriber sub_;
    cv::Mat nonZeros;
    cv::Mat avg;

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            //use cv_bridge to convert ROS image to OpenCV image
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        //mask the image for the color red
        cv::Mat dstImage = cv::Mat(640, 640, CV_8UC3, cv::Scalar(255,255,255));
        inRange(cv_ptr->image, cv::Scalar(0, 0, 100), cv::Scalar(50, 50, 255), dstImage);
        //find the non-zero pixels and average them
        findNonZero(dstImage, nonZeros);
        double sumX = 0.0;
        double sumY = 0.0;
        // Iterate over the non-zero points
        for (size_t i = 0; i < nonZeros.total(); i++) {
            sumX += nonZeros.at<cv::Point>(i).x;
            sumY += nonZeros.at<cv::Point>(i).y;
        }
        // Calculate the average (x, y) coordinates
        cv::Point2f avgPoint;
        if (nonZeros.total() > 0) {
            avgPoint.x = static_cast<float>(sumX / nonZeros.total());
            avgPoint.y = static_cast<float>(sumY / nonZeros.total());
        } else {
            // Handle the case where there are no red pixels
            avgPoint.x = 0.0f;
            avgPoint.y = 0.0f;
        }
        //log avgPoint coordinates
        RCLCPP_INFO(this->get_logger(), "x: %f y: %f", avgPoint.x, avgPoint.y);

        //draw a circle at the average location
        //cv::circle(dstImage, avgPoint, 10, CV_RGB(255, 0, 0));
        //show the image
        //cv::imshow("View", dstImage);
        cv::waitKey(30);
        //find ratio of image width to average x coordinate
        double ratio = (320.0 - avgPoint.x) / 320;
        //convert ratio to angle
        double angle = ratio * 3.14159;
        //convert angle to float32 to publish
        float angle32 = static_cast<float>(angle);
        RCLCPP_INFO(this->get_logger(), "angle: %f", angle32);

        // Create a message object for publishing with a fixed value
        auto output_msg = std_msgs::msg::Float32();
        output_msg.data = angle32;  // Set the desired angle to 0.0
        pub_->publish(output_msg);
        /*         *
         * pub_ = image_transport::create_publisher(this, "modified_cam", custom_qos);
         * sensor_msgs::msg::Image::SharedPtr outputMsg =
                cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", dstImage).toImageMsg();

        pub_.publish(*outputMsg.get()); */
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConverter>());
    rclcpp::shutdown();
    return 0;
}