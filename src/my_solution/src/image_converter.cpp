#include "image_converter.h"

//much thanks: https://gist.github.com/nightduck/a07c185faad82aeaacbfa87298d035c0
ImageConverter::ImageConverter() : Node("image_converter"){
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    pub_ = this->create_publisher<std_msgs::msg::Float32>("desired_angle", 10);
    curr_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "current_angle", 10, std::bind(&ImageConverter::angleCallback, this, _1));
    sub_ = image_transport::create_subscription(
            this, "robotcam",std::bind(&ImageConverter::imageCallback,
                                       this, _1), "raw", custom_qos);
    }

    //current angle callback which updates the current angle
    void ImageConverter::angleCallback(const std_msgs::msg::Float32::SharedPtr msg){
        curr_angle = *msg;
        //log curr_angle value
        //RCLCPP_INFO(this->get_logger(), "curr_angle: %f", curr_angle.data);
    }

    void ImageConverter::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        //use cv_bridge to convert ROS image to OpenCV image
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        //mask the image for the color red
        cv::Mat dstImage = cv::Mat(640, 640, CV_8UC3, cv::Scalar(255,255,255));
        inRange(cv_ptr->image, cv::Scalar(0, 0, 100), cv::Scalar(50, 50, 255), dstImage);

        //find the non-zero pixels and average them
        findNonZero(dstImage, nonZeros);
        avgX = 0.0;
        // Iterate over the non-zero points
        for (size_t i = 0; i < nonZeros.total(); i++) {
            avgX += nonZeros.at<cv::Point>(i).x;
        }
        // Calculate the average (x, y) coordinates
        if (nonZeros.total() > 0) {
            avgX = avgX / nonZeros.total();
        } else {
            // Handle the case where there are no red pixels
            avgX = -1.0f;
        }
        //log avgPoint coordinates
        //RCLCPP_INFO(this->get_logger(), "x: %f", avgX);
        //find ratio of image width to average x coordinate
        float ratio = (320.0 - avgX) / 320.0;
        //if absolute value of ratio is less than 0.185, keep current angle
        if(avgX == -1.0f){
            angle = 0.0f;
        }
        else if (abs(ratio) < 0.185) {
            angle = curr_angle.data;
        }
        else {
            angle = ((ratio/4) * 0.785) + curr_angle.data;
        }
        //RCLCPP_INFO(this->get_logger(), "new_angle: %f", angle);
        curr_angle.data = angle;

        // Create a message object for publishing with a fixed value
        auto output_msg = std_msgs::msg::Float32();
        output_msg.data = angle;
        pub_->publish(output_msg);
    }

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConverter>());
    rclcpp::shutdown();
    return 0;
}