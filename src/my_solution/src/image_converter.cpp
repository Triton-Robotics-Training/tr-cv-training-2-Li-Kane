#include "image_converter.h"

//much thanks: https://gist.github.com/nightduck/a07c185faad82aeaacbfa87298d035c0
ImageConverter::ImageConverter() : Node("image_converter"){
    // this window is just to show openCV changes
    cv::namedWindow("Image window");
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
    curr_angle = msg->data;
    RCLCPP_INFO(this->get_logger(), "Current angle: '%f'", curr_angle);
}

// image callback which processes the image and publishes the desired angle
void ImageConverter::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    //use cv_bridge to convert ROS image to OpenCV image
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    // draw a circle and display
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(150, 150), 10, CV_RGB(0,255,0));
    cv::imshow("Image window", cv_ptr->image);
    cv::waitKey(3);
    // Create a message object for publishing with a fixed value
    float angle = 0.0;
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