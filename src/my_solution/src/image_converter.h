//
// Created by bvox0102 on 11/28/23.
//

#ifndef TR_CV_TRAINING_2_LI_KANE_IMAGE_CONVERTER_H
#define TR_CV_TRAINING_2_LI_KANE_IMAGE_CONVERTER_H

#include <stdlib.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cv_bridge/cv_bridge.hpp> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // include everything about OpenCV

using std::placeholders::_1;

class ImageConverter : public rclcpp::Node {
public:
    ImageConverter();
private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);
    void angleCallback(const std_msgs::msg::Float32::SharedPtr msg);
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curr_angle_sub_;

    float curr_angle;
    image_transport::Subscriber sub_;
};

#endif //TR_CV_TRAINING_2_LI_KANE_IMAGE_CONVERTER_H
