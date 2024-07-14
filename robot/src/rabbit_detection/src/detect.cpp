#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    try
    {
        cv_bridge::CvImagePtr bgr_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat hsv, mask;
        // cv::circle(bgr_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
        cv::cvtColor(bgr_ptr->image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 0, 200), cv::Scalar(5, 50, 255), mask);
        cv::imshow("raw", bgr_ptr->image);
        cv::imshow("mask", mask);
        cv::waitKey(10);
    }
    catch (const cv_bridge::Exception &e)
    {
        auto logger = rclcpp::get_logger("rabbit_detector");
        RCLCPP_ERROR(logger, "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("detector", options);
    // TransportHints does not actually declare the parameter
    node->declare_parameter<std::string>("image_transport", "compressed");
    cv::namedWindow("view");
    cv::startWindowThread();
    image_transport::ImageTransport it(node);
    image_transport::TransportHints hints(node.get());
    image_transport::Subscriber sub = it.subscribe("/image_raw", 1, imageCallback, &hints);
    rclcpp::spin(node);
    cv::destroyWindow("view");

    return 0;
}