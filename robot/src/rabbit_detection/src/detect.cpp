#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

const uint16_t SIZE = 500;

void topViewTransform(
    cv::Mat &image_original,
    cv::Mat &image_transformed,
    cv::Point2f &upper_left,
    cv::Point2f &upper_right,
    cv::Point2f &lower_right,
    cv::Point2f &lower_left,
    cv::Point2f &upper_left_target,
    cv::Point2f &upper_right_target,
    cv::Point2f &lower_right_target,
    cv::Point2f &lower_left_target)
{
    cv::Point2f source_points[4], dest_points[4];

    source_points[0] = upper_left;
    source_points[1] = upper_right;
    source_points[2] = lower_right;
    source_points[3] = lower_left;

    dest_points[0] = upper_left_target;
    dest_points[1] = upper_right_target;
    dest_points[2] = lower_right_target;
    dest_points[3] = lower_left_target;
    // calculate transformation matrix
    cv::Mat transform_matrix = cv::getPerspectiveTransform(source_points, dest_points);
    cv::warpPerspective(image_original, image_transformed, transform_matrix, cv::Size(SIZE, SIZE));
}

void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    try
    {
        cv::Point2f upper_left(96, 114);
        cv::Point2f upper_right(688, 22);
        cv::Point2f lower_right(1210, 274);
        cv::Point2f lower_left(366, 682);

        // target coordinates
        cv::Point2f upper_left_target(0, 0);
        cv::Point2f upper_right_target(SIZE, 0);
        cv::Point2f lower_right_target(SIZE, SIZE);
        cv::Point2f lower_left_target(0, SIZE);
        // convert ROS image to CV image
        cv_bridge::CvImagePtr bgr_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        // apply transformation
        cv::Mat img;
        topViewTransform(
            bgr_ptr->image, // original image
            img,            // transformed image
            upper_left,
            upper_right,
            lower_right,
            lower_left,
            upper_left_target,
            upper_right_target,
            lower_right_target,
            lower_left_target);

        cv::Mat hsv, mask;
        // cv::circle(bgr_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(0, 10, 200), cv::Scalar(30, 100, 255), mask);
        cv::imshow("raw", img);
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
