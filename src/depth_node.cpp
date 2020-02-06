/**
    depth_node.cpp

    A ROS node that keeps track of the latest depth map and provides a service to
    query the depth of a given pixel

    Subscribed: camera/aligned_depth_to_color/image_raw/
    Publishes: 

    Created: 2020/02/06
    Author: Brendan Halloran
**/

// Standard Library

// External Libraries
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

// Local Headers

namespace ecte477
{
    class DepthNode
    {
    public:
        // Constants
        static constexpr double DEPTH_SCALE = 0.001;

        // Constructor
        DepthNode()
        {
            this->subscriber = nh.subscribe<sensor_msgs::Image>("camera/aligned_depth_to_color/image_raw/", 1, &DepthNode::callback_depth, this);
        }

        void callback_depth(sensor_msgs::ImageConstPtr const& depth_image)
        {
            ROS_INFO("callback_depth()");

            // Get image message as an OpenCV Mat object (most functions you use for image processing will want this)
            cv::Mat frame = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image; // Note encoding

            cv::Point2i centre_point(frame.cols / 2, frame.rows / 2);
            double centre_depth = DEPTH_SCALE * static_cast<double>(frame.at<uint16_t>(centre_point));
            ROS_INFO("centre depth: %.4f", centre_depth);

            cv::imshow("cv_depth", frame);
            cv::waitKey(2);
        }

    private:
        ros::Subscriber subscriber;
        ros::NodeHandle nh;
    };
} // namespace ecte477

int main(int argc, char** argv)
{
    ROS_INFO("Starting ROS Depth module");
    ros::init(argc, argv, "depth_node");
    ecte477::DepthNode dp;
    ros::spin();
    ROS_INFO("Shutting down ROS Depth module");

    return 0;
}