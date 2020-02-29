/**
    beacon_node.cpp

    A ROS node that searching in colour images for any beacons and tries to 
    find its location relative to the robot.

    Subscribed: camera/colour/image_raw/compressed
    Publishes: 

    Created: 2020/02/06
    Author: Brendan Halloran
**/

// Standard Library
#include <string>
#include <vector>

// External Libraries
#include <XmlRpcException.h>
#include <XmlRpcValue.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

// Local Headers

namespace ecte477
{
    struct Beacon
    {
        Beacon(int id, std::string top, std::string bottom, bool found) : id(id), top(top), bottom(bottom), found(found) {}
        int id;
        std::string top;
        std::string bottom;
        bool found;
    };

    class BeaconNode
    {
    public:
        // Constants
        static constexpr double DEPTH_SCALE = 0.001;

        BeaconNode()
        {
            this->subscriber_colour_image = nh.subscribe<sensor_msgs::Image>("camera/color/image_raw/", 1, &BeaconNode::callback_colour_image, this);
            this->subscriber_depth = nh.subscribe<sensor_msgs::Image>("camera/aligned_depth_to_color/image_raw/", 1, &BeaconNode::callback_depth, this);

            // Process beacons into vector
            ros::NodeHandle nh_params("~"); // ~ is used to get params
            try
            {
                XmlRpc::XmlRpcValue beacons_parameter;
                nh_params.getParam("beacons", beacons_parameter);
                for (int i = 0; i < beacons_parameter.size(); i++)
                {
                    auto& beacon_dict = beacons_parameter[i];
                    this->beacons.emplace_back(static_cast<int>(beacon_dict["id"]),
                                               static_cast<std::string>(beacon_dict["top"]),
                                               static_cast<std::string>(beacon_dict["bottom"]),
                                               false);
                }
            }
            catch (XmlRpc::XmlRpcException& e)
            {
                ROS_ERROR("Unable to parse beacon parameter. (%s)", e.getMessage().c_str());
                throw e;
            }
        }

        void callback_colour_image(sensor_msgs::ImageConstPtr const& colour_image)
        {
            ROS_INFO("callback_colour_image()"); // For testing, can be removed

            // Get image message as an OpenCV Mat object (most functions you use for image processing will want this)
            cv::Mat frame = cv_bridge::toCvShare(colour_image, sensor_msgs::image_encodings::BGR8)->image;

            cv::imshow("cv_img", frame);
            cv::waitKey(2);
        }

        void callback_depth(sensor_msgs::ImageConstPtr const& depth_image)
        {
            ROS_INFO("callback_depth()");

            // Get image message as an OpenCV Mat object (most functions you use for image processing will want this)
            cv::Mat frame = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1)->image; // Note encoding

            // Find the depth of the centre pixel in metres
            cv::Point2i centre_point(frame.cols / 2, frame.rows / 2);
            double centre_depth = DEPTH_SCALE * static_cast<double>(frame.at<uint16_t>(centre_point));
            ROS_INFO("centre depth: %.4f", centre_depth);

            cv::imshow("cv_depth", frame);
            cv::waitKey(2);
        }

    private:
        ros::Subscriber subscriber_colour_image, subscriber_depth;
        ros::NodeHandle nh;
        std::vector<Beacon> beacons;
    };
} // namespace ecte477

int main(int argc, char** argv)
{
    ROS_INFO("Starting ROS Beacon Detector module");
    ros::init(argc, argv, "beacon_node");
    ecte477::BeaconNode bd;
    ros::spin();
    ROS_INFO("Shutting down ROS Beacon Detector module");

    return 0;
}