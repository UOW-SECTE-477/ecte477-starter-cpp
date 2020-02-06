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
        BeaconNode()
        {
            this->subscriber = nh.subscribe<sensor_msgs::Image>("camera/color/image_raw/", 1, &BeaconNode::callback_colour_image, this);

            // Process beacons into vector
            ros::NodeHandle nh_params("~");
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
            ROS_INFO("callback_colour_image()");

            // Get image message as an OpenCV Mat object (most functions you use for image processing will want this)
            cv::Mat frame = cv_bridge::toCvShare(colour_image, sensor_msgs::image_encodings::BGR8)->image;

            cv::imshow("cv_img", frame);
            cv::waitKey(2);
        }

    private:
        ros::Subscriber subscriber;
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