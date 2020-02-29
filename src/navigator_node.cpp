/**
    navigator_node.cpp

    A ROS node that sends a PoseStamped goal to return home once the robot enters the RETURING state

    Subscribed: 
    Publishes: 

    Created: 2020/02/07
    Author: Brendan Halloran
**/

// Standard Library

// External Libraries
#include <ros/ros.h>
#include <std_msgs/String.h>

// Local Headers

namespace ecte477
{
    class NavigatorNode
    {
    public:
        NavigatorNode() : navigating(false)
        {
        }

        void callback_state(std_msgs::String const& state_msg)
        {
            ROS_INFO("callback_state()");
            // Do something
        }

    private:
        ros::NodeHandle nh;
        bool navigating;
    };
} // namespace ecte477

int main(int argc, char** argv)
{
    ROS_INFO("Starting ROS Navigator module");
    ros::init(argc, argv, "navigator_node");
    ecte477::NavigatorNode nv;
    ros::spin();
    ROS_INFO("Shutting down ROS Navigator module");

    return 0;
}