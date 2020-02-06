/**
    command_server_node.cpp

    A ROS node that sends commands based on the current state of the 
    USAR problem. Explore, Return, Stop.

    Subscribed: 
    Publishes: cmd/

    Created: 2020/02/06
    Author: Brendan Halloran
**/

// Standard Library

// External Libraries
#include <ros/ros.h>
#include <std_msgs/String.h>

// Local Headers

namespace ecte477
{
    class CommandServerNode
    {
    public:
        CommandServerNode()
        {
            // Do something
        }

        void callback_state(std_msgs::String const& command)
        {
            ROS_INFO("callback_state()");
            // Do something
        }

    private:
        ros::Subscriber subscriber;
        ros::Publisher publisher;
        ros::NodeHandle nh;
    };
} // namespace ecte477

int main(int argc, char** argv)
{
    ROS_INFO("Starting ROS Command Server module");
    ros::init(argc, argv, "command_server_node");
    ecte477::CommandServerNode cs;
    ros::spin();
    ROS_INFO("Shutting down ROS Command Server module");

    return 0;
}