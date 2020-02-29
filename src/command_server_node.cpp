/**
    command_server_node.cpp

    A ROS node that sends commands based on the current state of the 
    USAR problem. Explore, Return, Stop.

    Listens to the start and stop commands on the cmd/ topic. These can 
    be sent with:
    rostopic pub -1 /cmd std_msgs/String -- 'start'
    rostopic pub -1 /cmd std_msgs/String -- 'stop'

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
#include "usar/commands.hpp"

namespace ecte477
{
    class CommandServerNode
    {
    public:
        CommandServerNode()
        {
            this->state = RobotState::WAITING_TO_START;
            this->subscriber_command = nh.subscribe<std_msgs::String>("cmd", 1, &CommandServerNode::callback_command, this);
            this->publisher_state = nh.advertise<std_msgs::String>("state", 1);

            ros::Rate loop_rate(10);
            while (ros::ok())
            {
                this->loop();
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        void loop()
        {
            std_msgs::String msg;
            msg.data = this->state;
            this->publisher_state.publish(msg);
        }

        void callback_command(std_msgs::StringConstPtr const& command)
        {
            ROS_INFO("callback_command()");
            if (command->data == Commands::START)
            {
                this->state = RobotState::EXPLORE;
            }
            else if (command->data == Commands::STOP)
            {
                this->state = RobotState::PAUSED;
            }
        }

    private:
        ros::Subscriber subscriber_command;
        ros::Publisher publisher_state;
        ros::NodeHandle nh;
        std::string state;
    };
} // namespace ecte477

int main(int argc, char** argv)
{
    ROS_INFO("Starting ROS Command Server module");
    ros::init(argc, argv, "command_server_node");
    ecte477::CommandServerNode cs;
    // ros::spin();
    ROS_INFO("Shutting down ROS Command Server module");

    return 0;
}