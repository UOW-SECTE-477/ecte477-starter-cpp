/**
    wall_follower_node.cpp

    A ROS node that commands a Turtlebot3 to follow a wall to its right
    according to the latest laser scan.

    Subscribed: scan/, cmd/
    Publishes: cmd_vel/

    Created: 2020/02/06
    Author: Brendan Halloran
**/

// Standard Library
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

// External Libraries
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

// Local Headers
#include "usar/commands.hpp"

namespace ecte477
{
    enum class FollowSide
    {
        LEFT = 1,
        RIGHT = 2
    };

    class WallFollowerNode
    {
    public:
        // Constants
        static constexpr char BASE_FRAME[] = "base_link"; // Param from the SLAM module
        static constexpr double MAX_SIDE_LIMIT = 0.50;    // This furthest distance we 'see' the wall to the side
        static constexpr double MIN_APPROACH_DIST = 0.30; // The closest we want to get to a wall from the front
        static constexpr double MAX_APPROACH_DIST = 0.50; // The distance we want to start slowing to approach a front wall
        static constexpr double ROBOT_RADIUS = 0.20;      // The bounding circle around the robot
        static constexpr double MAX_TRANS_SPEED = 0.25;   // Forward movement
        static constexpr double MAX_TURN_SPEED = 1.4;     // Rotation

        // Constructor
        WallFollowerNode()
            : explore(false),
              stopped(false),
              follow_side(FollowSide::LEFT)
        {
            this->subscriber_laser_scan = nh.subscribe<sensor_msgs::LaserScan>("scan", 1, &WallFollowerNode::callback_laser_scan, this);
            this->subscriber_command = nh.subscribe<std_msgs::String>("cmd", 1, &WallFollowerNode::callback_command, this);
            this->publisher_twist = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);
        }

        void callback_laser_scan(sensor_msgs::LaserScanConstPtr const& laser_scan)
        {
            ROS_INFO("callback_laser_scan()");

            // If we are no longer exloring don't process the scan
            if (!this->explore)
            {
                // If we aren't exploring, but we haven't told the robot to stop, do so now
                if (!this->stopped)
                {
                    this->stopped = true;
                    geometry_msgs::Twist vel_msg;
                    vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z = 0.0;
                    vel_msg.angular.x = vel_msg.angular.y = vel_msg.angular.z = 0.0;
                    this->publisher_twist.publish(vel_msg);
                }
                return;
            }
            // Otherwise, process the scan.

            // Find the transform between the laser scanner and the robots base frame
            tf::StampedTransform transformation;
            try
            {
                tf_listener.waitForTransform(BASE_FRAME, laser_scan->header.frame_id, laser_scan->header.stamp, ros::Duration(2.0));
                tf_listener.lookupTransform(BASE_FRAME, laser_scan->header.frame_id, laser_scan->header.stamp, transformation);
            }
            catch (tf::TransformException& e)
            {
                ROS_ERROR("Unable to get transformation: %s", e.what());
                return;
            }

            // We want to find how far in front of the robot the wall to the left extends
            // and how far away the nearest point in front of the robot is.
            double x_side_max = std::numeric_limits<double>::min();
            double x_front_min = std::numeric_limits<double>::max();
            double angle = laser_scan->angle_min;

            // Transform scans into 3D points relative to robots base frame
            for (auto const& scan : laser_scan->ranges)
            {
                // 3D homogeneous point [x; y; z; 1]
                tf::Vector3 point(cos(angle) * scan, sin(angle) * scan, 0);
                point = transformation * point;

                // Update angle
                angle += laser_scan->angle_increment;

                // Find if point is forward-most wall point on correct side
                if (std::abs(point.x()) <= ROBOT_RADIUS && std::abs(point.y()) <= MAX_SIDE_LIMIT)
                {
                    if ((this->follow_side == FollowSide::LEFT && point.y() > 0) || (this->follow_side == FollowSide::RIGHT && point.y() < 0))
                    {
                        // Point is between the front and back of robot and within range on correct side
                        if (point.x() > x_side_max)
                        {
                            x_side_max = point.x();
                        }
                    }
                }

                // Find if point is nearest wall point in front of the robot
                if (point.x() > 0 && point.x() <= MAX_APPROACH_DIST && std::abs(point.y()) <= ROBOT_RADIUS)
                {
                    // Point is between left and right sides of robot and within range from the front
                    if (point.x() < x_front_min)
                    {
                        x_front_min = point.x();
                    }
                }
            }
            ROS_INFO("Detected walls %.2f left, %.2f front\n", x_side_max, x_front_min);

            double turn = 0.0, drive = 0.0;

            if (x_side_max == std::numeric_limits<double>::min())
            {
                // No wall to side, so turn that way
                turn = 1.0;
                drive = 0.0;
            }
            else if (x_front_min <= MIN_APPROACH_DIST)
            {
                // Wall to side and blocked at front, so turn opposite way
                turn = -1.0;
                drive = 0.0;
            }
            else
            {
                // Wall to side but front is clear, so drive forward and start turning if
                // an opening is starting to appear to side

                // If side wall doesn't extend past front of robot start turning and reduce speed
                double turn_side = std::max(std::min((ROBOT_RADIUS - x_side_max) / (2.0 * ROBOT_RADIUS), 1.0), 0.0);
                double drive_side = std::max(std::min((ROBOT_RADIUS + x_side_max) / (2.0 * ROBOT_RADIUS), 1.0), 0.0);

                // If front wall is between MAX_APPROACH_DIST and MIN_APPROACH_DIST then slow down and reduce turn
                double turn_front = std::max(std::min((MAX_APPROACH_DIST - x_front_min) / (MAX_APPROACH_DIST - MIN_APPROACH_DIST), 1.0), 0.0);
                double drive_front = std::max(std::min((x_front_min - MIN_APPROACH_DIST) / (MAX_APPROACH_DIST - MIN_APPROACH_DIST), 1.0), 0.0);

                // Combine two turn and drive decisions
                turn = turn_side - 0.5 * turn_front;
                drive = drive_side * drive_front;
            }

            // Flip turn if we are following to the right
            if (this->follow_side == FollowSide::RIGHT)
            {
                turn *= -1.0;
            }

            // Convert to a Twist message for 'cmd_vel/'
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = drive * MAX_TRANS_SPEED;
            vel_msg.angular.z = turn * MAX_TURN_SPEED;
            vel_msg.linear.y = vel_msg.linear.z = 0.0;
            vel_msg.angular.x = vel_msg.angular.y = 0.0;
            ROS_INFO("Publishing velocities %.2f m/s, %.2f r/s\n", vel_msg.linear.x, vel_msg.angular.z);
            this->publisher_twist.publish(vel_msg);
            this->stopped = false;
        }

        void callback_command(std_msgs::StringConstPtr const& command_msg)
        {
            ROS_INFO("callback_command()");
            Commands command = commands[command_msg->data];
            if (command == Commands::EXPLORE)
            {
                this->explore = true;
                ROS_INFO("Starting Exploring");
            }
            else if (command == Commands::RETURN || command == Commands::HALT)
            {
                this->explore = false;
                this->stopped = false;
                ROS_INFO("Stopping Exploring");
            }
        }

        void shutdown()
        {
            this->stopped = true;
            this->explore = false;
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z = 0.0;
            vel_msg.angular.x = vel_msg.angular.y = vel_msg.angular.z = 0.0;
            this->publisher_twist.publish(vel_msg);
        }

    private:
        ros::Subscriber subscriber_laser_scan, subscriber_command;
        ros::Publisher publisher_twist;
        ros::NodeHandle nh;
        tf::TransformListener tf_listener;

        bool stopped, explore;
        FollowSide follow_side;
    };

    constexpr decltype(WallFollowerNode::BASE_FRAME) WallFollowerNode::BASE_FRAME; // Required to make linker happy for cxx_std < 17
} // namespace ecte477

int main(int argc, char** argv)
{
    ROS_INFO("Starting ROS Wall Following module");
    ros::init(argc, argv, "wall_follower_node");
    ecte477::WallFollowerNode wf;
    ros::spin();
    ROS_INFO("Shutting down ROS Wall Following module");
    wf.shutdown();
    return 0;
}