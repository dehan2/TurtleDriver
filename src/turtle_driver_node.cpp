#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "ConstForTurtleDriver.h"
#include "TurtleDriver.h"
#include <functional>
#include <iostream>

using namespace std;

bool isStart = false;

void publish_velocity_command(const ros::Publisher& pub, const VelocityCommand& command)
{
    geometry_msgs::Twist velocity;
    velocity.linear.x = command.vx;
    velocity.linear.y = command.vy;
    velocity.angular.z = command.az;
    pub.publish(velocity);
}



void start_callback(const std_msgs::Bool::ConstPtr& msg)
{
    cout<<"Start order received: "<<msg->data <<endl;
    isStart = msg->data;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_driver_node");
    ros::NodeHandle n;

    if(argc != 3)
    {
        ROS_INFO("Usage: turtle_driver_node <driving_type> <order_file>");
        return 1;
    }

    cout <<"driving_type: " <<argv[1] <<", order file: "<<argv[2] << endl;

    TurtleDriver turtleDriver;
    turtleDriver.read_order_file(argv[2]);
    
    cout<<"order read finished: "<<turtleDriver.get_orders().size() <<endl;
    for(auto& order : turtleDriver.get_orders())
    {
        cout<<"order: ["<<order.goalx <<", "<<order.goaly <<", "<<order.goaltheta<<", "<<order.speed <<"]"<<endl;
    }


    ros::Subscriber sub = n.subscribe("/turtle1/pose", 10, &TurtleDriver::pose_callback, &turtleDriver);
    ros::Subscriber sub2 = n.subscribe("/driver_start", 10, &start_callback);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    ros::Rate loop_rate(50);
    
    while(ros::ok())
    {
        ros::spinOnce();
        if(turtleDriver.is_pose_initialized() == true && isStart == true)
        {
            cout<<"Pose initialized: " << turtleDriver.get_pose().x <<", "<<turtleDriver.get_pose().y <<", "<<turtleDriver.get_pose().headingAngle <<endl;
            break;
        }
    }

    turtleDriver.move_to_next_order(true);

    while(ros::ok())
    {
        ros::spinOnce();

        VelocityCommand command = turtleDriver.generate_velocity_command_to_achieve_goal(*turtleDriver.get_current_order());
        //cout<<"Velocity command: "<<command.vx <<", "<<command.vy <<", "<<command.az <<endl;
        publish_velocity_command(pub, command);

        if(turtleDriver.is_goal_reached() == true)
        {
            cout<<"Goal reached: " << turtleDriver.get_pose().x <<", "<<turtleDriver.get_pose().y <<", "<<turtleDriver.get_pose().headingAngle <<endl;

            if(turtleDriver.move_to_next_order() == true)
            {
                // If there is a next order, move to the next order
                const TurtleOrder& nextOrder = *turtleDriver.get_current_order();
                cout<<"Move to next order: " << nextOrder.goalx <<", "<<nextOrder.goaly<<", "<<nextOrder.goaltheta<<", "<<nextOrder.speed <<endl;

            }
            else
            {
                // If there is no next order, stop the node
                ROS_INFO("All orders are finished.");
                break;
            }
        }

        loop_rate.sleep();
    }

    return 0;
}