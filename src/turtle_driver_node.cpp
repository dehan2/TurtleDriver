#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "ConstForTurtleDriver.h"
#include "TurtleDriver.h"
#include <functional>
#include <iostream>

using namespace std;

void publish_velocity_command(const ros::Publisher& pub, const VelocityCommand& command)
{
    geometry_msgs::Twist velocity;
    velocity.linear.x = command.vx;
    velocity.linear.y = command.vy;
    velocity.angular.z = command.az;
    pub.publish(velocity);
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

    bool isOnMoving = false;
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    turtleDriver.move_to_next_order(true);

    ros::Rate loop_rate(10);

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


        //cout<<"Pos: "<<turtleDriver.get_pose().x <<", "<<turtleDriver.get_pose().y <<", "<<turtleDriver.get_pose().headingAngle <<endl;

        /*if(isOnMoving == true)
        {
            if(turtleDriver.is_goal_reached() == true)
            {
                cout<<"Goal reached." <<endl;

                // Stop the turtle
                publish_velocity_command(pub, VelocityCommand());
                isOnMoving = false;

                if(turtleDriver.move_to_next_order() == false)
                {
                    // If there is no next order, stop the node
                    ROS_INFO("All orders are finished.");
                    break;
                }
            }
        }
        else
        {
            VelocityCommand command = turtleDriver.generate_velocity_command_to_achieve_goal(*turtleDriver.get_current_order());
            cout<<"Velocity command: "<<command.vx <<", "<<command.vy <<", "<<command.az <<endl;

            publish_velocity_command(pub, command);
            isOnMoving = true;
        }*/
    }

    return 0;
}