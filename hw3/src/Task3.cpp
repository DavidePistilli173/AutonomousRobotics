#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>

#include "Task3.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Task3::Task3()
{
    _corridorEnd.target_pose.header.frame_id = "marrtino_map";
    _corridorEnd.target_pose.pose.position.x = -1.30;
    _corridorEnd.target_pose.pose.position.y = 2.3825;
    _corridorEnd.target_pose.pose.orientation.w = 0.72356;
};

bool Task3::init(int argc, char** argv)
{
    ros::init(argc, argv, NODE_NAME);
    return true;
}

void Task3::run()
{
    ros::NodeHandle n;
    _corridorEnd.target_pose.header.stamp = ros::Time::now();

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))) 
        ROS_INFO("Waiting for the move_base action server"); 


    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "marrtino_map";
    goal.target_pose.pose.position.x = -0.40;
    goal.target_pose.pose.position.y = 1.5;
    goal.target_pose.pose.orientation.w = 0.71;

    ROS_INFO("Sending goal");
    ac.sendGoal(_corridorEnd);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("You have arrived to the goal position");
    else
        ROS_INFO("The base failed for some reason");
}