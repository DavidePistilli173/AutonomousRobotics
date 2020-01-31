#include <ros/ros.h>

#include "Task.hpp"

std::string NODE_NAME = "hw2_task";
std::string PLANNING_GROUP = "manipulator";

int main(int argc, char** argv)
{
	/*
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    ROS_INFO("SACIASBISAUBISABFIASBSAFBSAIFUSAHISHYU");
    */
	
    Task task;
    if (!task.init(argc, argv))
    {
        ROS_ERROR("Unable to intialise program.");
        return -1;
    }
    task.run();
    return 0;
}
