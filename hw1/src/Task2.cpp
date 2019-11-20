#include "Task2.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

Task2::Task2(){};

bool Task2::init(int argc, char** argv)
{
    /* Node init. */
    ros::init(argc, argv, NODE_NAME);

    /* Argument number check. */
    if (argc > N+2)
    {
        ROS_INFO("Too many arguments.\n");
        return false;
    }
    if (argc < 3)
    {
        ROS_INFO("Too few arguments.\n");
        return false;
    }

    /* Load point clouds. */

    return true;
}

void Task2::run()
{
    
}