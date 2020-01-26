#include <ros/ros.h>

#include "Task3.hpp"

int main(int argc, char** argv)
{
    Task3 task;
    if (!task.init(argc, argv))
    {
        ROS_ERROR("Unable to intialise program.");
        return -1;
    }
    task.run();
    return 0;
}