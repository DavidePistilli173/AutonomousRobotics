#include "Task1.hpp"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    Task1 task;
    if (!task.init(argc, argv))
    {
        ROS_INFO("Failed to initialise. Terminating...\n");
        return -1;
    }
    task.run();
    return 0;
}