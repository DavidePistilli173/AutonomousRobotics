#include <ros/ros.h>

#include "Task2.hpp"

int main(int argc, char** argv)
{
    Task2 task;
    if (!task.init(argc, argv))
    {
        ROS_ERROR("Failed to initialise. Terminating...\n");
        return -1;
    }
    task.run();
    return 0;
}